#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阀板顺序扫描双吹测试 (新协议, 板 192.168.200.52)
================================================================
孔 0~scan-end(默认43, 共44孔), 两个相邻孔为一组顺序扫描:
  组 = 孔a 吹 blow ms → 静默 gap ms → 孔a+1 吹 blow ms → 静默 group-interval ms
  (0,1) → (2,3) → ... → (42,43) → 回到 (0,1) 循环, 直到 Ctrl+C。
启动时自动初始化: 先发功能码02设置高压时间/占空比(默认1.5ms/50%,
可用 --hv-ms/--duty 改), 回显确认后才进入功能码01吹气循环。

时序口径:
  孔a 发送时刻 = 组起点 t
  孔a+1 发送时刻 = t + blow + gap            (静默 gap 从第1吹结束算)
  下一组起点    = t + blow + gap + blow + group_interval  (500ms 从第2吹结束算)

帧格式(单孔帧): FF 01 | 孔 | 孔 | 延时(2B) | 持续(2B) | CRC32(前8B,大端)
板子每收一条命令约0.2ms后原样回显(12B+零填充), 顺带统计回显对账。

用法:
    python3 valve_pair_test.py                 # 孔0~43, 2ms吹, 孔间隔10ms, 组间隔500ms, 无限
    python3 valve_pair_test.py --count 22      # 正好吹完一轮22组
    python3 valve_pair_test.py --blow 2 --gap 10 --group-interval 500
    python3 valve_pair_test.py --hv-ms 2 --duty 8      # 换初始化参数(高压2ms, 80%)
    python3 valve_pair_test.py --no-params             # 跳过初始化, 沿用板子已有参数
    python3 valve_pair_test.py --params-only           # 只初始化参数不吹气
"""
import socket
import struct
import zlib
import time
import argparse
import collections
import select
import subprocess
import os
import sys
import threading
import signal

BOARD_IP   = '192.168.200.52'
BOARD_PORT = 10100
LOCAL_IP   = '192.168.200.123'
IFACE_HINT = 'enP3p49s0'      # 直连网卡, 本机IP缺失时自动补回


def ensure_local_ip(ip):
    """本机IP在直连网卡上缺失时自动补回 (NetworkManager链路事件会冲掉)."""
    try:
        out = subprocess.run(['ip', '-o', '-4', 'addr', 'show'],
                             capture_output=True, text=True).stdout
        for line in out.splitlines():
            c = line.split()
            if len(c) >= 4 and c[3].split('/')[0] == ip:
                return True
    except Exception:
        pass
    cmd = ['ip', 'addr', 'add', f'{ip}/24', 'dev', IFACE_HINT]
    run = cmd if os.geteuid() == 0 else ['sudo', '-n'] + cmd
    r = subprocess.run(run, capture_output=True, text=True, check=False)
    if r.returncode == 0 or 'exists' in (r.stderr or '').lower():
        print(f'📌 本机 {ip} 缺失, 已自动补加到 [{IFACE_HINT}]')
        return True
    print(f'❌ 本机未配置 {ip}: {(r.stderr or "").strip()}\n'
          f'   请手动: sudo ip addr add {ip}/24 dev {IFACE_HINT}')
    return False


def build_cmd(blow_ms: int, start: int = 0, end: int = None, delay_ms: int = 0) -> bytes:
    """功能码01打开帧(12B): FF 01 | 起始阀 | 结束阀 | 延时(2B) | 持续(2B) | CRC32(前8B,大端).
    注: 文档里 Data2-4 三个保留字节实际不在线上帧中."""
    if end is None:
        end = start
    body = (bytes([0xFF, 0x01, start & 0xFF, end & 0xFF])
            + struct.pack('>HH', delay_ms & 0xFFFF, blow_ms & 0xFFFF))
    return body + (zlib.crc32(body) & 0xFFFFFFFF).to_bytes(4, 'big')


def build_param_cmd(hv_ms: float, duty: int) -> bytes:
    """功能码02设置参数帧(10B): FF 02 | 高压打开时间(2B,0.1ms单位) | 维持占空比(2B,1-10) | CRC32(前6B,大端)."""
    hv = int(round(hv_ms * 10))
    if not 0 <= hv <= 0xFFFF:
        raise ValueError(f'高压时间 {hv_ms}ms 超范围 (0~6553.5ms)')
    if not 1 <= duty <= 10:
        raise ValueError(f'占空比 {duty} 超范围 (1~10)')
    body = bytes([0xFF, 0x02]) + struct.pack('>HH', hv, duty)
    return body + (zlib.crc32(body) & 0xFFFFFFFF).to_bytes(4, 'big')


def wait_until(ns: int, stop_event=None):
    """睡到目标时刻: 粗睡到剩1.5ms, 余下自旋 (本机 sleep 会睡过头2~3ms)."""
    while True:
        if stop_event is not None and stop_event.is_set():
            return False
        rest = ns - time.perf_counter_ns()
        if rest <= 0:
            return True
        if rest > 1_500_000:
            sleep_seconds = (rest - 1_500_000) / 1e9
            if stop_event is None:
                time.sleep(sleep_seconds)
            elif stop_event.wait(sleep_seconds):
                return False
        else:
            while time.perf_counter_ns() < ns:
                if stop_event is not None and stop_event.is_set():
                    return False
            return True


def main():
    ap = argparse.ArgumentParser(description='阀板双吹节奏测试 (每组两吹, Ctrl+C 停)')
    ap.add_argument('--blow', type=int, default=2, help='每孔吹气 ms (默认 2)')
    ap.add_argument('--scan-end', type=int, default=43,
                    help='扫描最后孔号 (默认 43, 即孔0~43共44孔, 22组一轮)')
    ap.add_argument('--gap', type=float, default=10.0,
                    help='组内两孔之间的静默 ms, 从第一吹结束算 (默认 10)')
    ap.add_argument('--group-interval', type=float, default=500.0,
                    help='第二吹结束到下一组的静默 ms (默认 500)')
    ap.add_argument('--echo-timeout', type=float, default=0.5,
                    help='每轮及退出前等待回显的最长秒数 (默认 0.5)')
    ap.add_argument('--count', type=int, default=0, help='组数 (默认 0=无限, Ctrl+C 停)')
    ap.add_argument('--hv-ms', type=float, default=1.5, 
                    help='开阀高压时间 ms (默认 1.5, 启动时自动下发功能码02)')
    ap.add_argument('--duty', type=int, default=5, help='维持占空比 1-10 (默认 5=50%%)')
    ap.add_argument('--no-params', action='store_true',
                    help='跳过初始化, 不发功能码02, 沿用板子已有参数')
    ap.add_argument('--params-only', action='store_true', help='只初始化参数不吹气')
    ap.add_argument('-q', '--quiet', action='store_true', help='连每轮的轮报也不打, 只出最终统计')
    args = ap.parse_args()

    if not 0 <= args.blow <= 65535:
        sys.exit(f'❌ 吹气 {args.blow}ms 超范围 (0~65535)')
    if not 1 <= args.scan_end <= 63:
        sys.exit(f'❌ --scan-end 超范围 1~63 (当前 {args.scan_end})')
    if args.gap < 0 or args.group_interval < 0:
        sys.exit('❌ --gap 和 --group-interval 不能为负数')
    if args.echo_timeout < 0:
        sys.exit('❌ --echo-timeout 不能为负数')
    if args.no_params and args.params_only:
        sys.exit('❌ --no-params 与 --params-only 互相矛盾')
    if not ensure_local_ip(LOCAL_IP):
        sys.exit(1)

    # 扫描编排: 孔0~scan-end 两孔一组, 每孔预构好自己的单孔帧
    pairs = [(h, min(h + 1, args.scan_end)) for h in range(0, args.scan_end + 1, 2)]
    frames = {h: build_cmd(args.blow, h, h) for h in range(args.scan_end + 1)}
    frame_set = set(frames.values())
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Give the receiver enough headroom even if the Python thread is briefly
    # delayed by timing/printing work.
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    s.bind((LOCAL_IP, 0))
    s.setblocking(False)
    board = (BOARD_IP, BOARD_PORT)

    print(f'✅ 就绪: {s.getsockname()[0]}:{s.getsockname()[1]} -> {BOARD_IP}:{BOARD_PORT}')

    # ---- 初始化: 功能码02 自动设置参数 (要等回显确认, 不确认不吹) ----
    if not args.no_params:
        pcmd = build_param_cmd(args.hv_ms, args.duty)
        print(f'⚙️  初始化 TX[{len(pcmd)}B]: {pcmd.hex(" ").upper()}  '
              f'(功能码02: 高压 {args.hv_ms}ms, 占空比 {args.duty * 10}%)')
        s.sendto(pcmd, board)
        deadline = time.monotonic() + 0.5
        param_echoed = False
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            readable, _, _ = select.select([s], [], [], remaining)
            if not readable:
                break
            data, addr = s.recvfrom(2048)
            if addr == board and len(data) >= 10 and data[:10] == pcmd:
                print('   参数帧已发出且板子回显一致 ✅')
                param_echoed = True
                break
        if not param_echoed:
            s.close()
            sys.exit(f'❌ 参数帧 {pcmd.hex(" ").upper()} 0.5s内无回显, 不开始吹气')
        if args.params_only:
            print('   (--params-only: 只初始化参数, 不吹气)')
            s.close()
            return
    print(f'📡 扫描: 孔0~{args.scan_end} (共{args.scan_end + 1}孔, {len(pairs)}组/轮), '
          f'组=(孔a吹{args.blow}ms → 静默{args.gap}ms → 孔a+1吹{args.blow}ms → 等{args.group_interval}ms)')
    print(f'   单孔帧示例 (孔0): {frames[0].hex(" ").upper()}')
    print(f'   扫完一轮后回到孔0 继续, Ctrl+C 停止\n')

    # Each transmitted frame is registered before the receive thread can
    # account its echo. The protocol has no sequence field, so repeated copies
    # of the same valve command are matched to the oldest outstanding send.
    pending_by_frame = {
        frame: collections.deque() for frame in frame_set
    }
    round_tx = collections.Counter()
    round_echo = collections.Counter()
    round_rtt_ms = collections.defaultdict(list)
    stats_condition = threading.Condition()
    receiver_stop = threading.Event()
    stop_requested = threading.Event()
    stop_notice_printed = False

    def request_safe_stop(_signum=None, _frame=None):
        """Request a stop without interrupting send/accounting mid-group."""
        nonlocal stop_notice_printed
        stop_requested.set()
        if not stop_notice_printed:
            stop_notice_printed = True
            print('\n⏹ 收到 Ctrl+C：停止新组，正在完成当前组并等待已发送回显...')

    previous_sigint_handler = signal.signal(signal.SIGINT, request_safe_stop)

    groups = tx = echo = 0
    late_or_duplicate = ignored = receive_errors = 0
    cycles = 0                          # 完整扫过 0~scan-end 的轮数
    intra_ms, period_ms, all_rtt_ms = [], [], []
    r_intra, r_period = [], []
    finalized_rounds = set()

    def receiver_loop():
        """Continuously receive echoes so timing waits cannot hide packets."""
        nonlocal echo, late_or_duplicate, ignored, receive_errors

        while not receiver_stop.is_set():
            try:
                readable, _, _ = select.select([s], [], [], 0.05)
                if not readable:
                    continue
                data, addr = s.recvfrom(2048)
            except BlockingIOError:
                continue
            except OSError:
                if not receiver_stop.is_set():
                    with stats_condition:
                        receive_errors += 1
                continue

            now_ns = time.perf_counter_ns()
            frame = data[:12]
            with stats_condition:
                if addr != board or len(data) < 12 or frame not in pending_by_frame:
                    ignored += 1
                elif pending_by_frame[frame]:
                    round_id, sent_ns = pending_by_frame[frame].popleft()
                    rtt_ms = (now_ns - sent_ns) / 1e6
                    echo += 1
                    round_echo[round_id] += 1
                    round_rtt_ms[round_id].append(rtt_ms)
                    all_rtt_ms.append(rtt_ms)
                    stats_condition.notify_all()
                else:
                    late_or_duplicate += 1

    def send_command(frame, round_id):
        """Send one command and atomically register the expected echo."""
        nonlocal tx

        sent_ns = time.perf_counter_ns()
        with stats_condition:
            sent_bytes = s.sendto(frame, board)
            if sent_bytes != len(frame):
                raise OSError(
                    f'UDP只发送了 {sent_bytes}/{len(frame)} 字节'
                )
            pending_by_frame[frame].append((round_id, sent_ns))
            round_tx[round_id] += 1
            tx += 1

    def wait_for_round(round_id):
        """Wait for one round's echoes, then expire only its missing entries."""
        deadline = time.monotonic() + args.echo_timeout

        with stats_condition:
            while round_echo[round_id] < round_tx[round_id]:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                stats_condition.wait(remaining)

            sent_count = round_tx[round_id]
            echo_count = round_echo[round_id]
            timeout_count = sent_count - echo_count
            rtt_values = list(round_rtt_ms[round_id])

            # Do not let a timed-out echo get matched to the same valve command
            # in a later scan round.
            if timeout_count:
                for queue in pending_by_frame.values():
                    retained = [item for item in queue if item[0] != round_id]
                    queue.clear()
                    queue.extend(retained)

        finalized_rounds.add(round_id)
        return sent_count, echo_count, timeout_count, rtt_values

    receiver_thread = threading.Thread(
        target=receiver_loop,
        name='valve-echo-receiver',
        daemon=True,
    )
    receiver_thread.start()

    t_group = time.perf_counter_ns() + 200_000_000     # 0.2s 后开始, 给打印留时间
    try:
        while ((args.count == 0 or groups < args.count) and
               not stop_requested.is_set()):
            for a, b in pairs:          # (0,1) (2,3) ... (42,43), 一轮后 for 自然回卷重来
                if stop_requested.is_set() or (args.count and groups >= args.count):
                    break
                round_id = cycles + 1
                # Safe boundary: no command from this group has been sent.
                if not wait_until(t_group, stop_requested):
                    break
                t1 = time.perf_counter_ns()
                send_command(frames[a], round_id)
                # After the first send, always finish the pair. The SIGINT
                # handler only sets a flag, so this section cannot be torn in
                # half by Ctrl+C.
                wait_until(t1 + int((args.blow + args.gap) * 1e6))   # 孔b = t + blow + gap
                t2 = time.perf_counter_ns()
                send_command(frames[b], round_id)
                groups += 1
                intra = (t2 - t1) / 1e6
                intra_ms.append(intra); r_intra.append(intra)
                # 后台线程持续收回显；主线程只负责精确的阀门发送节奏。
                t_next = t2 + int((args.blow + args.group_interval) * 1e6)
                wait_until(t_next, stop_requested)
                period = (t_next - t_group) / 1e6
                period_ms.append(period); r_period.append(period)
                t_group = t_next
                if b == args.scan_end:      # 本轮扫完, 孔43 已吹
                    wait_started_ns = time.perf_counter_ns()
                    r_tx, r_echo, r_timeout, r_rtt = wait_for_round(round_id)
                    cycles += 1
                    if not args.quiet and r_period:
                        rtt_text = ''
                        if r_rtt:
                            rtt_text = (
                                f', RTT {min(r_rtt):.3f}/'
                                f'{sum(r_rtt) / len(r_rtt):.3f}/'
                                f'{max(r_rtt):.3f}ms'
                            )
                        print(f'[{time.strftime("%H:%M:%S")}] ↩️  第{cycles}轮扫完 '
                              f'(孔0~{args.scan_end}, {len(r_period)}组/{r_tx}吹): '
                              f'回显 {r_echo}/{r_tx}, 超时 {r_timeout}{rtt_text}, '
                              f'两孔实隔 {min(r_intra):.2f}~{max(r_intra):.2f}ms '
                              f'(目标{args.blow + args.gap:.0f}), '
                              f'组周期 {min(r_period):.1f}~{max(r_period):.1f}ms '
                              f'(目标{args.blow * 2 + args.gap + args.group_interval:.0f})')
                    r_intra, r_period = [], []

                    # A timeout wait intentionally pauses the scan. Restart
                    # scheduling from now instead of catching up in a burst.
                    if time.perf_counter_ns() - wait_started_ns > 1_000_000:
                        t_group = time.perf_counter_ns()
    except KeyboardInterrupt:
        request_safe_stop()
        print('\n⏹  收到 Ctrl+C, 停止')

    # A finite --count or Ctrl+C can stop in the middle of a round. Give every
    # command from that partial round the same grace period before declaring a
    # timeout.
    for round_id in sorted(round_tx):
        if round_id not in finalized_rounds:
            wait_for_round(round_id)

    receiver_stop.set()
    receiver_thread.join(timeout=0.2)
    signal.signal(signal.SIGINT, previous_sigint_handler)

    lost = tx - echo
    print(f'\n📊 统计: 共 {groups} 组 / {tx} 条命令, 完整扫描 {cycles} 轮, '
          f'回显 {echo} 条, 超时 {lost} 条')
    if all_rtt_ms:
        print(f'   回显RTT ms: min={min(all_rtt_ms):.3f} '
              f'均值={sum(all_rtt_ms)/len(all_rtt_ms):.3f} '
              f'max={max(all_rtt_ms):.3f}')
    if late_or_duplicate or ignored or receive_errors:
        print(f'   异常回包: 迟到/重复={late_or_duplicate}, '
              f'非本测试={ignored}, 接收错误={receive_errors}')
    if intra_ms:
        print(f'   两孔实隔 ms: min={min(intra_ms):.2f} 均值={sum(intra_ms)/len(intra_ms):.2f} '
              f'max={max(intra_ms):.2f} (目标 {args.blow + args.gap:.0f})')
    if period_ms:
        print(f'   组周期   ms: min={min(period_ms):.2f} 均值={sum(period_ms)/len(period_ms):.2f} '
              f'max={max(period_ms):.2f} (目标 {args.blow * 2 + args.gap + args.group_interval:.0f})')
    s.close()


if __name__ == '__main__':
    main()
