module phy_smi_helper(
    input logic clk,
    input  logic rst,
    input  logic mdclk,
    output logic phyrst,
    output logic phy_rdy,
    output logic phy_full_duplex,
    inout  wire mdio
);
assign phyrst = rphyrst;
logic rphyrst;

logic SMI_trg;
logic SMI_ack;
logic SMI_ready;
logic SMI_rw;
logic [4:0] SMI_adr;
logic [15:0] SMI_data;
logic [15:0] SMI_wdata;

byte SMI_status;

// IEEE 802.3 Clause 22 standard registers.
localparam logic [4:0] PHY_REG_BMCR = 5'd0;
localparam logic [4:0] PHY_REG_BMSR = 5'd1;
localparam logic [4:0] PHY_REG_ANAR = 5'd4;
localparam logic [4:0] PHY_REG_ANLPAR = 5'd5;

// ANAR[8] = 1: advertise 100BASE-TX full duplex only.
// ANAR[4:0] = 5'b00001: IEEE 802.3 selector field.
localparam logic [15:0] ANAR_100M_FULL_DUPLEX_ONLY = 16'h0101;

// BMCR[12] = 1: enable auto-negotiation
// BMCR[9]  = 1: restart auto-negotiation (self-clearing)
localparam logic [15:0] BMCR_ENABLE_RESTART_AN = 16'h1200;

logic link_partner_100m_full_duplex;

always_ff@(posedge mdclk or negedge rst)begin
    if(rst == 1'b0)begin
        phy_rdy <= 1'b0;
        phy_full_duplex <= 1'b0;
        rphyrst <= 1'b0;
        SMI_trg <= 1'b0;
        SMI_adr <= PHY_REG_ANAR;
        SMI_wdata <= ANAR_100M_FULL_DUPLEX_ONLY;
        SMI_rw <= 1'b0;
        SMI_status <= 0;
        link_partner_100m_full_duplex <= 1'b0;
    end else begin
        rphyrst <= 1'b1;
        SMI_trg <= 1'b1;
        if(SMI_ack && SMI_ready)begin
            case(SMI_status)
                0:begin
                    // ANAR has been written. Enable and restart negotiation.
                    SMI_adr <= PHY_REG_BMCR;
                    SMI_wdata <= BMCR_ENABLE_RESTART_AN;
                    SMI_rw <= 1'b0;
                    SMI_status <= 1;
                end
                1:begin
                    // Poll BMSR after the BMCR write completes.
                    SMI_adr <= PHY_REG_BMSR;
                    SMI_rw <= 1'b1;
                    SMI_status <= 2;
                end
                2:begin
                    // BMSR[5]: auto-negotiation complete
                    // BMSR[2]: link status (latch-low, so keep polling)
                    if(SMI_data[5] && SMI_data[2])begin
                        phy_rdy <= 1'b1;
                        SMI_adr <= PHY_REG_ANLPAR;
                        SMI_rw <= 1'b1;
                        SMI_status <= 3;
                    end else begin
                        phy_rdy <= 1'b0;
                        phy_full_duplex <= 1'b0;
                        link_partner_100m_full_duplex <= 1'b0;
                        SMI_adr <= PHY_REG_BMSR;
                        SMI_rw <= 1'b1;
                    end
                end
                3:begin
                    // ANLPAR[8] shows that the partner advertised 100BASE-TX
                    // full duplex. Read BMCR next for the resolved duplex bit.
                    link_partner_100m_full_duplex <= SMI_data[8];
                    SMI_adr <= PHY_REG_BMCR;
                    SMI_rw <= 1'b1;
                    SMI_status <= 4;
                end
                4:begin
                    // RTL8201F BMCR[8] reflects the resolved duplex mode after
                    // auto-negotiation. Require the partner ability as well.
                    phy_full_duplex <= SMI_data[8] &&
                                       link_partner_100m_full_duplex;
                    SMI_adr <= PHY_REG_BMSR;
                    SMI_rw <= 1'b1;
                    SMI_status <= 2;
                end
                default:begin
                    phy_rdy <= 1'b0;
                    phy_full_duplex <= 1'b0;
                    link_partner_100m_full_duplex <= 1'b0;
                    SMI_adr <= PHY_REG_ANAR;
                    SMI_wdata <= ANAR_100M_FULL_DUPLEX_ONLY;
                    SMI_rw <= 1'b0;
                    SMI_status <= 0;
                end
            endcase
            end
    end
end
//1 = read, 0 = write
SMI_ct ct(
    .clk(mdclk), .rst(rphyrst), .rw(SMI_rw), .trg(SMI_trg), .ready(SMI_ready), .ack(SMI_ack),
    .phy_adr(5'd1), .reg_adr(SMI_adr),
    .data(SMI_wdata),
    .smi_data(SMI_data),
    .mdio(mdio)
);

endmodule
