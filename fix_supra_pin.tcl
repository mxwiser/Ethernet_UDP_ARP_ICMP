# Normalize Quartus handoff files for the Supra compatible flow.
#
# Quartus writes bidirectional directions as "bidir" in its PIN report, while
# Supra accepts the Verilog-style keyword "inout". Quartus can also label an
# electrically bidirectional top-level port as input or output in the VO file.
# This script discovers every bidirectional port from the PIN report and fixes
# both files without relying on design-specific port names.
#
# To use it in another project, copy this file beside the QSF and add:
# set_global_assignment -name POST_FLOW_SCRIPT_FILE \
#     "quartus_sh:fix_supra_pin.tcl"

proc find_existing_file {candidates} {
    foreach candidate $candidates {
        if {[file exists $candidate]} {
            return [file normalize $candidate]
        }
    }
    return ""
}

proc base_port_name {pin_name} {
    set base_name [string trim $pin_name]

    # A Verilog escaped identifier starts with a backslash. The PIN report does
    # not normally use it, but accepting it makes the script more portable.
    if {[string length $base_name] > 0 &&
        [string index $base_name 0] eq "\\"} {
        set base_name [string range $base_name 1 end]
    }

    # Convert bus bits such as gpio[3] to their VO declaration name gpio.
    while {[regsub {\[[^][]+\]$} $base_name "" stripped_name]} {
        set base_name $stripped_name
    }

    return $base_name
}

set project_name [lindex $quartus(args) 1]
set revision_name [lindex $quartus(args) 2]

if {$revision_name eq ""} {
    set revision_name $project_name
}

set pin_candidates [list \
    [file join "output_files" "${revision_name}.pin"] \
    [file join "quartus_logs" "${revision_name}.pin"] \
    "${revision_name}.pin"]

set vo_candidates [list \
    [file join "simulation" "modelsim" "${revision_name}.vo"] \
    [file join "simulation" "questa" "${revision_name}.vo"] \
    [file join "simulation" "questa_sim" "${revision_name}.vo"] \
    [file join "simulation" "verilog" "${revision_name}.vo"] \
    "${revision_name}.vo"]

# Also accept custom one-level PIN and one/two-level VO output directories.
foreach candidate [glob -nocomplain -types f \
    [file join "*" "${revision_name}.pin"]] {
    lappend pin_candidates $candidate
}
foreach candidate [glob -nocomplain -types f \
    [file join "*" "${revision_name}.vo"] \
    [file join "*" "*" "${revision_name}.vo"]] {
    lappend vo_candidates $candidate
}

set pin_file [find_existing_file $pin_candidates]
if {$pin_file eq ""} {
    post_message -type warning \
        "Supra compatibility: cannot find ${revision_name}.pin"
    return
}

set input_file [open $pin_file rb]
set pin_contents [read $input_file]
close $input_file

# Build the port list before replacing "bidir", while also accepting a PIN
# file already normalized by an earlier invocation (idempotent operation).
array set bidirectional_ports {}
foreach pin_line [split $pin_contents "\n"] {
    set fields [split $pin_line ":"]
    if {[llength $fields] < 4} {
        continue
    }

    set pin_name [string trim [lindex $fields 0]]
    set direction [string tolower [string trim [lindex $fields 2]]]
    if {$pin_name eq "" ||
        ($direction ne "bidir" && $direction ne "inout")} {
        continue
    }

    set bidirectional_ports([base_port_name $pin_name]) 1
}

if {[array size bidirectional_ports] == 0} {
    post_message -type info \
        "Supra compatibility: no bidirectional ports found in $pin_file"
    return
}

set pin_replacement_count [regsub -all -nocase \
    {(:[ \t]*)bidir([ \t]*:)} $pin_contents \
    {\1inout\2} supra_pin_contents]

if {$pin_replacement_count > 0} {
    set output_file [open $pin_file wb]
    puts -nonewline $output_file $supra_pin_contents
    close $output_file
}

set vo_file [find_existing_file $vo_candidates]
if {$vo_file eq ""} {
    post_message -type warning \
        "Supra compatibility: cannot find ${revision_name}.vo"
    return
}

set input_file [open $vo_file rb]
set vo_contents [read $input_file]
close $input_file

set top_module_started 0
set top_module_finished 0
set vo_replacement_count 0
set vo_inout_count 0
array set matched_ports {}
set supra_vo_lines {}

# Quartus puts the top-level design first in the VO. Only edit declarations in
# that module, so identically named ports in primitive/library modules remain
# untouched.
foreach vo_line [split $vo_contents "\n"] {
    set updated_line $vo_line

    if {!$top_module_started &&
        [regexp {^[ \t]*module[ \t]+} $vo_line]} {
        set top_module_started 1
    }

    if {$top_module_started && !$top_module_finished} {
        set trimmed_line [string trim $vo_line]
        if {[regexp \
            {^(input|output|inout)[ \t]+(.+);[ \t\r]*$} \
            $trimmed_line -> direction declaration]} {
            set declaration_tokens [regexp -all -inline {\S+} $declaration]
            set declared_name [string trimright \
                [lindex $declaration_tokens end] ","]
            set lookup_name [base_port_name $declared_name]

            if {[info exists bidirectional_ports($lookup_name)]} {
                set matched_ports($lookup_name) 1
                if {$direction eq "inout"} {
                    incr vo_inout_count
                } else {
                    regsub \
                        {^([ \t]*)(input|output)([ \t]+)} \
                        $vo_line {\1inout\3} updated_line
                    incr vo_replacement_count
                }
            }
        }

        if {[regexp {^[ \t]*endmodule\M} $vo_line]} {
            set top_module_finished 1
        }
    }

    lappend supra_vo_lines $updated_line
}

if {$vo_replacement_count > 0} {
    set output_file [open $vo_file wb]
    puts -nonewline $output_file [join $supra_vo_lines "\n"]
    close $output_file
}

set missing_ports {}
foreach port_name [lsort [array names bidirectional_ports]] {
    if {![info exists matched_ports($port_name)]} {
        lappend missing_ports $port_name
    }
}

post_message -type info \
    "Supra compatibility: converted $pin_replacement_count PIN direction(s) and $vo_replacement_count VO declaration(s); $vo_inout_count VO declaration(s) already inout"

if {[llength $missing_ports] > 0} {
    post_message -type warning \
        "Supra compatibility: bidirectional PIN port(s) not found in the first VO module: [join $missing_ports {, }]"
}
