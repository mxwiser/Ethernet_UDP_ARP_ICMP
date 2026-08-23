# Quartus reports bidirectional pins as "bidir", while the Supra 2026.06
# compatible-flow PIN parser accepts the Verilog-style keyword "inout".
# Quartus can also label an electrically bidirectional top-level port as an
# output in the simulation VO. Normalize both handoff files after compilation.

set project_name [lindex $quartus(args) 1]
set revision_name [lindex $quartus(args) 2]

if {$revision_name eq ""} {
    set revision_name $project_name
}

set pin_file [file normalize [file join "output_files" "${revision_name}.pin"]]
set vo_file [file normalize \
    [file join "simulation" "modelsim" "${revision_name}.vo"]]

if {![file exists $pin_file]} {
    post_message -type warning "Supra PIN compatibility: file not found: $pin_file"
    return
}

set input_file [open $pin_file rb]
set pin_contents [read $input_file]
close $input_file

# Limit the replacement to a colon-delimited direction field, so comments and
# signal names containing the word "bidir" remain unchanged.
set replacement_count [regsub -all {(:[ \t]*)bidir([ \t]*:)} \
    $pin_contents {\1inout\2} supra_pin_contents]

if {$replacement_count == 0} {
    post_message -type info "Supra PIN compatibility: no bidir fields found in $pin_file"
} else {
    set output_file [open $pin_file wb]
    puts -nonewline $output_file $supra_pin_contents
    close $output_file

    post_message -type info \
        "Supra PIN compatibility: converted $replacement_count bidir field(s) to inout in $pin_file"
}

if {![file exists $vo_file]} {
    post_message -type warning "Supra VO compatibility: file not found: $vo_file"
    return
}

set input_file [open $vo_file rb]
set vo_contents [read $input_file]
close $input_file

# The explicit RTL I/O path has both an input buffer and an output buffer with
# dynamic OE. Correct only the scalar top-level declaration that Quartus can
# nevertheless emit as "output mdio" during an incremental compilation.
set vo_replacement_count [regsub -all -line \
    {^output[ \t]+mdio[ \t]*;} $vo_contents \
    {inout  mdio;} supra_vo_contents]

if {$vo_replacement_count > 0} {
    set output_file [open $vo_file wb]
    puts -nonewline $output_file $supra_vo_contents
    close $output_file
    post_message -type info \
        "Supra VO compatibility: converted output mdio to inout mdio in $vo_file"
} elseif {[regexp -line {^inout[ \t]+mdio[ \t]*;} $vo_contents]} {
    post_message -type info \
        "Supra VO compatibility: mdio is already inout in $vo_file"
} else {
    post_message -type warning \
        "Supra VO compatibility: no scalar mdio declaration found in $vo_file"
}
