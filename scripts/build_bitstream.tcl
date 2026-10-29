# build_bitstream.tcl - non-project Vivado build for the Basys 3 link.
#
#   vivado -mode batch -nojournal -nolog -source scripts/build_bitstream.tcl
#
# Output: build/vivado/basys3_link.bit, timing.rpt, utilization.rpt
# Prints a RESULT line with WNS so a script can check that timing was met.

set root [file normalize [file join [file dirname [info script]] ..]]
set out  [file join $root build vivado]
set part xc7a35tcpg236-1
file mkdir $out

read_verilog [glob [file join $root rtl *.v]]
read_xdc [file join $root constraints basys3_link.xdc]

synth_design -top basys3_top -part $part
opt_design
place_design
route_design

report_timing_summary -file [file join $out timing.rpt]
report_utilization    -file [file join $out utilization.rpt]

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set whs [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]]

write_bitstream -force [file join $out basys3_link.bit]

if {$wns < 0 || $whs < 0} {
    puts "RESULT: TIMING FAILED (WNS $wns ns, WHS $whs ns) - see build/vivado/timing.rpt"
} else {
    puts "RESULT: OK (WNS $wns ns, WHS $whs ns) - bitstream build/vivado/basys3_link.bit"
}
