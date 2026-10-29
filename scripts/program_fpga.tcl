# program_fpga.tcl - load a bitstream into the Basys 3 over its USB-JTAG.
#
#   vivado -mode batch -nojournal -nolog -source scripts/program_fpga.tcl
#   vivado -mode batch -nojournal -nolog -source scripts/program_fpga.tcl -tclargs path/to/file.bit
#
# Programs the FPGA's configuration RAM (lost on power cycle).
# Alternative without Vivado:  openFPGALoader -b basys3 build/vivado/basys3_link.bit

set root [file normalize [file join [file dirname [info script]] ..]]
if {[llength $argv] > 0} {
    set bit [file normalize [lindex $argv 0]]
} else {
    set bit [file join $root build vivado basys3_link.bit]
}
if {![file exists $bit]} {
    puts "RESULT: FAILED - bitstream not found: $bit (run build_bitstream.tcl first)"
    exit 1
}

open_hw_manager
connect_hw_server
set targets [get_hw_targets]
if {[llength $targets] == 0} {
    puts "RESULT: FAILED - no JTAG target found. Is the Basys 3 plugged in and switched on?"
    exit 1
}
open_hw_target [lindex $targets 0]

set devs [get_hw_devices xc7a35t*]
if {[llength $devs] == 0} {
    puts "RESULT: FAILED - no xc7a35t on the JTAG chain (found: [get_hw_devices])"
    exit 1
}
set dev [lindex $devs 0]
current_hw_device $dev
refresh_hw_device -update_hw_probes false $dev
set_property PROGRAM.FILE $bit $dev
program_hw_devices $dev

set done "unknown"
catch {
    refresh_hw_device -update_hw_probes false $dev
    set done [get_property REGISTER.CONFIG_STATUS.BIT14_DONE_PIN $dev]
}
close_hw_target
disconnect_hw_server
close_hw_manager

if {$done == 1} {
    puts "RESULT: OK - programmed $bit (DONE high)"
} else {
    puts "RESULT: PROGRAMMED, but DONE status read as '$done' - check the board's DONE LED"
}
