# argv 0 = Lock/split mode
# argv 1 = ECC ON/OFF (TCM cache, cache, DDR, TCM)
# argv 2 = Use the PMU or not
# argv 3 = Name of the kernel to be tested
if {$argc != 4} {
	puts "NOT ENOUGH ARGUMENTS PUT IN"
	return 1
} else {
	connect -url tcp:127.0.0.1:3121
	source C:/Xilinx/SDK/2019.1/scripts/sdk/util/zynqmp_utils.tcl
	targets -set -nocase -filter {name =~"RPU*" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 1
	clear_rpu_reset
	if {[lindex $argv 0] == "lock"} {
		puts "R5 Run In Lock-Step"
	} else {
		puts "Run in Split Mode"
		enable_split_mode
	}
	if {[lindex $argv 2] == "pmu_on"} {
		puts "disable PMU gate"
		targets -set -nocase -filter {name =~"PSU*" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 1
		catch {disable_pmu_gate}
	} else {puts "Not disabling PMU gate"}
	targets -set -nocase -filter {name =~"APU*" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 1
	loadhw -hw C:/Users/harvi/Final-Year-Project-hi116/design_1_wrapper_hw_platform_0/system.hdf -mem-ranges [list {0x80000000 0xbfffffff} {0x400000000 0x5ffffffff} {0x1000000000 0x7fffffffff}]
	configparams force-mem-access 1
	targets -set -nocase -filter {name =~"APU*" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 1
	source C:/Users/harvi/Final-Year-Project-hi116/design_1_wrapper_hw_platform_0/psu_init.tcl
	psu_init
	source C:/Xilinx/SDK/2019.1/scripts/sdk/util/fsbl.tcl
	after 1000
	psu_ps_pl_isolation_removal
	after 1000
	psu_ps_pl_reset_config
	catch {psu_protection}
	targets -set -nocase -filter {name =~"*R5*0" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 1
	rst -processor
	# If ECC is on
	if {[lindex $argv 1] == "ecc_on"} {
		puts "ECC is enabled, enable DDR and TCM ECC accordingly"
		puts "DDR ECC enable..."
		catch {XFsbl_DdrEccInit}
		if {[lindex $argv 0] == "lock"} {
			puts "INIT TCM ECC LOCK-STEP"
			catch {XFsbl_TcmEccInit R5_L}
		} elseif {[lindex $argv 0] == "split"} {
			puts "INIT TCM ECC of Core 0 only, as test operates in split mode"
			catch {XFsbl_TcmEccInit R5_0}
		}
	} else {puts "No ECC protection is enabled"}
	# set kernel to be downloaded by device
	set kernel [lindex $argv 3]
	dow C:/Users/harvi/Final-Year-Project-hi116/$kernel/Debug/$kernel.elf
	# decide whether to download pmu or not
	if {[lindex $argv 2] == "pmu_on"} {
		puts "PMU is also downloaded by the board"
		targets -set -nocase -filter {name =~"MicroBlaze*PMU*" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 0
		catch {stop}
		rst -processor
		dow C:/Users/harvi/Final-Year-Project-hi116/pmu/Debug/pmu.elf
	} else {puts "PMU is not downloaded, running without PMU"}
	configparams force-mem-access 0
	bpadd -addr &exit
	if {[lindex $argv 2] == "pmu_on"} {
		targets -set -nocase -filter {name =~"MicroBlaze*PMU*" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 0
		con
	}
	targets -set -nocase -filter {name =~"*R5*0" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 1
	con
}

puts "WRITE TO PMU MASK REG_EN"
# CCF
mwr -force 0xffd80538 [expr 1<<9]
# DDR
mwr -force 0xffd80538 1

# Disable OCM, XMPU and XPPU Error
puts "WRITE TO PMU MASK REG DIS"
mwr -force 0xFFD8053C [expr 1<<25]
mwr -force 0xFFD8053C [expr 1<<24]
mwr -force 0xFFD8053C [expr 1<<1]

puts "WRITE TO RPU0_IEN"
mwr -force 0xff9a011c 0x1fffe

puts "DONE CONFIG"