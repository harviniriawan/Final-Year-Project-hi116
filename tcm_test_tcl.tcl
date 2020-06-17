# Rebuild Project to initialise seed
set proj_path [lindex $argv 1]
setws $proj_path
getws
projects -build -type app -name A53_INJ
###################################################################################
connect -url tcp:127.0.0.1:3121
source C:/Xilinx/SDK/2019.1/scripts/sdk/util/zynqmp_utils.tcl
targets -set -nocase -filter {name =~"RPU*" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 1
# enable_split_mode
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
targets -set -nocase -filter {name =~"APU*" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 1
enable_a32_mode 0
targets -set -nocase -filter {name =~"*A53*0" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 1
rst -processor
dow C:/Users/harvi/Final-Year-Project-hi116/A53_INJ/Debug/A53_INJ.elf
targets -set -nocase -filter {name =~"*R5*0" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 1
rst -processor
catch {XFsbl_TcmEccInit R5_0}
set kernel [lindex $argv 0]
dow C:/Users/harvi/Final-Year-Project-hi116/$kernel/Debug/$kernel.elf
configparams force-mem-access 0
targets -set -nocase -filter {name =~"*A53*0" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 1
con
targets -set -nocase -filter {name =~"*R5*0" && jtag_cable_name =~ "Avnet USB-to-JTAG/UART Pod V1 1234-oj1A"} -index 1
con