set proj_path [lindex $argv 1]
set kernel [lindex $argv 0]

setws $proj_path
getws
importprojects $proj_path
getprojects
projects -build