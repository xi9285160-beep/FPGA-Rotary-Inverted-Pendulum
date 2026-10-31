# ============================================================================
# build_check.tcl — 命令行综合 + 布局布线（PIDSYB 工程）
#
#   cd PIDSYB
#   gw_sh.exe build_check.tcl
#
# 报告输出：
#   impl/gwsynthesis/FPGA_Pendulum_syn.rpt.html   综合报告
#   impl/pnr/FPGA_Pendulum.rpt.txt                资源与管脚报告
#   impl/pnr/FPGA_Pendulum_tr_content.html        时序报告
#   impl/pnr/FPGA_Pendulum.bin / .fs              比特流
# ============================================================================
open_project FPGA_Pendulum.gprj
run all
puts "=== BUILD DONE ==="
exit
