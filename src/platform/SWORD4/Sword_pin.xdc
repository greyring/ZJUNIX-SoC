#set_property -dict {PACKAGE_PIN AC18 IOSTANDARD LVDS} [get_ports sysclk_p]
#set_property -dict {IOSTANDARD LVDS} [get_ports sysclk_n]
set_property -dict {PACKAGE_PIN W13 IOSTANDARD LVCMOS18} [get_ports rstn]

set_property -dict {PACKAGE_PIN M24 IOSTANDARD LVCMOS33} [get_ports {seg_sout[2]}]
set_property -dict {PACKAGE_PIN L24 IOSTANDARD LVCMOS33} [get_ports {seg_sout[1]}]
set_property -dict {PACKAGE_PIN R18 IOSTANDARD LVCMOS33} [get_ports {seg_sout[0]}]

set_property -dict {PACKAGE_PIN N26 IOSTANDARD LVCMOS33} [get_ports {led_sout[1]}]
set_property -dict {PACKAGE_PIN M26 IOSTANDARD LVCMOS33} [get_ports {led_sout[0]}]

set_property -dict {PACKAGE_PIN AA22 IOSTANDARD LVCMOS33} [get_ports {segment[7]}]
set_property -dict {PACKAGE_PIN AC23 IOSTANDARD LVCMOS33} [get_ports {segment[6]}]
set_property -dict {PACKAGE_PIN AC24 IOSTANDARD LVCMOS33} [get_ports {segment[5]}]
set_property -dict {PACKAGE_PIN W20  IOSTANDARD LVCMOS33} [get_ports {segment[4]}]
set_property -dict {PACKAGE_PIN Y21  IOSTANDARD LVCMOS33} [get_ports {segment[3]}]
set_property -dict {PACKAGE_PIN AD23 IOSTANDARD LVCMOS33} [get_ports {segment[2]}]
set_property -dict {PACKAGE_PIN AD24 IOSTANDARD LVCMOS33} [get_ports {segment[1]}]
set_property -dict {PACKAGE_PIN AB22 IOSTANDARD LVCMOS33} [get_ports {segment[0]}]

#set_property -dict {PACKAGE_PIN AD21 IOSTANDARD LVCMOS33} [get_ports {anode[3]}]
#set_property -dict {PACKAGE_PIN AC21 IOSTANDARD LVCMOS33} [get_ports {anode[2]}]
set_property -dict {PACKAGE_PIN AB21 IOSTANDARD LVCMOS33} [get_ports {anode[1]}]
set_property -dict {PACKAGE_PIN AC22 IOSTANDARD LVCMOS33} [get_ports {anode[0]}]
set_property -dict {PACKAGE_PIN AD21 IOSTANDARD LVCMOS33} [get_ports {btnR}]
set_property -dict {PACKAGE_PIN AC21 IOSTANDARD LVCMOS33} [get_ports {btnL}]

set_property -dict {PACKAGE_PIN AF24 IOSTANDARD LVCMOS33} [get_ports {LED[7]}]
set_property -dict {PACKAGE_PIN AE21 IOSTANDARD LVCMOS33} [get_ports {LED[6]}]
set_property -dict {PACKAGE_PIN Y22  IOSTANDARD LVCMOS33} [get_ports {LED[5]}]
set_property -dict {PACKAGE_PIN Y23  IOSTANDARD LVCMOS33} [get_ports {LED[4]}]
set_property -dict {PACKAGE_PIN AA23 IOSTANDARD LVCMOS33} [get_ports {LED[3]}]
set_property -dict {PACKAGE_PIN Y25  IOSTANDARD LVCMOS33} [get_ports {LED[2]}]
set_property -dict {PACKAGE_PIN AB26 IOSTANDARD LVCMOS33} [get_ports {LED[1]}]
set_property -dict {PACKAGE_PIN W23  IOSTANDARD LVCMOS33} [get_ports {LED[0]}]

#set_property -dict {PACKAGE_PIN AF24 IOSTANDARD LVCMOS33} [get_ports buzzer]

set_property -dict {PACKAGE_PIN V16 IOSTANDARD LVCMOS18 PULLUP true} [get_ports {btnY[4]}]
set_property -dict {PACKAGE_PIN W14 IOSTANDARD LVCMOS18 PULLUP true} [get_ports {btnY[3]}]
set_property -dict {PACKAGE_PIN V14 IOSTANDARD LVCMOS18 PULLUP true} [get_ports {btnY[2]}]
set_property -dict {PACKAGE_PIN V19 IOSTANDARD LVCMOS18 PULLUP true} [get_ports {btnY[1]}]
set_property -dict {PACKAGE_PIN V18 IOSTANDARD LVCMOS18 PULLUP true} [get_ports {btnY[0]}]
set_property -dict {PACKAGE_PIN W16 IOSTANDARD LVCMOS18 PULLUP true} [get_ports {btnX[4]}]
set_property -dict {PACKAGE_PIN W15 IOSTANDARD LVCMOS18 PULLUP true} [get_ports {btnX[3]}]
set_property -dict {PACKAGE_PIN W19 IOSTANDARD LVCMOS18 PULLUP true} [get_ports {btnX[2]}]
set_property -dict {PACKAGE_PIN W18 IOSTANDARD LVCMOS18 PULLUP true} [get_ports {btnX[1]}]
set_property -dict {PACKAGE_PIN V17 IOSTANDARD LVCMOS18 PULLUP true} [get_ports {btnX[0]}]

set_property -dict {PACKAGE_PIN AA10 IOSTANDARD LVCMOS15} [get_ports {SW[0]}]
set_property -dict {PACKAGE_PIN AB10 IOSTANDARD LVCMOS15} [get_ports {SW[1]}]
set_property -dict {PACKAGE_PIN AA13 IOSTANDARD LVCMOS15} [get_ports {SW[2]}]
set_property -dict {PACKAGE_PIN AA12 IOSTANDARD LVCMOS15} [get_ports {SW[3]}]
set_property -dict {PACKAGE_PIN Y13  IOSTANDARD LVCMOS15} [get_ports {SW[4]}]
set_property -dict {PACKAGE_PIN Y12  IOSTANDARD LVCMOS15} [get_ports {SW[5]}]
set_property -dict {PACKAGE_PIN AD11 IOSTANDARD LVCMOS15} [get_ports {SW[6]}]
set_property -dict {PACKAGE_PIN AD10 IOSTANDARD LVCMOS15} [get_ports {SW[7]}]
set_property -dict {PACKAGE_PIN AE10 IOSTANDARD LVCMOS15} [get_ports {SW[8]}]
set_property -dict {PACKAGE_PIN AE12 IOSTANDARD LVCMOS15} [get_ports {SW[9]}]
set_property -dict {PACKAGE_PIN AF12 IOSTANDARD LVCMOS15} [get_ports {SW[10]}]
set_property -dict {PACKAGE_PIN AE8  IOSTANDARD LVCMOS15} [get_ports {SW[11]}]
set_property -dict {PACKAGE_PIN AF8  IOSTANDARD LVCMOS15} [get_ports {SW[12]}]
set_property -dict {PACKAGE_PIN AE13 IOSTANDARD LVCMOS15} [get_ports {SW[13]}]
set_property -dict {PACKAGE_PIN AF13 IOSTANDARD LVCMOS15} [get_ports {SW[14]}]
set_property -dict {PACKAGE_PIN AF10 IOSTANDARD LVCMOS15} [get_ports {SW[15]}]

set_property -dict {PACKAGE_PIN AF23 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports sdClk]
set_property -dict {PACKAGE_PIN AD25 IOSTANDARD LVCMOS33 SLEW FAST PULLUP true} [get_ports sdCmd]
set_property -dict {PACKAGE_PIN AE25 IOSTANDARD LVCMOS33 SLEW FAST PULLUP true} [get_ports {sdDat[0]}]
set_property -dict {PACKAGE_PIN AE22 IOSTANDARD LVCMOS33 SLEW FAST PULLUP true} [get_ports {sdDat[1]}]
set_property -dict {PACKAGE_PIN AF22 IOSTANDARD LVCMOS33 SLEW FAST PULLUP true} [get_ports {sdDat[2]}]
set_property -dict {PACKAGE_PIN Y20  IOSTANDARD LVCMOS33 SLEW FAST PULLUP true} [get_ports {sdDat[3]}]
set_property -dict {PACKAGE_PIN AE26 IOSTANDARD LVCMOS33} [get_ports sdCd]
set_property -dict {PACKAGE_PIN AE23 IOSTANDARD LVCMOS33} [get_ports sdRst]

set_property -dict {PACKAGE_PIN L25 IOSTANDARD LVCMOS33 PULLUP true} [get_ports uartRx]
set_property -dict {PACKAGE_PIN P24 IOSTANDARD LVCMOS33 DRIVE 16 SLEW FAST PULLUP true} [get_ports uartTx]

set_property -dict {PACKAGE_PIN N18 IOSTANDARD LVCMOS33 PULLUP true} [get_ports ps2Clk]
set_property -dict {PACKAGE_PIN M19 IOSTANDARD LVCMOS33 PULLUP true} [get_ports ps2Dat]

set_property -dict {PACKAGE_PIN N21 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {VGAColor[0]}]
set_property -dict {PACKAGE_PIN N22 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {VGAColor[1]}]
set_property -dict {PACKAGE_PIN R21 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {VGAColor[2]}]
set_property -dict {PACKAGE_PIN P21 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {VGAColor[3]}]
set_property -dict {PACKAGE_PIN R22 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {VGAColor[4]}]
set_property -dict {PACKAGE_PIN R23 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {VGAColor[5]}]
set_property -dict {PACKAGE_PIN T24 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {VGAColor[6]}]
set_property -dict {PACKAGE_PIN T25 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {VGAColor[7]}]
set_property -dict {PACKAGE_PIN M22 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports HSync]
set_property -dict {PACKAGE_PIN T20 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {VGAColor[8]}]
set_property -dict {PACKAGE_PIN R20 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {VGAColor[9]}]
set_property -dict {PACKAGE_PIN T22 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {VGAColor[10]}]
set_property -dict {PACKAGE_PIN T23 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports {VGAColor[11]}]
set_property -dict {PACKAGE_PIN M21 IOSTANDARD LVCMOS33 SLEW FAST} [get_ports VSync]

set_property -dict {PACKAGE_PIN E15 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_ce_n[0]}]
set_property -dict {PACKAGE_PIN G15 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_ce_n[1]}]
set_property -dict {PACKAGE_PIN K20 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_ce_n[2]}]

set_property -dict {PACKAGE_PIN D20 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[0]}]
set_property -dict {PACKAGE_PIN D18 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[1]}]
set_property -dict {PACKAGE_PIN E16 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[2]}]
set_property -dict {PACKAGE_PIN E18 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[3]}]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[4]}]
set_property -dict {PACKAGE_PIN E20 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[5]}]
set_property -dict {PACKAGE_PIN F15 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[6]}]
set_property -dict {PACKAGE_PIN F18 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[7]}]
set_property -dict {PACKAGE_PIN H19 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[8]}]
set_property -dict {PACKAGE_PIN J16 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[9]}]
set_property -dict {PACKAGE_PIN J18 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[10]}]
set_property -dict {PACKAGE_PIN J20 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[11]}]
set_property -dict {PACKAGE_PIN G19 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[12]}]
set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[13]}]
set_property -dict {PACKAGE_PIN F20 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[14]}]
set_property -dict {PACKAGE_PIN G17 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[15]}]
set_property -dict {PACKAGE_PIN F17 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[16]}]
set_property -dict {PACKAGE_PIN F19 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[17]}]
set_property -dict {PACKAGE_PIN H18 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[18]}]
set_property -dict {PACKAGE_PIN G20 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_addr[19]}]

set_property -dict {PACKAGE_PIN M16 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[0]}]
set_property -dict {PACKAGE_PIN L19 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[1]}]
set_property -dict {PACKAGE_PIN L17 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[2]}]
set_property -dict {PACKAGE_PIN K18 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[3]}]
set_property -dict {PACKAGE_PIN L18 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[4]}]
set_property -dict {PACKAGE_PIN K17 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[5]}]
set_property -dict {PACKAGE_PIN K16 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[6]}]
set_property -dict {PACKAGE_PIN M17 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[7]}]
set_property -dict {PACKAGE_PIN H26 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[8]}]
set_property -dict {PACKAGE_PIN H23 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[9]}]
set_property -dict {PACKAGE_PIN H21 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[10]}]
set_property -dict {PACKAGE_PIN J26 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[11]}]
set_property -dict {PACKAGE_PIN L20 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[12]}]
set_property -dict {PACKAGE_PIN J19 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[13]}]
set_property -dict {PACKAGE_PIN J21 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[14]}]
set_property -dict {PACKAGE_PIN K21 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[15]}]
set_property -dict {PACKAGE_PIN B26 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[16]}]
set_property -dict {PACKAGE_PIN C22 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[17]}]
set_property -dict {PACKAGE_PIN A24 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[18]}]
set_property -dict {PACKAGE_PIN A23 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[19]}]
set_property -dict {PACKAGE_PIN E22 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[20]}]
set_property -dict {PACKAGE_PIN E23 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[21]}]
set_property -dict {PACKAGE_PIN C24 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[22]}]
set_property -dict {PACKAGE_PIN D23 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[23]}]
set_property -dict {PACKAGE_PIN B20 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[24]}]
set_property -dict {PACKAGE_PIN A20 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[25]}]
set_property -dict {PACKAGE_PIN C21 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[26]}]
set_property -dict {PACKAGE_PIN B21 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[27]}]
set_property -dict {PACKAGE_PIN A22 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[28]}]
set_property -dict {PACKAGE_PIN B22 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[29]}]
set_property -dict {PACKAGE_PIN D21 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[30]}]
set_property -dict {PACKAGE_PIN E21 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[31]}]
set_property -dict {PACKAGE_PIN H24 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[32]}]
set_property -dict {PACKAGE_PIN E26 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[33]}]
set_property -dict {PACKAGE_PIN G25 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[34]}]
set_property -dict {PACKAGE_PIN F24 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[35]}]
set_property -dict {PACKAGE_PIN F25 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[36]}]
set_property -dict {PACKAGE_PIN G24 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[37]}]
set_property -dict {PACKAGE_PIN G21 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[38]}]
set_property -dict {PACKAGE_PIN G26 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[39]}]
set_property -dict {PACKAGE_PIN F22 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[40]}]
set_property -dict {PACKAGE_PIN G22 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[41]}]
set_property -dict {PACKAGE_PIN C26 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[42]}]
set_property -dict {PACKAGE_PIN D24 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[43]}]
set_property -dict {PACKAGE_PIN E25 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[44]}]
set_property -dict {PACKAGE_PIN F23 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[45]}]
set_property -dict {PACKAGE_PIN D25 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[46]}]
set_property -dict {PACKAGE_PIN D26 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_data[47]}]
set_property -dict {PACKAGE_PIN D19 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_oe_n[0]}]
set_property -dict {PACKAGE_PIN U19 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_oe_n[1]}]
set_property -dict {PACKAGE_PIN P16 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_oe_n[2]}]
set_property -dict {PACKAGE_PIN J15 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_we_n[0]}]
set_property -dict {PACKAGE_PIN T19 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_we_n[1]}]
set_property -dict {PACKAGE_PIN P23 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_we_n[2]}]

set_property -dict {PACKAGE_PIN R26 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_ub_n[0]}]
set_property -dict {PACKAGE_PIN P20 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_ub_n[1]}]
set_property -dict {PACKAGE_PIN P18 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_ub_n[2]}]
set_property -dict {PACKAGE_PIN K26 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_lb_n[0]}]
set_property -dict {PACKAGE_PIN M20 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_lb_n[1]}]
set_property -dict {PACKAGE_PIN R17 IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_lb_n[2]}]

#set_property -dict {IOSTANDARD LVCMOS33 SLEW FAST DRIVE 4} [get_ports {sram_*}]