`timescale 1ns / 1ps

module L2Cache(
    input clk,
    input rst,
    //Wishbone slave interface
    input [31:0] ws_addr, input [511:0] ws_din,
    input [63:0] ws_dm, input ws_stb, input ws_we,
    output reg ws_ack = 0, output [511:0] ws_dout,
    
    //Wishbone DDR interface
    output [31:0] ws_DDRaddr, output [511:0] ws_DDRdin,
    output [63:0] ws_DDRdm, output ws_DDRcyc, output ws_DDRstb, output ws_DDRwe,
    input ws_DDRack, input [511:0] ws_DDRdout,
    
    //Wishbone SRAM interface
    output [31:0] ws_SRAMaddr, output [511:0] ws_SRAMdin,
    output [63:0] ws_SRAMdm,   
    output ws_SRAMstb,     
    output ws_SRAMwe,
    input ws_SRAMack, 
    input [511:0] ws_SRAMdout
    );
    
    reg [31:0]wrAddr;
    reg [511:0]wrDin;
    reg [63:0]wrDm;
    reg wrWe;
    
    
    assign ws_DDRcyc = clk;
endmodule
