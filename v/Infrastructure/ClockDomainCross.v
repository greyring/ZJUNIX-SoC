`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/24/2016 05:30:32 PM
// Design Name: 
// Module Name: ClockDomainCross
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
//This module ensures a positive pulse on I will be captured on O, when clki has a higher frequency than clko.
//module ClockDomainCross_extend(
//	input clki, input clko, input i, output o
//);
//	reg i_reg;
//	reg o_feedback;
//	reg o_reg;

//	always @ (posedge clki)
//	begin
//		o_feedback <= o;
//		if(i | o_feedback)
//			i_reg <= i;
//	end
//	always @ (posedge clko) o_reg <= i | i_reg;
//	assign o = o_reg;
	
//endmodule

//module PulseExtender #(
//	parameter EXT_LENGTH = 1,
//	parameter EXT_LENGTH_BITS = 1,
//	parameter O_REG = 0
//)(
//	input clk, input I, output reg O
//);
//	reg [EXT_LENGTH_BITS-1:0] counter = 0;
	
//	always @ (posedge clk)
//	if(I)
//		counter <= EXT_LENGTH;
//	else if(|counter)
//		counter <= counter - 1'b1;
	
//	generate
//	if(O_REG)
//	begin: O_REGISTERED
//		always @ (posedge clk) O <= I | (|counter);
//	end
//	else
//	begin: O_UNREGISTERED
//		always @* O <= I | (|counter);
//	end
//	endgenerate

//endmodule

//module PulseLimiter #(
//	parameter LIMIT_LENGTH = 1,
//	parameter LIMIT_LENGTH_BITS = 1,
//	parameter O_REG = 0
//)(
//	input clk, input I, output reg O
//);
//	reg [LIMIT_LENGTH_BITS-1:0] counter;
	
//	always @ (posedge clk)
//	if(~I)
//		counter <= LIMIT_LENGTH;
//	else if(|counter)
//		counter <= counter - 1'b1;

//	generate
//	if(O_REG)
//	begin: O_REGISTERED
//		always @ (posedge clk) O <= I & |counter;
//	end
//	else
//	begin: O_UNREGISTERED
//		always @* O <= I & |counter;
//	end
//	endgenerate
	
//endmodule

module ClockDomainCross #(
	parameter I_REG = 1,
	parameter O_REG = 1
)(
	input clki, input clko, input i, output o
);
	wire internal;
	PipeReg #(I_REG) inReg(.clk(clki), .i(i), .o(internal));
	PipeReg #(O_REG) outReg(.clk(clko), .i(internal), .o(o));

endmodule

module PipeReg #(
	parameter DEPTH = 1
)(
	input clk, input i, output o
);
	generate
	if(DEPTH < 1)
		assign o = i;
	else if(DEPTH == 1)
	begin
		(* SHREG_EXTRACT = "NO" *)
		reg o_reg;
		always @ (posedge clk) o_reg <= i;
		assign o = o_reg;
	end
	else
	begin
		(* SHREG_EXTRACT = "NO" *)
		reg [DEPTH-1:0] o_reg;
		always @ (posedge clk) o_reg <= {i, o_reg[DEPTH-1:1]};
		assign o = o_reg[0];
	end
	endgenerate
	
endmodule

module Handshake_freqUp( //clkAck has a higher frequency than clkStb
	input clkStb, input clkAck,
	input stbI, output reg stbO,
	input ackI, output reg ackO
);
	always @ (posedge clkAck)
	if(ackI)
		stbO <= 1'b0;
	else if(stbI)
		stbO <= 1'b1;
	
	reg ack;
	always @ (posedge clkStb or posedge ackI)
	if(ackI)
		ack <= 1'b1;
	else
		ack <= 1'b0;
	always @ (posedge clkStb)
		ackO <= ack;
	
endmodule

module Handshake_freqDown( // clkAck has a lower or equal frequency than clkStb
	input clkStb, input clkAck,
	input stbI, output reg stbO,
	input ackI, output reg ackO
);
	always @ (posedge clkAck or posedge stbI)
	if(stbI)
		stbO <= 1'b1;
	else if(ackI)
		stbO <= 1'b0;
	
	always @ (posedge clkStb)
		ackO <= ackI;
	
endmodule

module AsyncHandshake #(
	parameter STB_FREQ = 100,
	parameter ACK_FREQ = 100
)(
	input clkStb, input clkAck,
	input stbI, output stbO,
	input ackI, output ackO
);

	generate if(STB_FREQ < ACK_FREQ)
	begin: FREQ_UP
		Handshake_freqUp(clkStb, clkAck, stbI, stbO, ackI, ackO);
	end else begin: FREQ_DOWN
		Handshake_freqDown(clkStb, clkAck, stbI, stbO, ackI, ackO);
	end endgenerate

endmodule
