`timescale 1ns / 1ps

module SRAM_wsWrapper(
	input clkCPU,
	input rst,
	//Wishbone slave interface
	input [31:0] ws_addr, input [767:0] ws_din,//16*48
	input [95:0] ws_dm, //16*6
	input ws_stb,
	input ws_we, 
	output reg ws_ack = 0,
	output [767:0] ws_dout,

	//sram interface
	input [47:0] sramOutData,
	output reg [31:0] sramAddr,
	output reg [47:0] sramInData, 
	output reg [5:0]sramDm,
	output reg sramStb,
	input sramNak
);	
	reg [31:0] addr_reg;
	reg [47:0] rdData[15:0];
	reg [767:0] wrData;
	reg [95:0] wrDm;
	reg [4:0] count;
	
	localparam STATE_READY = 3'b000;
	localparam STATE_READ = 3'b001;
	localparam STATE_READ_WAIT = 3'b010;
	localparam STATE_WRITE = 3'b011;
	localparam STATE_WRITE_WAIT = 3'b100;
	localparam STATE_READ_END = 3'b110;
	localparam STATE_WRITE_END = 3'b111;
	reg [2:0] state = STATE_READY;
	
    wire [47:0]wrData_[15:0];
    wire [3:0]wrDm_[15:0];
    generate
    genvar i;
    for (i = 0; i<16; i=i+1) begin
        assign wrData_[i][47:0] = wrData[48*i+47:48*i];
        assign wrDm_[i] = wrDm[6*i+3:6*i];
    end
    endgenerate

	always @ (posedge clkCPU) begin    
        case(state)
        STATE_READY: begin;
           ws_ack <= 1'b0;
           sramStb <= 1'b0;
           sramDm <= 6'b0;
           addr_reg <= 32'b0;
           rdData[0] <= 48'b0; rdData[1] <= 48'b0; rdData[2] <= 48'b0; rdData[3] <= 48'b0;
           rdData[4] <= 48'b0; rdData[5] <= 48'b0; rdData[6] <= 48'b0; rdData[7] <= 48'b0;
           rdData[8] <= 48'b0; rdData[9] <= 48'b0; rdData[10] <= 48'b0; rdData[11] <= 48'b0;
           rdData[12] <= 48'b0; rdData[13] <= 48'b0; rdData[14] <= 48'b0; rdData[15] <= 48'b0;
           wrData <= 48'b0;
           if(ws_stb)
           begin
               addr_reg <= ws_addr;
               count <= 5'b0;
               if(ws_we)
               begin
                   state <= STATE_WRITE;
                   wrData <= ws_din;
                   wrDm <= ws_dm;                  
               end
               else
                   state <= STATE_READ;
           end
        end
        STATE_WRITE: begin
           if (sramNak == 1'b0)
           begin
               if (count == 16)
                   state <= STATE_WRITE_END;
               else begin
                   sramStb <= 1'b1;
                   sramAddr <= addr_reg;
                   sramDm <= wrDm_[count];
                   sramInData <= wrData_[count];
                   count <= count + 1;
                   addr_reg <= addr_reg + 4;
               end
           end
        end
        STATE_READ: begin
           if(sramNak == 1'b0)
           begin
               if (count > 1)
                   rdData[count-2] <= sramOutData;
               if (count == 16)
                   state <= STATE_READ_END;
               else begin
                   sramStb <= 1'b1;
                   sramDm <= 6'b0;
                   sramAddr <= addr_reg;
                   count <= count + 1;
                   addr_reg <= addr_reg+4;
               end
           end
        end
        STATE_READ_END: begin
           sramStb <= 1'b0;
           sramDm <= 6'b0;
           if(sramNak == 1'b0)
           begin
               rdData[15] <= sramOutData;
               ws_ack <= 1'b1;
               state <= STATE_READY;
           end
        end
        STATE_WRITE_END: begin
           sramStb <= 1'b0;
           sramDm <= 6'b0;
           if(sramNak == 1'b0)
           begin
               ws_ack <= 1'b1;
               state <= STATE_READY;
           end
        end
        default: state <= STATE_READY;
	    endcase
	end

    assign ws_dout = {rdData[15], rdData[14], rdData[13], rdData[12], 
                      rdData[11], rdData[10], rdData[9], rdData[8], 
                      rdData[7], rdData[6], rdData[5], rdData[4],
                      rdData[3], rdData[2], rdData[1], rdData[0]};
endmodule	


module SRAM_wsWrapper_sim();

reg clk=1'b1;
reg rst=1'b1;
reg [31:0]ws_addr = 32'b0;
reg [511:0]ws_din = 512'b0;
reg [63:0]ws_dm = 64'b0;
reg ws_stb = 1'b0;
reg ws_we = 1'b0;
wire ws_ack;
wire [511:0]ws_dout;

wire [47:0]sramOutData;
wire [31:0]sramAddr;
wire [31:0]sramInData;
wire [3:0]sramDm;
wire sramStb, sramNak;

SRAM_wsWrapper sram_wsWrapper(
    .clkCPU(clk), .rst(rst),
	//Wishbone slave interface
	.ws_addr(ws_addr), .ws_din(ws_din),
	.ws_dm(ws_dm), .ws_cyc(clk), .ws_stb(ws_stb), .ws_we(ws_we),
	.ws_ack(ws_ack), .ws_dout(ws_dout),
	
	.sramOutData(sramOutData),
	.sramAddr(sramAddr),
	.sramInData(sramInData), 
	.sramDm(sramDm),
	.sramStb(sramStb),
	.sramNak(sramNak)
);

wire [2:0]sram_ce_n, sram_oe_n, sram_we_n, sram_ub_n, sram_lb_n;
wire [19:0]sram_addr;
reg sram_inout = 0;
reg [47:0]sram_data_;
wire [47:0]sram_data;
reg clk2 = 1'b1;
assign sram_data = sram_inout ? sram_data_ : 48'hZZZZZZZZZZZZ;

SRAM sram(
    .clk(clk2),  // main clock
    .rst(rst),  // synchronous reset

    // SRAM interfaces
    .sram_ce_n(sram_ce_n),
    .sram_oe_n(sram_oe_n),
    .sram_we_n(sram_we_n),
    .sram_ub_n(sram_ub_n),  
    .sram_lb_n(sram_lb_n),
    .sram_addr(sram_addr),
    .sram_data(sram_data),
    
    // WishBone Bus
    .wb_stb(sramStb),  // chip select
    .wb_addr(sramAddr),  // address
    .wb_we(sramDm),
    .wb_din(sramInData),
    .wb_dout(sramOutData),
    .wb_nak(sramNak)
   );

initial forever #5 clk <= !clk;
initial forever #5 clk2 <= !clk2;

initial begin
    #10
    rst = 1'b0;
    ws_addr = 0;
    //ws_din = 512'h12345678_87654321_5A5A5A5A_A5A5A5A5_12345678_87654321_5A5A5A5A_A5A5A5A5_12345678_87654321_5A5A5A5A_A5A5A5A5_12345678_87654321_5A5A5A5A_A5A5A5A5;
    //ws_dm = 64'hffffffff_ffffffff;
    ws_dm = 64'h00000000_00000000;
    ws_stb = 1'b1;
    ws_we = 1'b0;
    #10
    ws_stb = 1'b0;
    
    #21
    sram_inout = 1'b1;
    sram_data_ = 48'h000012345678;
    #20
    sram_inout = 1'b0;
    #10
    sram_inout = 1'b1;
    sram_data_ = 48'h000012345678;
    #20
    sram_inout = 1'b0;
    #10
    sram_inout = 1'b1;
    sram_data_ = 48'h000012345678;
    #20
    sram_inout = 1'b0;
    #10
    sram_inout = 1'b1;
    sram_data_ = 48'h000012345678;
    #20
    sram_inout = 1'b0;
    #10
    sram_inout = 1'b1;
    sram_data_ = 48'h000012345678;
    #20
    sram_inout = 1'b0;
    #10
    sram_inout = 1'b1;
    sram_data_ = 48'h000012345678;
    #20
    sram_inout = 1'b0;
    #10
    sram_inout = 1'b1;
    sram_data_ = 48'h000012345678;
    #20
    sram_inout = 1'b0;
    #10
    sram_inout = 1'b1;
    sram_data_ = 48'h000012345678;
    #20
    sram_inout = 1'b0;
    
end

endmodule