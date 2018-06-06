`timescale 1ns / 1ps
/**
 * Adapts the user interface of DDR3 MIG core to Wishbone slave interface.
 * 
 * @author Yunye Pu
 */
module DDR3_wsWrapper(
	//input clkIn, 
	input sysclk_p,
	input sysclk_n,
	input clkCPU,
	input rst, output clkOut,
	//Wishbone slave interface
	input [31:0] ws_addr, input [511:0] ws_din,
	input [63:0] ws_dm, 
	input ws_cyc, 	
	input ws_stb, 	
	input ws_we,
	output reg ws_ack = 0, 
	output [511:0] ws_dout,
	
	//debug signals
	output [2:0] dbg_state, //output [7:0] dbg_signal,
/*	
	//DDR3 interface
	output [13:0] ddr3_addr,
	output [2:0] ddr3_ba,
	output ddr3_cas_n,
	output [0:0] ddr3_ck_n,
	output [0:0] ddr3_ck_p,
	output [0:0] ddr3_cke,
	output ddr3_ras_n,
	output ddr3_reset_n,
	output ddr3_we_n,
	inout [31:0] ddr3_dq,
	inout [3:0] ddr3_dqs_n,
	inout [3:0] ddr3_dqs_p,
	output ddr3_cs_n,
	output [3:0] ddr3_dm,
	output [0:0] ddr3_odt,
	*/
	//sram interface
	input [31:0] sramOutData,
	output reg [31:0] sramAddr,
	output reg [31:0] sramInData, 
	output reg [3:0]sramDm,
	output reg sramStb,
	input sramNak

);
	localparam MEM_READ = 3'b001;
	localparam MEM_WRITE = 3'b000;
	
	reg [31:0] addr_reg;
	reg [31:0] rdData[15:0];
	reg [511:0] wrData;
	reg [63:0] wrDm;
	(* MARK_DEBUG = "true" *)
	reg [4:0] count;
	
	localparam STATE_READY = 3'b000;
	localparam STATE_READ = 3'b001;
	localparam STATE_READ_WAIT = 3'b010;
	localparam STATE_WRITE = 3'b011;
	localparam STATE_WRITE_WAIT = 3'b100;
	localparam STATE_READ_END = 3'b110;
	localparam STATE_WRITE_END = 3'b111;
	reg [2:0] state = STATE_READY;
	
    wire [31:0]wrData_[15:0];
    assign wrData_[0][31:0] = wrData[31:0]; assign wrData_[1][31:0] = wrData[63:32]; assign wrData_[2][31:0] = wrData[95:64]; assign wrData_[3][31:0] = wrData[127:96];
    assign wrData_[4][31:0] = wrData[159:128]; assign wrData_[5][31:0] = wrData[191:160]; assign wrData_[6][31:0] = wrData[223:192]; assign wrData_[7][31:0] = wrData[255:224];
	assign wrData_[8][31:0] = wrData[287:256]; assign wrData_[9][31:0] = wrData[319:288]; assign wrData_[10][31:0] = wrData[351:320]; assign wrData_[11][31:0] = wrData[383:352];
	assign wrData_[12][31:0] = wrData[415:384]; assign wrData_[13][31:0] = wrData[447:416]; assign wrData_[14][31:0] = wrData[479:448]; assign wrData_[15][31:0] = wrData[511:480];
	
	wire [3:0]wrDm_[15:0];
	assign wrDm_[0] = wrDm[3:0]; assign wrDm_[1] = wrDm[7:4]; assign wrDm_[2] = wrDm[11:8]; assign wrDm_[3] = wrDm[15:12];
	assign wrDm_[4] = wrDm[19:16]; assign wrDm_[5] = wrDm[23:20]; assign wrDm_[6] = wrDm[27:24]; assign wrDm_[7] = wrDm[31:28];
	assign wrDm_[8] = wrDm[35:32]; assign wrDm_[9] = wrDm[39:36]; assign wrDm_[10] = wrDm[43:40]; assign wrDm_[11] = wrDm[47:44];
	assign wrDm_[12] = wrDm[51:48]; assign wrDm_[13] = wrDm[55:52]; assign wrDm_[14] = wrDm[59:56]; assign wrDm_[15] = wrDm[63:60];
	
	always @ (posedge clkCPU) begin        
        case(state)
        STATE_READY: begin;
           ws_ack <= 1'b0;
           sramStb <= 1'b0;
           sramDm <= 4'b0;
           addr_reg <= 0;
           rdData[0] <= 0; rdData[1] <= 0; rdData[2] <= 0; rdData[3] <= 0;
           rdData[4] <= 0; rdData[5] <= 0; rdData[6] <= 0; rdData[7] <= 0;
           rdData[8] <= 0; rdData[9] <= 0; rdData[10] <= 0; rdData[11] <= 0;
           rdData[12] <= 0; rdData[13] <= 0; rdData[14] <= 0; rdData[15] <= 0;
           wrData <= 0;
           if(ws_stb)
           begin
               addr_reg <= ws_addr;
               count <= 0;
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
                   sramDm <= 4'b0000;
                   sramAddr <= addr_reg;
                   count <= count + 1;
                   addr_reg <= addr_reg+4;
               end
           end
        end
        STATE_READ_END: begin
           sramStb <= 1'b0;
           sramDm <= 4'b0;
           if(sramNak == 1'b0)
           begin
               rdData[15] <= sramOutData;
               ws_ack <= 1'b1;
               state <= STATE_READY;
           end
        end
        STATE_WRITE_END: begin
           sramStb <= 1'b0;
           sramDm <= 4'b0;
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
	                      
	    DDR3 u_DDR3 (
      
              .app_addr(0),
              .app_cmd(0),
              .app_en(0),
              .app_rdy(32'b0),
      
              .app_wdf_data(0),
              .app_wdf_mask(0),
              .app_wdf_end(0),
              .app_wdf_wren(0),
              .app_wdf_rdy(32'b0),
      
              .app_rd_data(32'b0),
              .app_rd_data_end(32'b0),
              .app_rd_data_valid(32'b0),
      
              .app_sr_req(1'b0),
              .app_ref_req(1'b0),
              .app_zq_req(1'b0),
              .app_sr_active(),
              .app_ref_ack(),
              .app_zq_ack(),
      
              .sys_clk_p(sysclk_p),
              .sys_clk_n(sysclk_n),
              .ui_clk(clkOut),
              .ui_clk_sync_rst(),
              .sys_rst(1'b0)
          );
	      
endmodule	


module DDR3_wsWrapper_sim();

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

DDR3_wsWrapper ddr3_wsWrapper(
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
	       
/*	always @ (posedge clkOut)
	case(state)
	STATE_READY: begin
		addrLow <= 1'b0;
		if(ws_cyc & ws_stb)
		begin
			addr_reg <= ws_addr[28:6];
			if(ws_we)
			begin
				state <= STATE_WRITE_DATA;
				wrData <= ws_din;
				wrDm <= ws_dm;
				ws_ack <= 1'b1;
			end
			else
				state <= STATE_READ;
		end
	end
	STATE_WRITE_DATA: begin
		ws_ack <= 1'b0;
		if(app_wdf_rdy)
		begin
			addrLow <= addrLow + 1'b1;
			if(addrLow == 1'b1)
				state <= STATE_WRITE_CMD;
		end
	end
	STATE_WRITE_CMD: begin
		if(app_rdy)
		begin
			addrLow <= addrLow + 1'b1;
			if(addrLow == 1'b1)
				state <= STATE_READY;
		end
	end
	STATE_READ: begin
		if(app_rdy)
		begin
			addrLow <= addrLow + 1'b1;
			if(addrLow == 1'b1)
				state <= STATE_READ_WAIT;
		end
	end
	STATE_READ_WAIT: begin
		if(app_rd_data_valid)
		begin
			addrLow <= addrLow + 1'b1;
			rdData <= {app_rd_data, rdData[511:256]};
			if(addrLow == 1'b1)
			begin
				state <= STATE_WS_END;
				ws_ack <= 1'b1;
			end
		end
	end
	STATE_WS_END: begin
		ws_ack <= 1'b0;
		state <= STATE_READY;
	end
	default: state <= STATE_READY;
	endcase
	
	assign app_en = (state == STATE_WRITE_CMD) | (state == STATE_READ);
	assign app_wdf_wren = (state == STATE_WRITE_DATA);
	assign app_wdf_end = app_wdf_wren;
	assign app_cmd = (state == STATE_WRITE_CMD)? MEM_WRITE: MEM_READ;
//	assign app_addr = {addr_reg, addrLow, 4'h0};
	assign app_addr = {1'b0, addr_reg, addrLow, 3'h0};//This might be an issue in the MIG core
	
	assign ws_dout = rdData;
	
	assign app_wdf_data = addrLow? wrData[511:256]: wrData[255:0];
	assign app_wdf_mask = addrLow? wrDm[63:32]: wrDm[31:0];

	DDR3 u_DDR3 (
		.ddr3_addr(ddr3_addr),
		.ddr3_ba(ddr3_ba),
		.ddr3_cas_n(ddr3_cas_n),
		.ddr3_ck_n(ddr3_ck_n),
		.ddr3_ck_p(ddr3_ck_p),
		.ddr3_cke(ddr3_cke),
		.ddr3_ras_n(ddr3_ras_n),
		.ddr3_reset_n(ddr3_reset_n),
		.ddr3_we_n(ddr3_we_n),
		.ddr3_dq(ddr3_dq),
		.ddr3_dqs_n(ddr3_dqs_n),
		.ddr3_dqs_p(ddr3_dqs_p),
		.ddr3_cs_n(ddr3_cs_n),
		.ddr3_dm(ddr3_dm),
		.ddr3_odt(ddr3_odt),
		.init_calib_complete(init_calib_complete),

		.app_addr(app_addr),
		.app_cmd(app_cmd),
		.app_en(app_en),
		.app_rdy(app_rdy),

		.app_wdf_data(app_wdf_data),
		.app_wdf_mask(~app_wdf_mask),
		.app_wdf_end(app_wdf_end),
		.app_wdf_wren(app_wdf_wren),
		.app_wdf_rdy(app_wdf_rdy),

		.app_rd_data(app_rd_data),
		.app_rd_data_end(app_rd_data_end),
		.app_rd_data_valid(app_rd_data_valid),

		.app_sr_req(1'b0),
		.app_ref_req(1'b0),
		.app_zq_req(1'b0),
		.app_sr_active(),
		.app_ref_ack(),
		.app_zq_ack(),

		.sys_clk_p(sysclk_p),
		.sys_clk_n(sysclk_n),
		.ui_clk(clkOut),
		.ui_clk_sync_rst(),
		.sys_rst(1'b0)
	);*/

