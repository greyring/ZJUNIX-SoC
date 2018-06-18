`timescale 1ns / 1ps

module L2Cache(
    input clk,
    input rst,
    //Wishbone slave interface
    (* MARK_DEBUG = "true" *)
    input [31:0] ws_addr, 
    (* MARK_DEBUG = "true" *)
    input [511:0] ws_din,
    input [63:0] ws_dm, 
    (* MARK_DEBUG = "true" *)
    input ws_stb, 
    (* MARK_DEBUG = "true" *)
    input ws_we,
    (* MARK_DEBUG = "true" *)
    output reg ws_ack = 0, 
    output reg[511:0] ws_dout,
    
    //Wishbone DDR interface
    (* MARK_DEBUG = "true" *)
    output reg [31:0] ws_DDRaddr, 
    output reg  [511:0] ws_DDRdin,
    output reg [63:0] ws_DDRdm, 
    output ws_DDRcyc, 
    (* MARK_DEBUG = "true" *)
    output reg ws_DDRstb, 
    (* MARK_DEBUG = "true" *)
    output reg ws_DDRwe,
    (* MARK_DEBUG = "true" *)
    input ws_DDRack, 
    input [511:0] ws_DDRdout,
    
    //Wishbone SRAM interface
    (* MARK_DEBUG = "true" *)
    output reg [31:0] ws_SRAMaddr,
    (* MARK_DEBUG = "true" *)
    output reg [767:0] ws_SRAMdin,
    (* MARK_DEBUG = "true" *)
    output reg [95:0] ws_SRAMdm,
    (* MARK_DEBUG = "true" *)
    output reg ws_SRAMstb,   
    (* MARK_DEBUG = "true" *)  
    output reg ws_SRAMwe,
    (* MARK_DEBUG = "true" *)
    input ws_SRAMack, 
    (* MARK_DEBUG = "true" *)
    input [767:0] ws_SRAMdout
    
    );
    
    reg [31:0]wrAddr;
    reg [511:0]wrDin;
    reg [63:0]wrDm;
    reg wrWe;
    
    reg [31:0]addr;
    
    reg [11:0]sramLabel;//11:2 tag (31:22) 1 dirty 0 valid
    reg [31:0]sramData[15:0];
    wire [511:0]sramData_r;
    
    wire [95:0]sramDm_w;
    wire [767:0]sramData_w;
    
    wire [511:0]ddrData_w;
    wire [767:0]sramData_rw;
    
    always @(posedge clk) begin
        sramData[0]=ws_SRAMack?ws_SRAMdout[48*0 +: 32] : sramData[0];
        sramData[1]=ws_SRAMack?ws_SRAMdout[48*1 +: 32] : sramData[1];
        sramData[2]=ws_SRAMack?ws_SRAMdout[48*2 +: 32] : sramData[2];
        sramData[3]=ws_SRAMack?ws_SRAMdout[48*3 +: 32] : sramData[3];
        sramData[4]=ws_SRAMack?ws_SRAMdout[48*4 +: 32] : sramData[4];
        sramData[5]=ws_SRAMack?ws_SRAMdout[48*5 +: 32] : sramData[5];
        sramData[6]=ws_SRAMack?ws_SRAMdout[48*6 +: 32] : sramData[6];
        sramData[7]=ws_SRAMack?ws_SRAMdout[48*7 +: 32] : sramData[7];
        sramData[8]=ws_SRAMack?ws_SRAMdout[48*8 +: 32] : sramData[8];
        sramData[9]=ws_SRAMack?ws_SRAMdout[48*9 +: 32] : sramData[9];
        sramData[10]=ws_SRAMack?ws_SRAMdout[48*10 +: 32] : sramData[10];
        sramData[11]=ws_SRAMack?ws_SRAMdout[48*11 +: 32] : sramData[11];
        sramData[12]=ws_SRAMack?ws_SRAMdout[48*12 +: 32] : sramData[12];
        sramData[13]=ws_SRAMack?ws_SRAMdout[48*13 +: 32] : sramData[13];
        sramData[14]=ws_SRAMack?ws_SRAMdout[48*14 +: 32] : sramData[14];
        sramData[15]=ws_SRAMack?ws_SRAMdout[48*15 +: 32] : sramData[15];
    end
    
    assign sramData_r[32*0 +: 32] = sramData[0];
    assign sramData_r[32*1 +: 32] = sramData[1];
    assign sramData_r[32*2 +: 32] = sramData[2];
    assign sramData_r[32*3 +: 32] = sramData[3];
    assign sramData_r[32*4 +: 32] = sramData[4];
    assign sramData_r[32*5 +: 32] = sramData[5];
    assign sramData_r[32*6 +: 32] = sramData[6];
    assign sramData_r[32*7 +: 32] = sramData[7];
    assign sramData_r[32*8 +: 32] = sramData[8];
    assign sramData_r[32*9 +: 32] = sramData[9];
    assign sramData_r[32*10 +: 32] = sramData[10];
    assign sramData_r[32*11 +: 32] = sramData[11];
    assign sramData_r[32*12 +: 32] = sramData[12];
    assign sramData_r[32*13 +: 32] = sramData[13];
    assign sramData_r[32*14 +: 32] = sramData[14];
    assign sramData_r[32*15 +: 32] = sramData[15];
 
    assign sramDm_w[5:0] = {2'b11, wrDm[3:0]};
    assign sramDm_w[6*1 +: 6] = {2'b0, wrDm[4*1 +: 4]};
    assign sramDm_w[6*2 +: 6] = {2'b0, wrDm[4*2 +: 4]};
    assign sramDm_w[6*3 +: 6] = {2'b0, wrDm[4*3 +: 4]};
    assign sramDm_w[6*4 +: 6] = {2'b0, wrDm[4*4 +: 4]};
    assign sramDm_w[6*5 +: 6] = {2'b0, wrDm[4*5 +: 4]};
    assign sramDm_w[6*6 +: 6] = {2'b0, wrDm[4*6 +: 4]};
    assign sramDm_w[6*7 +: 6] = {2'b0, wrDm[4*7 +: 4]};
    assign sramDm_w[6*8 +: 6] = {2'b0, wrDm[4*8 +: 4]};
    assign sramDm_w[6*9 +: 6] = {2'b0, wrDm[4*9 +: 4]};
    assign sramDm_w[6*10 +: 6] = {2'b0, wrDm[4*10 +: 4]};
    assign sramDm_w[6*11 +: 6] = {2'b0, wrDm[4*11 +: 4]};
    assign sramDm_w[6*12 +: 6] = {2'b0, wrDm[4*12 +: 4]};
    assign sramDm_w[6*13 +: 6] = {2'b0, wrDm[4*13 +: 4]};
    assign sramDm_w[6*14 +: 6] = {2'b0, wrDm[4*14 +: 4]};
    assign sramDm_w[6*15 +: 6] = {2'b0, wrDm[4*15 +: 4]};

    assign sramData_w[47:0] = {4'b0, wrAddr[31:22], 2'b11, wrDin[31:0]};
    assign sramData_w[48*1 +: 48] = {16'b0, wrDin[32*1 +: 32]};
    assign sramData_w[48*2 +: 48] = {16'b0, wrDin[32*2 +: 32]};
    assign sramData_w[48*3 +: 48] = {16'b0, wrDin[32*3 +: 32]};
    assign sramData_w[48*4 +: 48] = {16'b0, wrDin[32*4 +: 32]};
    assign sramData_w[48*5 +: 48] = {16'b0, wrDin[32*5 +: 32]};
    assign sramData_w[48*6 +: 48] = {16'b0, wrDin[32*6 +: 32]};
    assign sramData_w[48*7 +: 48] = {16'b0, wrDin[32*7 +: 32]};
    assign sramData_w[48*8 +: 48] = {16'b0, wrDin[32*8 +: 32]};
    assign sramData_w[48*9 +: 48] = {16'b0, wrDin[32*9 +: 32]};
    assign sramData_w[48*10 +: 48] = {16'b0, wrDin[32*10 +: 32]};
    assign sramData_w[48*11 +: 48] = {16'b0, wrDin[32*11 +: 32]};
    assign sramData_w[48*12 +: 48] = {16'b0, wrDin[32*12 +: 32]};
    assign sramData_w[48*13 +: 48] = {16'b0, wrDin[32*13 +: 32]};
    assign sramData_w[48*14 +: 48] = {16'b0, wrDin[32*14 +: 32]};
    assign sramData_w[48*15 +: 48] = {16'b0, wrDin[32*15 +: 32]};

    assign sramData_rw[47:0] = {4'b0, wrAddr[31:22], 2'b01, ws_DDRdout[31:0]};
    assign sramData_rw[48*1 +: 48] = {16'b0, ws_DDRdout[32*1 +: 32]};
    assign sramData_rw[48*2 +: 48] = {16'b0, ws_DDRdout[32*2 +: 32]};
    assign sramData_rw[48*3 +: 48] = {16'b0, ws_DDRdout[32*3 +: 32]};
    assign sramData_rw[48*4 +: 48] = {16'b0, ws_DDRdout[32*4 +: 32]};
    assign sramData_rw[48*5 +: 48] = {16'b0, ws_DDRdout[32*5 +: 32]};
    assign sramData_rw[48*6 +: 48] = {16'b0, ws_DDRdout[32*6 +: 32]};
    assign sramData_rw[48*7 +: 48] = {16'b0, ws_DDRdout[32*7 +: 32]};
    assign sramData_rw[48*8 +: 48] = {16'b0, ws_DDRdout[32*8 +: 32]};
    assign sramData_rw[48*9 +: 48] = {16'b0, ws_DDRdout[32*9 +: 32]};
    assign sramData_rw[48*10 +: 48] = {16'b0, ws_DDRdout[32*10 +: 32]};
    assign sramData_rw[48*11 +: 48] = {16'b0, ws_DDRdout[32*11 +: 32]};
    assign sramData_rw[48*12 +: 48] = {16'b0, ws_DDRdout[32*12 +: 32]};
    assign sramData_rw[48*13 +: 48] = {16'b0, ws_DDRdout[32*13 +: 32]};
    assign sramData_rw[48*14 +: 48] = {16'b0, ws_DDRdout[32*14 +: 32]};
    assign sramData_rw[48*15 +: 48] = {16'b0, ws_DDRdout[32*15 +: 32]};    
    
    assign ddrData_w[32*0 +: 32] = sramData[0];
    assign ddrData_w[32*1 +: 32] = sramData[1];
    assign ddrData_w[32*2 +: 32] = sramData[2];
    assign ddrData_w[32*3 +: 32] = sramData[3];
    assign ddrData_w[32*4 +: 32] = sramData[4];
    assign ddrData_w[32*5 +: 32] = sramData[5];
    assign ddrData_w[32*6 +: 32] = sramData[6];
    assign ddrData_w[32*7 +: 32] = sramData[7];
    assign ddrData_w[32*8 +: 32] = sramData[8];
    assign ddrData_w[32*9 +: 32] = sramData[9];
    assign ddrData_w[32*10 +: 32] = sramData[10];
    assign ddrData_w[32*11 +: 32] = sramData[11];
    assign ddrData_w[32*12 +: 32] = sramData[12];
    assign ddrData_w[32*13 +: 32] = sramData[13];
    assign ddrData_w[32*14 +: 32] = sramData[14];
    assign ddrData_w[32*15 +: 32] = sramData[15];
    
    always @(posedge clk)
        sramLabel = ws_SRAMack ? ws_SRAMdout[43:32] : sramLabel;
        
    (* MARK_DEBUG = "true" *)
    reg [3:0]state;
    localparam STATE_INIT = 0;
    localparam STATE_CLEAR = 1;
    localparam STATE_CLEAR_LOOP = 2;
    localparam STATE_CLEAR_END = 3;
    localparam STATE_IDLE = 4;
    localparam STATE_START_WAIT = 5;
    localparam STATE_START = 6;
    localparam STATE_WRITE_BACK = 7;
    localparam STATE_READ_READ = 8;
    localparam STATE_READ_WRITE = 9;
    localparam STATE_END = 10;
    
    always @(posedge clk) begin
        if (rst) begin
            ws_SRAMstb <= 1'b0;
            ws_DDRstb <= 1'b0;
            state <= STATE_INIT;
        end
        else
        case(state)
        STATE_INIT: begin
            ws_ack <= 1'b0;
            ws_SRAMstb <= 1'b1;
            ws_SRAMwe <= 1'b1;
            ws_SRAMdm <= 96'hffffffffffffffffffffffff;
            ws_SRAMaddr <= 32'b0;
            ws_SRAMdin <= 767'b0;
            state <= STATE_CLEAR;
        end
        STATE_CLEAR: begin
           ws_SRAMaddr <= ws_SRAMaddr + 64;
           state <= STATE_CLEAR_LOOP;
        end
        STATE_CLEAR_LOOP: begin
            if (ws_SRAMack) begin
                if (ws_SRAMaddr == 32'h00FFFFC0) begin
                //if (ws_SRAMaddr == 32'h00000040) begin
                    state <= STATE_CLEAR_END;
                    ws_SRAMstb <= 1'b0;
                    ws_SRAMwe <= 1'b0;
                    ws_SRAMdm <= 96'b0;
                end
                else
                    ws_SRAMaddr <= ws_SRAMaddr + 64;
            end
        end
        STATE_CLEAR_END: begin
            if (ws_SRAMack)
                state <= STATE_IDLE;
        end
        STATE_IDLE: begin
            ws_ack <= 1'b0;
            if (ws_stb) begin//fetch data in sram
                wrWe <= ws_we;
                wrDm <= ws_dm;
                wrDin <= ws_din;
                wrAddr <= ws_addr;
                ws_SRAMstb <= 1'b1;
                ws_SRAMwe <= 1'b0;
                ws_SRAMdm <= 96'b0;
                ws_SRAMaddr <= ws_addr;
                state <= STATE_START_WAIT;
            end
        end
        STATE_START_WAIT: begin
            ws_SRAMstb <= 1'b0;
            ws_SRAMwe <= 1'b0;
            ws_SRAMdm <= 96'b0;
            if (ws_SRAMack)
                state <= STATE_START;
        end
        STATE_START: begin
            if (sramLabel[11:2] != wrAddr[31:22] && sramLabel[0] && sramLabel[1]) begin//write back to ddr
                ws_DDRstb <= 1'b1;
                ws_DDRwe <= 1'b1;
                ws_DDRaddr <= {sramLabel[11:2], wrAddr[21:0]};
                ws_DDRdin <= ddrData_w;
                ws_DDRdm <= 64'hffffffffffffffff;
                state <= STATE_WRITE_BACK;
            end
            else begin
                if (wrWe) begin//write into SRAM
                    ws_SRAMstb <= 1'b1;
                    ws_SRAMwe <= 1'b1;
                    ws_SRAMdm <= sramDm_w;
                    ws_SRAMaddr <= wrAddr;
                    ws_SRAMdin <= sramData_w;
                    state <= STATE_END;
                end
                else begin
                    if (sramLabel[11:2] == wrAddr[31:22] && sramLabel[0]) begin//return data
                        ws_dout <= sramData_r;
                        ws_ack <= 1'b1;
                        state <= STATE_IDLE;
                    end
                    else begin//read from DDR
                        ws_DDRstb <= 1'b1;
                        ws_DDRwe <= 1'b0;
                        ws_DDRdm <= 64'b0;
                        ws_DDRaddr <= wrAddr;
                        state <= STATE_READ_READ;
                    end
                end
            end
        end
        STATE_WRITE_BACK: begin
            ws_DDRstb <= 1'b0;
            ws_DDRwe <= 1'b0;
            ws_DDRdm <= 64'b0;
            if (ws_DDRack) begin
                if (wrWe) begin//write to SRAM
                    ws_SRAMstb <= 1'b1;
                    ws_SRAMwe <= 1'b1;
                    ws_SRAMdm <= sramDm_w;
                    ws_SRAMaddr <= wrAddr;
                    ws_SRAMdin <= sramData_w;
                    state <= STATE_END;
                end
                else begin//read from DDR
                    ws_DDRstb <= 1'b1;
                    ws_DDRwe <= 1'b0;
                    ws_DDRdm <= 64'b0;
                    ws_DDRaddr <= wrAddr;
                    state <= STATE_READ_READ;
                end
            end
        end
        STATE_READ_READ:begin
            ws_DDRstb <= 1'b0;
            ws_DDRwe <= 1'b0;
            ws_DDRdm <= 64'b0;
            if (ws_DDRack) begin//write to SRAM
                ws_SRAMstb <= 1'b1;
                ws_SRAMwe <= 1'b1;
                ws_SRAMdm <= 96'hffffffffffffffffffffffff;
                ws_SRAMaddr <= wrAddr;
                ws_SRAMdin <= sramData_rw;
                state <= STATE_END;
            end
        end
        STATE_END: begin
            ws_SRAMstb <= 1'b0;
            ws_SRAMwe <= 1'b0;
            ws_SRAMdm <= 96'b0;
            if (ws_SRAMack) begin
                if (!wrWe)
                    ws_dout <= sramData_r;
                ws_ack <= 1'b1;
                state <= STATE_IDLE;
            end
        end
        endcase
    end
    
    assign ws_DDRcyc = clk;
   
endmodule

module L2Cache_sim();

    reg clk = 1'b1;
    reg rst = 1'b1;
    reg [31:0]ws_addr;
    reg [511:0]ws_din;
    reg [63:0]ws_dm;
    reg ws_stb, ws_we;
    wire ws_ack;
    wire [511:0]ws_dout;
    
    wire [31:0]ws_DDRaddr;
    wire [511:0]ws_DDRdin;
    wire [63:0]ws_DDRdm;
    wire ws_DDRcyc, ws_DDRstb;
    wire ws_DDRwe;
    reg ws_DDRack;
    reg [511:0]ws_DDRdout;
    
    wire [31:0]ws_SRAMaddr;
    wire [767:0]ws_SRAMdin;
    wire [95:0]ws_SRAMdm;
    wire ws_SRAMstb;
    wire ws_SRAMwe;
    wire ws_SRAMack;
    wire [767:0]ws_SRAMdout;
  L2Cache cache(
    .clk(clk),
    .rst(rst),
    //Wishbone slave interface
    .ws_addr(ws_addr), .ws_din(ws_din),
    .ws_dm(ws_dm), .ws_stb(ws_stb), .ws_we(ws_we),
    .ws_ack(ws_ack), .ws_dout(ws_dout),
    
    //Wishbone DDR interface
    .ws_DDRaddr(ws_DDRaddr), .ws_DDRdin(ws_DDRdin),
    .ws_DDRdm(ws_DDRdm), .ws_DDRcyc(ws_DDRcyc), .ws_DDRstb(ws_DDRstb), .ws_DDRwe(ws_DDRwe),
    .ws_DDRack(ws_DDRack), .ws_DDRdout(ws_DDRdout),
    
    //Wishbone SRAM interface
    .ws_SRAMaddr(ws_SRAMaddr), .ws_SRAMdin(ws_SRAMdin),
    .ws_SRAMdm(ws_SRAMdm),   
    .ws_SRAMstb(ws_SRAMstb),     
    .ws_SRAMwe(ws_SRAMwe),
    .ws_SRAMack(ws_SRAMack), 
    .ws_SRAMdout(ws_SRAMdout)
    );
    
    wire [47:0]sramOutData;
    wire [31:0]sramAddr;
    wire [47:0]sramInData;
    wire [5:0]sramDm;
    wire sramStb;
    wire sramNak;
    SRAM_wsWrapper sram_wsWrapper(
        .clkCPU(clk),
        .rst(rst),
        //Wishbone slave interface
        .ws_addr(ws_SRAMaddr), .ws_din(ws_SRAMdin),//16*48
        .ws_dm(ws_SRAMdm), //16*6
        .ws_stb(ws_SRAMstb),
        .ws_we(ws_SRAMwe), 
        .ws_ack(ws_SRAMack),
        .ws_dout(ws_SRAMdout),
    
        //sram interface
        .sramOutData(sramOutData),
        .sramAddr(sramAddr),
        .sramInData(sramInData), 
        .sramDm(sramDm),
        .sramStb(sramStb),
        .sramNak(sramNak)
    );    
    
    wire [2:0]sram_ce_n, sram_oe_n, sram_we_n, sram_ub_n, sram_lb_n;
    wire [19:0]sram_addr;
    wire [47:0]sram_data;
    reg sram_control = 0;
    reg [47:0]sram_in;
    assign sram_data = sram_control ? sram_in : 48'hz;
    SRAM sram(
        .clk(clk),  // main clock
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
        .wb_addr(sramAddr),  // address every 4 == 48bit
        .wb_we(sramDm),
        .wb_din(sramInData),
        .wb_dout(sramOutData),
        .wb_nak(sramNak)
        );
initial forever #5 clk <= !clk;

initial begin
    #10
    rst = 0;
    #1040
    ws_stb = 1'b1;
    ws_we = 1'b0;
    ws_addr = 32'h003FFFC0;
    sram_control = 1'b1;
    sram_in = 48'b0;
    #10
    ws_stb = 1'b0;
    #631
    sram_control = 1'b0;
    ws_DDRack = 1'b1;
    ws_DDRdout = 512'h12345678123456781234567812345678123456781234567812345678123456781234567812345678123456781234567812345678123456781234567812345678;
    #10
    ws_DDRack = 1'b0;
    #809
    ws_stb = 1'b1;//read again
    #20
    ws_stb = 1'b0;
    sram_control = 1'b1;
    sram_in = 48'h000112345678;
    #700
    sram_control = 1'b0;
    ws_stb = 1'b1;
    ws_we = 1'b1;
    ws_dm = 64'hffffffffffffffff;
    ws_addr = 32'h003FFFC0;
    ws_din = 512'h87654321876543218765432187654321876543218765432187654321876543218765432187654321876543218765432187654321876543218765432187654321;
    #20
    ws_stb = 1'b0;
    sram_control = 1'b1;
    sram_in = 48'h000112345678;
    #1500
    ws_stb = 1'b1;
    ws_we = 1'b0;
    ws_addr = 32'h007FFFC0;
    sram_control = 1'b0;
    
    #20
    ws_stb = 1'b0;
    sram_control = 1'b1;
    sram_in = 48'h000312345678;
    #1001
    ws_DDRack = 1'b1;
    #10
    ws_DDRack = 1'b0;
    #30
    ws_DDRack = 1'b1;
    ws_DDRdout = 512'h5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A5A;
    #10
    ws_DDRack = 1'b0;
    
    
    
    
    
end
endmodule
