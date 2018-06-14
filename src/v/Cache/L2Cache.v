`timescale 1ns / 1ps

module L2Cache(
    input clk,
    input rst,
    //Wishbone slave interface
    input [31:0] ws_addr, input [511:0] ws_din,
    input [63:0] ws_dm, input ws_stb, input ws_we,
    output reg ws_ack = 0, output reg [511:0] ws_dout,
    
    //Wishbone DDR interface
    output reg [31:0] ws_DDRaddr, output reg  [511:0] ws_DDRdin,
    output reg [63:0] ws_DDRdm, output ws_DDRcyc, output reg ws_DDRstb, output reg ws_DDRwe,
    input ws_DDRack, input [511:0] ws_DDRdout,
    
    //Wishbone SRAM interface
    output reg [31:0] ws_SRAMaddr, output reg [767:0] ws_SRAMdin,
    output reg [95:0] ws_SRAMdm,   
    output reg ws_SRAMstb,     
    output reg ws_SRAMwe,
    input ws_SRAMack, 
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
    
    generate
    genvar i;
    for (i = 0; i<16; i=i+1) begin
        always @(posedge clk)
            sramData[i] = ws_SRAMack ? ws_SRAMdout[i*48+31:i*48] : sramData[i];
        assign sramData_r[i*32+31:i*32] = sramData[i];
    end
    
    assign sramDm_w[5:0] = {2'b11, wrDm[3:0]};
    assign sramData_w[47:0] = {4'b0, wrAddr[31:22], 2'b11, wrDin[31:0]};
    assign sramData_rw[47:0] = {4'b0, wrAddr[31:22], 2'b01, ws_DDRdout[31:0]};
    for (i = 1; i<16; i=i+1) begin
        assign sramDm_w[i*6+5:i*6] = {2'b0, wrDm[i*4+3:i*4]};
        assign sramData_w[i*48+47:i*48] = {16'b0, wrDin[i*32+31:i*32]};
        assign sramData_rw[i*48+47:i*48] = {16'b0, ws_DDRdout[i*32+31:i*32]};
    end
    
    
    for (i = 0; i<16; i=i+1)
        assign ddrData_w = sramData[i];
    endgenerate
    
    always @(posedge clk)
        sramLabel = ws_SRAMack ? ws_SRAMdout[43:32] : sramLabel;
        
    reg [3:0]state, nxtstate;
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
                //if (ws_SRAMaddr == 32'h00FFFFC0)
                if (ws_SRAMaddr == 32'h00000040) begin
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
                        ws_ack <= 1'bb1;
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
    wire [676:0]ws_SRAMdin;
    wire [95:0]ws_SRAMdm;
    wire ws_SRAMstb;
    wire ws_SRAMwe;
    wire ws_SRAMack;
    wire [676:0]ws_SRAMdout;
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
    #1030
    ws_stb = 1'b1;
    ws_we = 1'b0;
    ws_addr = 32'h003FFFC0;
end
endmodule
