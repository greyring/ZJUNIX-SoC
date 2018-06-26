module SRAM(
    input wire clk,  // main clock
    input wire rst,  // synchronous reset

    // SRAM interfaces
    (* IOB="true" *)
    output reg [2:0]sram_ce_n,
    (* IOB="true" *)
    output reg [2:0]sram_oe_n,
    (* IOB="true" *)
    output reg [2:0]sram_we_n,
    (* IOB="true" *)
    output reg [2:0]sram_ub_n,
    (* IOB="true" *)    
    output reg [2:0]sram_lb_n,
    (* IOB="true" *)
    output reg [19:0] sram_addr,
    inout wire [47:0] sram_data,
    
    // WishBone Bus
    input wire wb_stb,  // chip select
    input wire [31:0] wb_addr,  // address every 4 == 48bit
    input wire [5:0] wb_we,
    input wire [47:0] wb_din,
    output wire [47:0] wb_dout,
    output reg wb_nak
    );
    
    (* IOB="true" *)
    reg [47:0] sram_dout;
    assign
            sram_data = (&sram_we_n) ? {48{1'bz}} : sram_dout,
            wb_dout = sram_data;
            
    localparam
        S_IDLE = 0,  // idle
        S_READ = 1,  // read data
        S_READ_D1 = 2,//read delay
        S_READ_D2 = 3, 
        S_READ_D3 = 4,
        S_READ_D4 = 5,
        S_READ_RES = 6, //read result
        S_WRITE = 7,  // send write address
        S_WRITE_2 = 8,
        S_WRITE_D = 9, // set we
        S_WRITE_D2 = 10,
        S_WRITE_RES = 11, //close we
        S_WRITE_RES2 = 12;

    reg [3:0] state = 0;
    reg [3:0] next_state;
    reg [5:0] bus_we;
    
    always @(*) begin
        next_state = S_IDLE;
        case (state)
            S_IDLE: begin
                if (wb_stb)
                    if (|wb_we)
                       next_state = S_WRITE;
                    else
                       next_state = S_READ;
                else
                    next_state = S_IDLE;
            end
            S_READ: next_state = S_READ_D1;
            S_READ_D1: next_state = S_READ_D2;
            S_READ_D2: next_state = S_READ_D3;
            S_READ_D3: next_state = S_READ_D4;
            S_READ_D4: next_state = S_READ_RES;
            S_READ_RES: begin
                 if (wb_stb)
                    if (|wb_we)
                       next_state = S_WRITE;
                    else
                       next_state = S_READ;
                 else
                    next_state = S_IDLE;
            end
            S_WRITE: next_state = S_WRITE_2;
            S_WRITE_2: next_state = S_WRITE_D;
            S_WRITE_D: next_state = S_WRITE_D2;
            S_WRITE_D2: next_state = S_WRITE_RES;
            S_WRITE_RES: next_state = S_WRITE_RES2;
            S_WRITE_RES2: begin
                 if (wb_stb)
                     if (|wb_we)
                         next_state = S_WRITE;
                     else
                         next_state = S_READ;
                 else
                     next_state = S_IDLE;
            end
        endcase
    end
    
    always @(posedge clk) begin
        if (rst)
            state <= S_IDLE;
        else
            state <= next_state;
    end
    
    always @(posedge clk) begin
        if (~rst) case (next_state)
            S_IDLE: begin
                wb_nak <= 1'b0;
                sram_ce_n <= 3'b111;
                sram_oe_n <= 3'b111;
                sram_we_n <= 3'b111;
                sram_ub_n <= 3'b111;
                sram_lb_n <= 3'b111;
            end
            S_READ: begin
                 wb_nak <= 1'b1;
                 sram_ce_n <= 3'b000;
                 sram_oe_n <= 3'b000;
                 sram_we_n <= 3'b111;
                 sram_ub_n <= 3'b000;
                 sram_lb_n <= 3'b000;
                 sram_addr <= wb_addr[21:2];
            end
            S_READ_RES: wb_nak <= 1'b0;
            S_WRITE: begin
                wb_nak <= 1'b1;
                sram_ce_n <= 3'b000;
                sram_oe_n <= 3'b111;
                sram_we_n <= 3'b111;
                sram_ub_n <= {~wb_we[5], ~wb_we[3], ~wb_we[1]};
                sram_lb_n <= {~wb_we[4], ~wb_we[2], ~wb_we[0]};
                sram_addr <= wb_addr[21:2];
                sram_dout <= wb_din;
                bus_we <= wb_we;
            end
            S_WRITE_D: begin
                sram_we_n <= {~(bus_we[5] | bus_we[4]), ~(bus_we[3] | bus_we[2]), ~(bus_we[1] | bus_we[0])};
            end
            S_WRITE_RES: sram_we_n <= 3'b111;
            S_WRITE_RES2: wb_nak <= 1'b0;
            default: begin
                wb_nak <= wb_nak;
                sram_ce_n <= sram_ce_n;
                sram_oe_n <= sram_oe_n;
                sram_we_n <= sram_we_n;
                sram_ub_n <= sram_ub_n;
                sram_lb_n <= sram_lb_n;
                sram_addr <= sram_addr;
                sram_dout <= sram_dout;
                bus_we <= bus_we;
            end
        endcase
    end
    
endmodule
