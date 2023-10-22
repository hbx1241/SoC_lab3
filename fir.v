`timescale 1ns / 1ps
module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  reg awready,
    output  reg wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  reg arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  reg rvalid,
    output  reg [(pDATA_WIDTH-1):0] rdata,    
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  reg                      ss_tready, 
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  reg [3:0]               tap_WE,
    output  wire tap_EN,
    output  reg [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  reg [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire  [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);
    reg [7:0] config_reg;
    reg [2:0] ap_config, ap_config_next;
    reg [2:0] state, state_next;
    reg [31:0] data_len, data_len_next;
    reg [pADDR_WIDTH-1:0] tap_r_addr, tap_w_addr, data_addr, data_addr_next, tap_addr, tap_addr_next;
    reg [pDATA_WIDTH-1:0] fir_out, fir_out_next, rdata_next;
    reg ss_tready_next;
    parameter s_idle = 0, s_get = 1, s_comp = 2, s_out = 3, s_next = 4, s_last = 5;

    assign tap_A = ((state == s_comp) || (state == s_get)) ? (tap_addr << 2) : (awvalid && wvalid) ? tap_w_addr : tap_r_addr;
    assign data_A = data_addr << 2;
    assign tap_EN = 1;
    assign data_EN = 1;
    assign data_Di = (state == s_idle) ? 32'b0 : ss_tdata;
    assign sm_tdata = fir_out;
    assign sm_tvalid = state == s_out;
    assign sm_tlast = state == s_last;

    always@(*) begin
        ap_config_next = ap_config;
        data_len_next = data_len;
        tap_w_addr = 12'h0;
        tap_WE = 4'b0;
        tap_Di = 32'b0;
        if (awvalid && wvalid && awready && wready) begin
            if (awaddr == 12'h0) begin
                if (ap_config[2] && (!ap_config[1])) ap_config_next = {~wdata[0], 1'b0, wdata[0]};
                else if (ap_config[0]) ap_config_next = {1'b0, ap_config[1], 1'b0};
            end 
            else if (awaddr == 12'h010) begin
                data_len_next = wdata;
            end
            else if (awaddr >= 12'h020) begin
                tap_w_addr = awaddr - 12'h20;
                tap_WE = 4'hf;
                tap_Di = wdata;
            end
        end
        if (arvalid && arready) begin
            if (araddr == 12'h0) ap_config_next = {ap_config[2], 1'b0, ap_config[0]};
        end
        if ((state == s_out) && ss_tlast) ap_config_next = {1'b1, 1'b1, 1'b0};
    end

    always@(*) begin
        rdata_next = rdata;
        tap_r_addr = 12'hfff;
        if (arvalid && arready) begin
            if (araddr == 12'h0) begin
                rdata_next = {5'b0, ap_config};
            end
            else if(araddr == 12'h010) begin
                rdata_next = data_len;
            end
            else if(araddr >= 12'h020) begin
                tap_r_addr = araddr - 12'h20;
                rdata_next = tap_Do;
            end
        end
    end
    
    always@(*) begin
        state_next = s_idle;
        if (state == s_idle) begin
            if (ap_config[0] && ss_tvalid) state_next = s_get;
            else state_next = s_idle;
        end
        else if (state == s_get) begin
            state_next = s_comp;
        end
        else if (state == s_comp) begin
            if (tap_addr == 12'd10) state_next = s_out;
            else state_next = s_comp;
        end
        else if (state == s_out) begin
            if (sm_tready) begin
                if (ss_tlast) state_next = s_last;
                else state_next = s_next;
            end
            else state_next = s_out;
        end
        else if (state == s_next) state_next = s_get;
        else if (state == s_last) state_next = s_last;
    end

    always@(*) begin
        data_addr_next = data_addr;
        if ((state == s_comp)) begin
            if (data_addr == 12'd0)
                data_addr_next = 12'd10;
            else data_addr_next = data_addr - 12'd1;
        end
        else if ((state == s_next) || (state == s_idle)) begin
            if (data_addr == 12'd10) data_addr_next = 12'd0;
            else data_addr_next = data_addr_next + 12'd1;
        end
    end

    always@(*) begin
        if ((state == s_idle) || (state == s_get)) data_WE = 4'hf;
        else data_WE = 4'h0;
    end
    always@(*) begin
        tap_addr_next = 0;
        if (state == s_comp) begin
            if (tap_addr == 12'd10) begin
                tap_addr_next = 12'd0;
            end
            else tap_addr_next = tap_addr + 12'd1;
        end
    end

    always@(*) begin
        fir_out_next = fir_out + (tap_Do * data_Do);
    end

    always@(posedge axis_clk) begin
        if (awvalid && wvalid) begin
            awready <= 1'b1;
            wready <= 1'b1;
        end
        else begin
            awready <= 1'b0;
            wready <= 1'b0;
        end
    end

    always@(posedge axis_clk) begin
        if (arvalid) begin
            arready <= 1'b1;
        end
        else arready <= 1'b0;
    end

    always@(posedge axis_clk) begin
        if ((arvalid && arready)) 
            rvalid <= 1'b1;
        else if (!rready) 
            rvalid <= rvalid;
        else
            rvalid <= 1'b0;
    end

    always@(*) ss_tready_next = state_next == s_next ? 1'b1: 1'b0;

    always@(posedge axis_clk or negedge axis_rst_n) begin
        if (!axis_rst_n) begin
            ap_config <= 3'b100;
            state <= s_idle;
            data_addr <= 12'h0;
            tap_addr <= 12'd0;
            fir_out <= 32'b0;
            data_len <= 32'b0;
            rdata <= 32'b0;
            ss_tready <= 1'b0;
            fir_out <= 32'b0;
        end
        else begin
            ap_config <= ap_config_next;
            state <= state_next;
            data_addr <= data_addr_next;
            tap_addr <= tap_addr_next;
            if ((state == s_next) || (state == s_idle))
                fir_out <= 32'b0;
            else fir_out <= fir_out_next;
            data_len <= data_len_next;
            rdata <= rdata_next;
            ss_tready <= ss_tready_next;
        end
    end
endmodule