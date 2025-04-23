//----------------------------------------------------------------------------------------
// File name:           gmii_to_axi.v
// Created by:          珊瑚伊斯特
// Created date:        2025.3
// Version:             V0.1
// Descriptions:        gmii_to_axi，FIFO结构。
//2025.4.12 修改：末尾64字节对齐时，补零。
//数据拼接成64b数据。
//----------------------------------------------------------------------------------------

module gmii_to_axi #(
    parameter NODE_ID = 4'b0001,  // 节点ID
    parameter ETH_TYPE = 4'b0001  // 以太网数据类型
)(
    input             gmii_rx_clk,    // 125MHz写时钟
    input             tx_clk_out,     // 156.25MHz读时钟
    input             rst_n,
    input             gmii_rx_dv,     // GMII数据有效
    input  [7:0]      gmii_rxd,       // GMII接收数据
    output reg        axis_tvalid,    // AXI有效信号
    output reg [63:0] axis_tdata,     // AXI数据
    output reg        axis_tlast,     // 包结束标志
    output reg [7:0]  axis_tkeep,     // 字节有效掩码
    input             axis_tready     // 下游就绪信号
);

// ==================== 状态机定义 =====================
localparam IDLE = 3'd0;      // 空闲状态
localparam COLLECT = 3'd1;   // 收集数据状态
localparam SEND = 3'd2;      // 发送数据状态
localparam PADDING = 3'd3;   // 补零状态
localparam LAST = 3'd4;      // 发送最后一个数据状态

// ==================== 内部信号定义 =====================
wire        fifo_rst;
wire        fifo_wr_en;
wire [7:0]  fifo_din;
wire        fifo_rd_en;
wire [63:0] fifo_dout;
wire        fifo_full;
wire        fifo_empty;
wire [6:0]  fifo_rd_count;
wire [9:0]  fifo_wr_count;
wire        fifo_wr_rst_busy;
wire        fifo_rd_rst_busy;

// FIFO复位信号
assign fifo_rst = ~rst_n;

// FIFO IP核实例化
fifo_generator_0 u_fifo (
    .rst(fifo_rst),                      // 复位信号
    .wr_clk(gmii_rx_clk),               // 写时钟
    .rd_clk(tx_clk_out),                // 读时钟
    .din(fifo_din),                     // 写数据
    .wr_en(fifo_wr_en),                 // 写使能
    .rd_en(fifo_rd_en),                 // 读使能
    .dout(fifo_dout),                   // 读数据
    .full(fifo_full),                   // FIFO满标志
    .almost_full(),                     // 几乎满标志（未使用）
    .empty(fifo_empty),                 // FIFO空标志
    .almost_empty(),                    // 几乎空标志（未使用）
    .rd_data_count(fifo_rd_count),      // 读数据计数
    .wr_data_count(fifo_wr_count),      // 写数据计数
    .wr_rst_busy(fifo_wr_rst_busy),     // 写复位忙标志
    .rd_rst_busy(fifo_rd_rst_busy)      // 读复位忙标志
);

// ==================== 写时钟域逻辑（125MHz）====================
reg [2:0]  wr_cnt;               // 字节计数器（0-7）
reg [63:0] temp_data;            // 临时数据寄存器
reg        pkt_end;              // 包结束标志

// 写控制逻辑
always @(posedge gmii_rx_clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_cnt <= 3'd0;
        temp_data <= 64'd0;
        pkt_end <= 1'b0;
        fifo_wr_en <= 1'b0;
        fifo_din <= 8'd0;
    end else begin
        if (gmii_rx_dv) begin
            // 数据填充到临时寄存器
            temp_data[wr_cnt*8 +:8] <= gmii_rxd;
            fifo_wr_en <= 1'b0;
            
            if (wr_cnt == 3'd7) begin
                // 8字节数据收集完成，写入FIFO
                if (!fifo_full) begin
                    fifo_din <= {temp_data[63:8], {NODE_ID, ETH_TYPE}};
                    fifo_wr_en <= 1'b1;
                end
                wr_cnt <= 3'd0;
            end else begin
                wr_cnt <= wr_cnt + 1'b1;
            end
        end else if (!gmii_rx_dv && wr_cnt != 3'd0) begin
            // 包结束处理，补零
            temp_data[wr_cnt*8 +:8] <= 8'd0;
            if (wr_cnt == 3'd7) begin
                if (!fifo_full) begin
                    fifo_din <= {temp_data[63:8], {NODE_ID, ETH_TYPE}};
                    fifo_wr_en <= 1'b1;
                end
                wr_cnt <= 3'd0;
            end else begin
                wr_cnt <= wr_cnt + 1'b1;
            end
        end else begin
            fifo_wr_en <= 1'b0;
        end
    end
end

// ==================== 读时钟域逻辑（156.25MHz）====================
reg [2:0] state;                // 状态机状态
reg [2:0] next_state;           // 下一状态
reg [63:0] send_data;           // 发送数据寄存器
reg        send_valid;          // 发送有效标志
reg        send_last;           // 发送最后一个数据标志

// 状态机转换逻辑
always @(posedge tx_clk_out or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        next_state <= IDLE;
        send_data <= 64'd0;
        send_valid <= 1'b0;
        send_last <= 1'b0;
        axis_tvalid <= 1'b0;
        axis_tdata <= 64'd0;
        axis_tlast <= 1'b0;
        axis_tkeep <= 8'h00;
        fifo_rd_en <= 1'b0;
    end else begin
        case (state)
            IDLE: begin
                if (!fifo_empty) begin
                    state <= SEND;
                    fifo_rd_en <= 1'b1;
                    send_valid <= 1'b1;
                    send_last <= (fifo_rd_count == 7'd1);
                end
            end
            
            SEND: begin
                if (axis_tready) begin
                    axis_tvalid <= 1'b1;
                    axis_tdata <= fifo_dout;
                    axis_tkeep <= 8'hFF;
                    axis_tlast <= send_last;
                    fifo_rd_en <= 1'b0;
                    
                    if (send_last) begin
                        state <= IDLE;
                        send_valid <= 1'b0;
                    end else if (!fifo_empty) begin
                        fifo_rd_en <= 1'b1;
                        send_last <= (fifo_rd_count == 7'd1);
                    end else begin
                        state <= IDLE;
                        send_valid <= 1'b0;
                    end
                end
            end
            
            default: state <= IDLE;
        endcase
    end
end

endmodule