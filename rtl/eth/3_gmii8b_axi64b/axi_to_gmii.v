//----------------------------------------------------------------------------------------
// File name:           axi_to_gmii
// Created by:          珊瑚伊斯特
// Created date:        2025.3
// Version:             V0.1
// Descriptions:        axi to gmii，使用FIFO实现数据缓存和时钟域转换
//2025.4 判断FIFO字节数如果字节数大于16字节就开始发送gmii_txd，直到整包数据发送完毕。保证gmii_txd是连续的。
//----------------------------------------------------------------------------------------

module axi_to_gmii(
    input                rst_n,          // 复位信号，低电平有效
    
    // AXI Stream 接口 (156.25MHz)
    input                tx_clk_out,     // AXI时钟域写时钟
    input                axis_tvalid,    // AXI数据有效信号
    input        [63:0]  axis_tdata,     // AXI数据
    input                axis_tlast,     // 包结束标志
    input        [7:0]   axis_tkeep,     // 字节有效掩码
    
    // GMII 接口 (125MHz)
    input                gmii_tx_clk,    // GMII时钟域读时钟
    output reg           gmii_tx_en,     // GMII发送数据有效
    output reg  [7:0]    gmii_txd        // GMII发送数据
);

// 参数定义
parameter START_THRESHOLD = 16;        // 开始发送阈值（字节）

// FIFO相关信号声明
reg  [7:0]  fifo_din;          // FIFO输入数据
reg         fifo_wr_en;        // FIFO写使能
wire [7:0]  fifo_dout;         // FIFO输出数据
wire        fifo_empty;        // FIFO空标志
wire        fifo_almost_empty; // FIFO几乎空标志
wire [10:0] wr_data_count;     // 写入数据计数
reg         fifo_rd_en;        // FIFO读使能

// 跨时钟域同步wr_data_count
(* ASYNC_REG = "TRUE" *) reg [7:0] sync_wr_count_1;  // 只需要8位，因为16字节只需要8位表示
(* ASYNC_REG = "TRUE" *) reg [7:0] sync_wr_count_2;  // 只需要8位，因为16字节只需要8位表示

//FIFO 8bit 2048深度，占18K
fifo_generator_0 fifo_inst (
    .rst(!rst_n),                    // 复位信号，高电平有效
    .wr_clk(tx_clk_out),            // 写时钟
    .rd_clk(gmii_tx_clk),           // 读时钟
    .din(fifo_din),                 // 写数据
    .wr_en(fifo_wr_en),             // 写使能
    .rd_en(fifo_rd_en),             // 读使能
    .dout(fifo_dout),               // 读数据
    .full(),                        // 未使用
    .almost_full(),                 // 未使用
    .empty(fifo_empty),             // 空标志
    .almost_empty(fifo_almost_empty),// 几乎空标志
    .wr_data_count(wr_data_count),  // 写入数据计数
    .wr_rst_busy(),                 // 未使用
    .rd_rst_busy()                  // 未使用
);

// ==================== 写时钟域(156.25MHz) ====================
reg [2:0]  byte_cnt;          // 字节计数器(0-7)
reg [63:0] data_reg;          // 数据寄存器
reg [7:0]  keep_reg;          // keep寄存器
reg        valid_reg;         // valid寄存器

// 写控制逻辑
always @(posedge tx_clk_out or negedge rst_n) begin
    if (!rst_n) begin
        byte_cnt <= 3'd0;
        fifo_wr_en <= 1'b0;
        fifo_din <= 8'd0;
        data_reg <= 64'd0;
        keep_reg <= 8'h0;
        valid_reg <= 1'b0;
    end else begin
        // 默认值
        fifo_wr_en <= 1'b0;
        
        if (axis_tvalid) begin
            // 新数据到达，更新寄存器
            data_reg <= axis_tdata;
            keep_reg <= axis_tkeep;
            valid_reg <= 1'b1;
            byte_cnt <= 3'd0;
        end else if (valid_reg) begin
            // 处理已缓存的数据，跳过低8bit节点信息
            case (byte_cnt)
                3'd0: begin
                    if (keep_reg[1]) begin  // 从[15:8]开始，跳过[7:0]
                        fifo_wr_en <= 1'b1;
                        fifo_din <= data_reg[15:8];
                    end
                    byte_cnt <= 3'd1;
                end
                3'd1: begin
                    if (keep_reg[2]) begin
                        fifo_wr_en <= 1'b1;
                        fifo_din <= data_reg[23:16];
                    end
                    byte_cnt <= 3'd2;
                end
                3'd2: begin
                    if (keep_reg[3]) begin
                        fifo_wr_en <= 1'b1;
                        fifo_din <= data_reg[31:24];
                    end
                    byte_cnt <= 3'd3;
                end
                3'd3: begin
                    if (keep_reg[4]) begin
                        fifo_wr_en <= 1'b1;
                        fifo_din <= data_reg[39:32];
                    end
                    byte_cnt <= 3'd4;
                end
                3'd4: begin
                    if (keep_reg[5]) begin
                        fifo_wr_en <= 1'b1;
                        fifo_din <= data_reg[47:40];
                    end
                    byte_cnt <= 3'd5;
                end
                3'd5: begin
                    if (keep_reg[6]) begin
                        fifo_wr_en <= 1'b1;
                        fifo_din <= data_reg[55:48];
                    end
                    byte_cnt <= 3'd6;
                end
                3'd6: begin
                    if (keep_reg[7]) begin
                        fifo_wr_en <= 1'b1;
                        fifo_din <= data_reg[63:56];
                    end
                    valid_reg <= 1'b0;  // 处理完当前数据
                end
            endcase
        end
    end
end

// 跨时钟域同步wr_data_count
always @(posedge gmii_tx_clk or negedge rst_n) begin
    if (!rst_n) begin
        sync_wr_count_1 <= 8'd0;
        sync_wr_count_2 <= 8'd0;
    end else begin
        sync_wr_count_1 <= wr_data_count[7:0];  // 只取低8位
        sync_wr_count_2 <= sync_wr_count_1;
    end
end

// ==================== 读时钟域(125MHz) ====================
reg        start_send;         // 开始发送标志
reg        start_send1;
reg        rd_en_cnt0;          
reg [1:0]  rd_en_cnt1;  
// 读控制逻辑
always @(posedge gmii_tx_clk or negedge rst_n) begin
    if (!rst_n) begin
        gmii_tx_en <= 1'b0;
        gmii_txd <= 8'd0;
        start_send <= 1'b0;
        fifo_rd_en <= 1'b0;
        rd_en_cnt0 <= 1'b0;
        rd_en_cnt1 <= 2'd0;
        start_send1 <= 1'b0;
    end else begin
        
        // 开始发送条件：FIFO数据量超过阈值且未开始发送
        if (sync_wr_count_2 >= START_THRESHOLD) begin
            start_send <= 1'b1;
            fifo_rd_en <= 1'b1; 
        end
        
        // 发送控制逻辑
        if (start_send) begin
            rd_en_cnt0 <=  1'b1;
            if (rd_en_cnt0 == 1'b1 ) begin
                start_send1 <= 1'b1;             //延时2个周期
            end
            if (start_send1) begin
                gmii_tx_en <= 1'b1;             //延时2个周期
                gmii_txd <= fifo_dout;
            end
            if (fifo_empty) begin
                rd_en_cnt1 <= rd_en_cnt1 + 1'b1;
                fifo_rd_en <= 1'b0;
                if (rd_en_cnt1 == 2'd2 ) begin  
                    gmii_txd <= 8'd0;
                    gmii_tx_en <= 1'b0; 
                    start_send <= 1'b0;
                    rd_en_cnt0 <= 1'b0;
                    rd_en_cnt1 <= 2'd0;
                    start_send1 <= 1'b0;
                end
            end
        end
    end
end
endmodule