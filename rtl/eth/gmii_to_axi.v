//----------------------------------------------------------------------------------------
// File name:           gmii_to_axi.v
// Created by:          珊瑚伊斯特
// Created date:        2025.3
// Version:             V0.1
// Descriptions:        gmii_to_axi，两缓冲存储器结构。
//2025.4.12 修改：末尾64字节对齐时，补零。last_tkeep用于保存最后一个tkeep值，加在axis_tdata最后一个64bit数据后面。
//----------------------------------------------------------------------------------------

module gmii_to_axi(
    // input             dclk,           // 100MHz时钟
    input             gmii_rx_clk,    // 125MHz写时钟
    input             tx_clk_out,     // 156.25MHz读时钟
    input             rst_n,
    input             gmii_rx_dv,     // GMII数据有效
    input  [7:0]      gmii_rxd,       // GMII接收数据
    output reg        axis_tvalid,    // AXI有效信号
    output reg [63:0] axis_tdata,
    output reg        axis_tlast,     // 包结束标志
    output reg [7:0]  axis_tkeep,     // 字节有效掩码
    input             axis_tready     // 下游就绪信号
);

// ==================== 写时钟域逻辑（125MHz）====================
reg [63:0] buffer [0:1];        // 双缓冲存储器
reg        wr_buf_sel;          // 写缓冲选择（0/1）
reg [2:0]  wr_cnt;              // 字节计数器（0-7）
reg [2:0]  valid_bytes [0:1];   // 各缓冲有效字节数
reg        pkt_end [0:1];       // 包结束标志
reg        gmii_rx_dv0;
reg        gmii_rx_dv1;

// 写控制逻辑
always @(posedge gmii_rx_clk or negedge rst_n) begin
    if (!rst_n) begin
        gmii_rx_dv0 <= 1'b0;
    end else begin
        gmii_rx_dv0 <= gmii_rx_dv;
        gmii_rx_dv1 <= gmii_rx_dv0;
    end
end
always @(posedge gmii_rx_clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_buf_sel <= 0;
        wr_cnt <= 0;
        buffer[0] <= 64'd0;
        buffer[1] <= 64'd0;
        valid_bytes[0] <= 3'd0;
        valid_bytes[1] <= 3'd0;
        pkt_end[0] <= 1'b0;
        pkt_end[1] <= 1'b0;
    end else begin
        if (gmii_rx_dv) begin
            pkt_end[wr_buf_sel] <= 1'b0;
            // 数据填充
            buffer[wr_buf_sel][wr_cnt*8 +:8] <= gmii_rxd;    //第一个字节 (wr_cnt=0): 填充到 [7:0]
                                                             //第二个字节 (wr_cnt=1): 填充到 [15:8]
                                                             //第三个字节 (wr_cnt=2): 填充到 [23:16] ...以此类推                      
            if (wr_cnt == 3'd7) begin                // 缓冲区写满
                valid_bytes[wr_buf_sel] <= 3'd0;     // 修改这里：0表示8个字节都有效
                wr_cnt <= 0;
                wr_buf_sel <= ~wr_buf_sel;          // 切换缓冲区，防止数据丢失
            end else begin
                wr_cnt <= wr_cnt + 1;
            end
        end 
        else if (!gmii_rx_dv && gmii_rx_dv0) begin      // 包结束处理
            if(wr_cnt != 3'd0) begin                    // 8字节不对齐结束
                valid_bytes[wr_buf_sel] <= wr_cnt;
                pkt_end[wr_buf_sel] <= 1'b1;
                wr_cnt <= 0;
                wr_buf_sel <= ~wr_buf_sel;
            end else begin                              // 正好8字节对齐结束
                // valid_bytes[wr_buf_sel] <= 3'd0;
                pkt_end[~wr_buf_sel] <= 1'b1;
            end    
        end
    end
end

// ==================== 读时钟域逻辑（156.25MHz）====================
reg        rd_buf_sel;          // 读缓冲选择（0/1）
reg        last_delay;  // 用于延时tlast信号
reg        need_padding;        // 是否需要发送padding数据
reg [7:0]  last_tkeep;         // 保存最后一个tkeep值
(* ASYNC_REG = "TRUE" *) reg [1:0] sync_wr_buf_sel; // 写指针同步链
reg        need_last;           // 指示是否需要发送最后的tlast

// 在复位部分添加新信号初始化
always @(posedge tx_clk_out or negedge rst_n) begin
    if (!rst_n) begin
        rd_buf_sel <= 0;
        axis_tvalid <= 1'b0;
        axis_tdata <= 64'd0;
        axis_tlast <= 1'b0;
        axis_tkeep <= 8'h00;
        last_delay <= 1'b0;
        need_padding <= 1'b0;
        last_tkeep <= 8'h00;
        need_last <= 1'b0;
    end else begin
        if (axis_tready) begin
            if (sync_wr_buf_sel[1] != rd_buf_sel) begin
                axis_tvalid <= 1'b1;
                axis_tkeep <= calc_tkeep(valid_bytes[rd_buf_sel]);
                
                // 数据补零的逻辑
                case (valid_bytes[rd_buf_sel])
                    3'd0: axis_tdata <= buffer[rd_buf_sel];
                    3'd1: axis_tdata <= {56'd0, buffer[rd_buf_sel][7:0]};
                    3'd2: axis_tdata <= {48'd0, buffer[rd_buf_sel][15:0]};
                    3'd3: axis_tdata <= {40'd0, buffer[rd_buf_sel][23:0]};
                    3'd4: axis_tdata <= {32'd0, buffer[rd_buf_sel][31:0]};
                    3'd5: axis_tdata <= {24'd0, buffer[rd_buf_sel][39:0]};
                    3'd6: axis_tdata <= {16'd0, buffer[rd_buf_sel][47:0]};
                    3'd7: axis_tdata <= {8'd0,  buffer[rd_buf_sel][55:0]};
                    default: axis_tdata <= buffer[rd_buf_sel];
                endcase
                
                if (pkt_end[rd_buf_sel]) begin
                    need_padding <= 1'b1;
                    need_last <= 1'b0;
                    last_tkeep <= calc_tkeep(valid_bytes[rd_buf_sel]);
                    axis_tlast <= 1'b0;
                end else begin
                    need_padding <= 1'b0;
                    need_last <= 1'b0;
                    axis_tlast <= 1'b0;
                end
                rd_buf_sel <= ~rd_buf_sel;
            end else if (need_padding) begin
                // 发送额外的64bit数据，包含tkeep信息
                axis_tvalid <= 1'b1;
                axis_tdata <= {56'd0, last_tkeep};
                axis_tkeep <= 8'hFF;
                axis_tlast <= 1'b0;      // 发送额外数据时保持tlast为0
                need_padding <= 1'b0;
                need_last <= 1'b1;       // 标记下一拍需要发送tlast
            end else if (need_last) begin
                // 发送最后的tlast信号
                axis_tvalid <= 1'b1;
                axis_tdata <= 64'd0;     // 数据全0
                axis_tkeep <= 8'hFF;
                axis_tlast <= 1'b1;      // 最后一拍置1
                need_last <= 1'b0;
            end else begin
                axis_tvalid <= 1'b0;
                axis_tlast <= 1'b0;
            end
        end else begin
            axis_tvalid <= 1'b0;
        end
    end
end

// tkeep生成函数
function [7:0] calc_tkeep(input [2:0] bytes);
    case(bytes)
        3'd0: calc_tkeep = 8'hFF;    // 8个有效字节
        3'd1: calc_tkeep = 8'h01;    // 1个有效字节
        3'd2: calc_tkeep = 8'h03;    // 2个有效字节
        3'd3: calc_tkeep = 8'h07;    // 3个有效字节
        3'd4: calc_tkeep = 8'h0F;    // 4个有效字节
        3'd5: calc_tkeep = 8'h1F;    // 5个有效字节
        3'd6: calc_tkeep = 8'h3F;    // 6个有效字节
        3'd7: calc_tkeep = 8'h7F;    // 7个有效字节
        default: calc_tkeep = 8'hFF;  // 防止异常情况
    endcase
endfunction

// ==================== 跨时钟域同步 ====================
// 写指针同步到读时钟域
always @(posedge tx_clk_out) begin
    sync_wr_buf_sel[0] <= wr_buf_sel;
    sync_wr_buf_sel[1] <= sync_wr_buf_sel[0];
    
end
endmodule