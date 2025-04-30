//----------------------------------------------------------------------------------------
// File name:           gmii_to_axi.v
// Created by:          珊瑚伊斯特
// Created date:        2025.3
// Version:             V0.1
// Descriptions:        gmii_to_axi，两缓冲存储器结构。
//2025.4.25拼接数据，7字节+节点信息。
//----------------------------------------------------------------------------------------

module gmii_to_axi(
    input             gmii_rx_clk,    // 2.5MHZ,25MHZ,125MHz写时钟
    input             tx_clk_out,     // 156.25MHz读时钟
    input             rst_n,
    input             gmii_rx_dv,     // GMII数据有效
    input  [7:0]      gmii_rxd,       // GMII接收数据
    input  [3:0]      node_id,        // 节点ID输入
    input  [3:0]      eth_type,       // 以太网类型输入
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
            // 数据填充,置拼7个字节
            buffer[wr_buf_sel][(wr_cnt+1)*8 +:8] <= gmii_rxd;  // 第一个字节 (wr_cnt=0): 填充到 [63:56], 第二个字节 (wr_cnt=1): 填充到 [55:48], 以此类推
            buffer[wr_buf_sel][7:0] <= 8'b0;                  // 低8位固定为0
                                                             //第二个字节 (wr_cnt=1): 填充到 [15:8]
                                                             //第三个字节 (wr_cnt=2): 填充到 [23:16] ...以此类推                      
            if (wr_cnt == 3'd6) begin                // 缓冲区写满
                valid_bytes[wr_buf_sel] <= 3'd0;     // 0表示7个字节都有效
                wr_cnt <= 0;
                wr_buf_sel <= ~wr_buf_sel;          // 切换缓冲区，防止数据丢失
            end else begin
                wr_cnt <= wr_cnt + 1;
            end
        end 
        else if (!gmii_rx_dv && gmii_rx_dv0) begin      // 包结束处理
            if(wr_cnt != 3'd0) begin                    // 8字节不对齐结束
                valid_bytes[wr_buf_sel] <= wr_cnt+1;
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

// ==================== 跨时钟域同步 ====================
// 写指针同步到读时钟域
always @(posedge tx_clk_out) begin
    sync_wr_buf_sel[0] <= wr_buf_sel;
    sync_wr_buf_sel[1] <= sync_wr_buf_sel[0];
end

// ==================== 读时钟域逻辑（156.25MHz）====================
reg        rd_buf_sel;          // 读缓冲选择（0/1）
reg        last_delay;  // 用于延时tlast信号
(* ASYNC_REG = "TRUE" *) reg [1:0] sync_wr_buf_sel; // 写指针同步链


always @(posedge tx_clk_out or negedge rst_n) begin
    if (!rst_n) begin
        rd_buf_sel <= 0;
        axis_tvalid <= 1'b0;
        axis_tdata <= 64'd0;
        axis_tlast <= 1'b0;
        axis_tkeep <= 8'h00;
        last_delay <= 1'b0;
    end else begin
        if (axis_tready) begin  // 只有在下游准备好时才发送数据
            if (sync_wr_buf_sel[1] != rd_buf_sel) begin
                axis_tvalid <= 1'b1;
                axis_tkeep <= calc_tkeep(valid_bytes[rd_buf_sel]);
                
                // 数据补零的逻辑
                case (valid_bytes[rd_buf_sel])
                    3'd0: axis_tdata <= {buffer[rd_buf_sel][63:8], {node_id, eth_type}};
                    3'd2: axis_tdata <= {48'd0, buffer[rd_buf_sel][15:8], {node_id, eth_type}};
                    3'd3: axis_tdata <= {40'd0, buffer[rd_buf_sel][23:8], {node_id, eth_type}};
                    3'd4: axis_tdata <= {32'd0, buffer[rd_buf_sel][31:8], {node_id, eth_type}};
                    3'd5: axis_tdata <= {24'd0, buffer[rd_buf_sel][39:8], {node_id, eth_type}};
                    3'd6: axis_tdata <= {16'd0, buffer[rd_buf_sel][47:8], {node_id, eth_type}};
                    3'd7: axis_tdata <= {8'd0,  buffer[rd_buf_sel][55:8], {node_id, eth_type}};
                    default: axis_tdata <= {buffer[rd_buf_sel][63:8], {node_id, eth_type}};
                endcase

                last_delay <= pkt_end[rd_buf_sel];  // 延时一个周期
                axis_tlast <= 1'b0;  // 使用延时的last信号
                rd_buf_sel <= ~rd_buf_sel;
            end else if(last_delay) begin
                axis_tlast <= last_delay;
                axis_tvalid <= 1'b0;
            end else begin
                axis_tvalid <= 1'b0;  // 没有新数据时拉低valid
            end
        end else begin
            axis_tvalid <= 1'b0;  // 下游未准备好时拉低valid
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
endmodule