//----------------------------------------------------------------------------------------
// File name:           axi_to_gmii
// Created by:          珊瑚伊斯特
// Created date:        2025.3
// Version:             V0.1
// Descriptions:        axi to gmii，三缓冲存储器结构。确保gmii_txd连续输出。
//数据拆分，将64b数据拆分成8b数据。
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

// 在模块顶部声明区域添加
reg [2:0] clear_valid_req;     // 读时钟域请求清除valid
(* ASYNC_REG = "TRUE" *) reg [2:0] sync_clear_req_1, sync_clear_req_2;  // 写时钟域同步

// ==================== 写时钟域(156.25MHz) ====================
reg [63:0] buffer [0:2];        // 三缓冲存储器
reg [7:0]  keep_buffer [0:2];   // 保存tkeep信息
reg        last_buffer [0:2];   // 保存tlast信息
reg        data_valid [0:2];    // 缓冲区数据有效标志
reg [1:0]  wr_buf_sel;          // 写缓冲选择
reg [1:0]  last_data_count;     // 最后一个数据计数器  
reg        axis_tvalid0;
reg [7:0]  last_keep;
reg        data_end;

// 写控制逻辑
always @(posedge tx_clk_out or negedge rst_n) begin
    if (!rst_n) begin
        wr_buf_sel <= 2'd0;
        buffer[0] <= 64'd0;
        buffer[1] <= 64'd0;
        buffer[2] <= 64'd0;
        keep_buffer[0] <= 8'h0;
        keep_buffer[1] <= 8'h0;
        keep_buffer[2] <= 8'h0;
        last_buffer[0] <= 1'b0;
        last_buffer[1] <= 1'b0;
        last_buffer[2] <= 1'b0;
        data_valid[0] <= 1'b0;
        data_valid[1] <= 1'b0;
        data_valid[2] <= 1'b0;
        sync_clear_req_1 <= 3'b0;
        sync_clear_req_2 <= 3'b0;
        last_data_count <= 2'd0;
        axis_tvalid0 <= 1'b0;
        last_keep <= 8'd0;
        data_end <= 1'b0;
    end else begin
        // 同步clear_valid_req信号
        sync_clear_req_1 <= clear_valid_req;
        sync_clear_req_2 <= sync_clear_req_1;
        axis_tvalid0 <= axis_tvalid;
        
        // 处理清除请求
        if (sync_clear_req_2[0]) data_valid[0] <= 1'b0;
        if (sync_clear_req_2[1]) data_valid[1] <= 1'b0;
        if (sync_clear_req_2[2]) data_valid[2] <= 1'b0;

        if (axis_tvalid) begin
            // 数据写入当前缓冲区
            last_data_count <= 2'd0;
            data_end <= 1'b0;
            if (axis_tvalid && (!axis_tvalid0)) begin
                buffer[wr_buf_sel] <= axis_tdata;
                keep_buffer[wr_buf_sel] <= axis_tkeep;
                data_valid[wr_buf_sel] <= 1'b1;
                last_buffer[wr_buf_sel] <= axis_tlast;
                wr_buf_sel <= (wr_buf_sel == 2'd2) ? 2'd0 : wr_buf_sel + 1'b1;  // 切换缓冲区
            end 
            if (!axis_tlast && axis_tvalid) begin
                last_data_count <= last_data_count + 1'b1;
            end   
            if (last_data_count == 2) begin
                last_keep <= axis_tdata[7:0];
                data_end <= 1'b1;
            end else begin
                last_keep <= 8'hff;
            end
        end
    end
end

// ==================== 读时钟域(125MHz) ====================
reg [2:0]  rd_cnt;             // 字节计数器(0-7)
reg [1:0]  rd_buf_sel;         // 读缓冲选择
reg [63:0] rd_data;            // 读数据寄存器
reg [7:0]  rd_keep;            // 读keep寄存器
reg        rd_last;            // 读last寄存器
reg        rd_valid;           // 读有效标志
reg        start_send;         // 开始发送标志
reg [1:0]  valid_data_count;   // 有效数据计数器
reg        packet_sending;     // 数据包发送中标志

// 有效数据计数器控制
always @(posedge gmii_tx_clk or negedge rst_n) begin
    if (!rst_n) begin
        valid_data_count <= 2'd0;
    end else begin
        // 更新有效数据计数
        case ({data_valid[0], data_valid[1], data_valid[2]})
            3'b000: valid_data_count <= 2'd0;
            3'b001, 3'b010, 3'b100: valid_data_count <= 2'd1;
            3'b011, 3'b101, 3'b110: valid_data_count <= 2'd2;
            3'b111: valid_data_count <= 2'd3;
        endcase
    end
end

// 读控制逻辑
always @(posedge gmii_tx_clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_cnt <= 3'd0;
        rd_buf_sel <= 2'd0;
        rd_data <= 64'd0;
        rd_keep <= 8'h0;
        rd_last <= 1'b0;
        packet_sending <= 1'b0;
        gmii_tx_en <= 1'b0;
        gmii_txd <= 8'd0;
        clear_valid_req <= 3'b0;
    end else begin
        // 默认清除请求信号
        clear_valid_req <= 3'b0;

        // 数据包发送启动逻辑
        if (!packet_sending && valid_data_count >= 2'd2) begin
            packet_sending <= 1'b1;
            rd_data <= buffer[rd_buf_sel];
            rd_keep <= keep_buffer[rd_buf_sel];
            rd_last <= last_buffer[rd_buf_sel];
            clear_valid_req[rd_buf_sel] <= 1'b1;  // 请求清除valid
            rd_cnt <= 3'd0;
            rd_buf_sel <= (rd_buf_sel == 2'd2) ? 2'd0 : rd_buf_sel + 1'b1;
        end

        // 数据发送逻辑
        if (packet_sending) begin
            // 数据输出，根据rd_keep判断是否输出有效数据
            case (rd_cnt)
                3'd0: begin
                    if (rd_keep[0])  begin                  // 仅在rd_keep为1时才输出数据
                        gmii_tx_en <= 1'b1;
                        gmii_txd <= rd_data[7:0];
                    end else begin
                        gmii_tx_en <= 1'b0;
                        gmii_txd <= 8'h00;
                    end    
                end
                3'd1: begin
                    if (rd_keep[1])  begin                  // 仅在rd_keep为1时才输出数据
                        gmii_tx_en <= 1'b1;
                        gmii_txd <= rd_data[15:8];
                    end else begin
                        gmii_tx_en <= 1'b0;
                        gmii_txd <= 8'h00;
                    end    
                end
                3'd2: begin
                    if (rd_keep[2])  begin                  // 仅在rd_keep为1时才输出数据
                        gmii_tx_en <= 1'b1;
                        gmii_txd <= rd_data[23:16];
                    end else begin
                        gmii_tx_en <= 1'b0;
                        gmii_txd <= 8'h00;
                    end    
                end
                3'd3: begin
                    if (rd_keep[3])  begin                  // 仅在rd_keep为1时才输出数据
                        gmii_tx_en <= 1'b1;
                        gmii_txd <= rd_data[31:24];
                    end else begin
                        gmii_tx_en <= 1'b0;
                        gmii_txd <= 8'h00;
                    end    
                end
                3'd4: begin
                    if (rd_keep[4])  begin                  // 仅在rd_keep为1时才输出数据
                        gmii_tx_en <= 1'b1;
                        gmii_txd <= rd_data[39:32];
                    end else begin
                        gmii_tx_en <= 1'b0;
                        gmii_txd <= 8'h00;
                    end    
                end
                3'd5: begin
                    if (rd_keep[5])  begin                  // 仅在rd_keep为1时才输出数据
                        gmii_tx_en <= 1'b1;
                        gmii_txd <= rd_data[47:40];
                    end else begin
                        gmii_tx_en <= 1'b0;
                        gmii_txd <= 8'h00;
                    end    
                end
                3'd6: begin
                    if (rd_keep[6])  begin                  // 仅在rd_keep为1时才输出数据
                        gmii_tx_en <= 1'b1;
                        gmii_txd <= rd_data[55:48];
                    end else begin
                        gmii_tx_en <= 1'b0;
                        gmii_txd <= 8'h00;
                    end    
                end
                3'd7: begin
                    if (rd_keep[7]) begin
                        gmii_tx_en <= 1'b1;
                        gmii_txd <= rd_data[63:56];
                    end else begin
                        gmii_tx_en <= 1'b0;
                        gmii_txd <= 8'h00;
                    end

                    // 检查数据包结束条件
                    if (valid_data_count == 2'd0) begin
                            rd_keep <= 8'h0;  // 清除keep信号，强制下一周期tx_en为0
                            packet_sending <= 1'b0;
                    end else if (last_keep == 8'hff) begin
                            rd_data <= buffer[rd_buf_sel];
                            rd_keep <= keep_buffer[rd_buf_sel];
                            rd_last <= last_buffer[rd_buf_sel];
                            clear_valid_req[rd_buf_sel] <= 1'b1;
                            rd_buf_sel <= (rd_buf_sel == 2'd2) ? 2'd0 : rd_buf_sel + 1'b1;
                        end else if (last_keep != 8'hff) begin
                            rd_data <= buffer[rd_buf_sel];
                            rd_keep <= last_keep;
                            rd_last <= last_buffer[rd_buf_sel];
                            clear_valid_req[rd_buf_sel] <= 1'b1;
                            rd_buf_sel <= (rd_buf_sel == 2'd2) ? 2'd0 : rd_buf_sel + 1'b1;

                        end

                end
            endcase

            // 计数器控制
            if (rd_cnt == 3'd7) begin
                rd_cnt <= 3'd0;
            end else begin
                rd_cnt <= rd_cnt + 1'b1;
            end
        end else begin
            gmii_tx_en <= 1'b0;
            gmii_txd <= 8'd0;
        end
    end
end

endmodule