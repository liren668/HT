//----------------------------------------------------------------------------------------
// File name:           gmii_to_axi.v
// Created by:          ɺ����˹��
// Created date:        2025.3
// Version:             V0.1
// Descriptions:        gmii_to_axi��������洢���ṹ��
//2025.4.12 �޸ģ�ĩβ64�ֽڶ���ʱ�����㡣last_tkeep���ڱ������һ��tkeepֵ������axis_tdata���һ��64bit���ݺ��档
//----------------------------------------------------------------------------------------

module gmii_to_axi(
    // input             dclk,           // 100MHzʱ��
    input             gmii_rx_clk,    // 125MHzдʱ��
    input             tx_clk_out,     // 156.25MHz��ʱ��
    input             rst_n,
    input             gmii_rx_dv,     // GMII������Ч
    input  [7:0]      gmii_rxd,       // GMII��������
    output reg        axis_tvalid,    // AXI��Ч�ź�
    output reg [63:0] axis_tdata,
    output reg        axis_tlast,     // ��������־
    output reg [7:0]  axis_tkeep,     // �ֽ���Ч����
    input             axis_tready     // ���ξ����ź�
);

// ==================== дʱ�����߼���125MHz��====================
reg [63:0] buffer [0:1];        // ˫����洢��
reg        wr_buf_sel;          // д����ѡ��0/1��
reg [2:0]  wr_cnt;              // �ֽڼ�������0-7��
reg [2:0]  valid_bytes [0:1];   // ��������Ч�ֽ���
reg        pkt_end [0:1];       // ��������־
reg        gmii_rx_dv0;
reg        gmii_rx_dv1;

// д�����߼�
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
            // �������
            buffer[wr_buf_sel][wr_cnt*8 +:8] <= gmii_rxd;    //��һ���ֽ� (wr_cnt=0): ��䵽 [7:0]
                                                             //�ڶ����ֽ� (wr_cnt=1): ��䵽 [15:8]
                                                             //�������ֽ� (wr_cnt=2): ��䵽 [23:16] ...�Դ�����                      
            if (wr_cnt == 3'd7) begin                // ������д��
                valid_bytes[wr_buf_sel] <= 3'd0;     // �޸����0��ʾ8���ֽڶ���Ч
                wr_cnt <= 0;
                wr_buf_sel <= ~wr_buf_sel;          // �л�����������ֹ���ݶ�ʧ
            end else begin
                wr_cnt <= wr_cnt + 1;
            end
        end 
        else if (!gmii_rx_dv && gmii_rx_dv0) begin      // ����������
            if(wr_cnt != 3'd0) begin                    // 8�ֽڲ��������
                valid_bytes[wr_buf_sel] <= wr_cnt;
                pkt_end[wr_buf_sel] <= 1'b1;
                wr_cnt <= 0;
                wr_buf_sel <= ~wr_buf_sel;
            end else begin                              // ����8�ֽڶ������
                // valid_bytes[wr_buf_sel] <= 3'd0;
                pkt_end[~wr_buf_sel] <= 1'b1;
            end    
        end
    end
end

// ==================== ��ʱ�����߼���156.25MHz��====================
reg        rd_buf_sel;          // ������ѡ��0/1��
reg        last_delay;  // ������ʱtlast�ź�
reg        need_padding;        // �Ƿ���Ҫ����padding����
reg [7:0]  last_tkeep;         // �������һ��tkeepֵ
(* ASYNC_REG = "TRUE" *) reg [1:0] sync_wr_buf_sel; // дָ��ͬ����
reg        need_last;           // ָʾ�Ƿ���Ҫ��������tlast

// �ڸ�λ����������źų�ʼ��
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
                
                // ���ݲ�����߼�
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
                // ���Ͷ����64bit���ݣ�����tkeep��Ϣ
                axis_tvalid <= 1'b1;
                axis_tdata <= {56'd0, last_tkeep};
                axis_tkeep <= 8'hFF;
                axis_tlast <= 1'b0;      // ���Ͷ�������ʱ����tlastΪ0
                need_padding <= 1'b0;
                need_last <= 1'b1;       // �����һ����Ҫ����tlast
            end else if (need_last) begin
                // ��������tlast�ź�
                axis_tvalid <= 1'b1;
                axis_tdata <= 64'd0;     // ����ȫ0
                axis_tkeep <= 8'hFF;
                axis_tlast <= 1'b1;      // ���һ����1
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

// tkeep���ɺ���
function [7:0] calc_tkeep(input [2:0] bytes);
    case(bytes)
        3'd0: calc_tkeep = 8'hFF;    // 8����Ч�ֽ�
        3'd1: calc_tkeep = 8'h01;    // 1����Ч�ֽ�
        3'd2: calc_tkeep = 8'h03;    // 2����Ч�ֽ�
        3'd3: calc_tkeep = 8'h07;    // 3����Ч�ֽ�
        3'd4: calc_tkeep = 8'h0F;    // 4����Ч�ֽ�
        3'd5: calc_tkeep = 8'h1F;    // 5����Ч�ֽ�
        3'd6: calc_tkeep = 8'h3F;    // 6����Ч�ֽ�
        3'd7: calc_tkeep = 8'h7F;    // 7����Ч�ֽ�
        default: calc_tkeep = 8'hFF;  // ��ֹ�쳣���
    endcase
endfunction

// ==================== ��ʱ����ͬ�� ====================
// дָ��ͬ������ʱ����
always @(posedge tx_clk_out) begin
    sync_wr_buf_sel[0] <= wr_buf_sel;
    sync_wr_buf_sel[1] <= sync_wr_buf_sel[0];
    
end
endmodule