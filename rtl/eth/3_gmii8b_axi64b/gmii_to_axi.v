//----------------------------------------------------------------------------------------
// File name:           gmii_to_axi.v
// Created by:          ɺ����˹��
// Created date:        2025.3
// Version:             V0.1
// Descriptions:        gmii_to_axi��FIFO�ṹ��
//2025.4.12 �޸ģ�ĩβ64�ֽڶ���ʱ�����㡣
//����ƴ�ӳ�64b���ݡ�
//----------------------------------------------------------------------------------------

module gmii_to_axi #(
    parameter NODE_ID = 4'b0001,  // �ڵ�ID
    parameter ETH_TYPE = 4'b0001  // ��̫����������
)(
    input             gmii_rx_clk,    // 125MHzдʱ��
    input             tx_clk_out,     // 156.25MHz��ʱ��
    input             rst_n,
    input             gmii_rx_dv,     // GMII������Ч
    input  [7:0]      gmii_rxd,       // GMII��������
    output reg        axis_tvalid,    // AXI��Ч�ź�
    output reg [63:0] axis_tdata,     // AXI����
    output reg        axis_tlast,     // ��������־
    output reg [7:0]  axis_tkeep,     // �ֽ���Ч����
    input             axis_tready     // ���ξ����ź�
);

// ==================== ״̬������ =====================
localparam IDLE = 3'd0;      // ����״̬
localparam COLLECT = 3'd1;   // �ռ�����״̬
localparam SEND = 3'd2;      // ��������״̬
localparam PADDING = 3'd3;   // ����״̬
localparam LAST = 3'd4;      // �������һ������״̬

// ==================== �ڲ��źŶ��� =====================
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

// FIFO��λ�ź�
assign fifo_rst = ~rst_n;

// FIFO IP��ʵ����
fifo_generator_0 u_fifo (
    .rst(fifo_rst),                      // ��λ�ź�
    .wr_clk(gmii_rx_clk),               // дʱ��
    .rd_clk(tx_clk_out),                // ��ʱ��
    .din(fifo_din),                     // д����
    .wr_en(fifo_wr_en),                 // дʹ��
    .rd_en(fifo_rd_en),                 // ��ʹ��
    .dout(fifo_dout),                   // ������
    .full(fifo_full),                   // FIFO����־
    .almost_full(),                     // ��������־��δʹ�ã�
    .empty(fifo_empty),                 // FIFO�ձ�־
    .almost_empty(),                    // �����ձ�־��δʹ�ã�
    .rd_data_count(fifo_rd_count),      // �����ݼ���
    .wr_data_count(fifo_wr_count),      // д���ݼ���
    .wr_rst_busy(fifo_wr_rst_busy),     // д��λæ��־
    .rd_rst_busy(fifo_rd_rst_busy)      // ����λæ��־
);

// ==================== дʱ�����߼���125MHz��====================
reg [2:0]  wr_cnt;               // �ֽڼ�������0-7��
reg [63:0] temp_data;            // ��ʱ���ݼĴ���
reg        pkt_end;              // ��������־

// д�����߼�
always @(posedge gmii_rx_clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_cnt <= 3'd0;
        temp_data <= 64'd0;
        pkt_end <= 1'b0;
        fifo_wr_en <= 1'b0;
        fifo_din <= 8'd0;
    end else begin
        if (gmii_rx_dv) begin
            // ������䵽��ʱ�Ĵ���
            temp_data[wr_cnt*8 +:8] <= gmii_rxd;
            fifo_wr_en <= 1'b0;
            
            if (wr_cnt == 3'd7) begin
                // 8�ֽ������ռ���ɣ�д��FIFO
                if (!fifo_full) begin
                    fifo_din <= {temp_data[63:8], {NODE_ID, ETH_TYPE}};
                    fifo_wr_en <= 1'b1;
                end
                wr_cnt <= 3'd0;
            end else begin
                wr_cnt <= wr_cnt + 1'b1;
            end
        end else if (!gmii_rx_dv && wr_cnt != 3'd0) begin
            // ��������������
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

// ==================== ��ʱ�����߼���156.25MHz��====================
reg [2:0] state;                // ״̬��״̬
reg [2:0] next_state;           // ��һ״̬
reg [63:0] send_data;           // �������ݼĴ���
reg        send_valid;          // ������Ч��־
reg        send_last;           // �������һ�����ݱ�־

// ״̬��ת���߼�
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