//----------------------------------------------------------------------------------------
// File name:           axi_to_gmii
// Created by:          ɺ����˹��
// Created date:        2025.3
// Version:             V0.1
// Descriptions:        axi to gmii��ʹ��FIFOʵ�����ݻ����ʱ����ת��
//2025.4 �ж�FIFO�ֽ�������ֽ�������16�ֽھͿ�ʼ����gmii_txd��ֱ���������ݷ�����ϡ���֤gmii_txd�������ġ�
//----------------------------------------------------------------------------------------

module axi_to_gmii(
    input                rst_n,          // ��λ�źţ��͵�ƽ��Ч
    
    // AXI Stream �ӿ� (156.25MHz)
    input                tx_clk_out,     // AXIʱ����дʱ��
    input                axis_tvalid,    // AXI������Ч�ź�
    input        [63:0]  axis_tdata,     // AXI����
    input                axis_tlast,     // ��������־
    input        [7:0]   axis_tkeep,     // �ֽ���Ч����
    
    // GMII �ӿ� (125MHz)
    input                gmii_tx_clk,    // GMIIʱ�����ʱ��
    output reg           gmii_tx_en,     // GMII����������Ч
    output reg  [7:0]    gmii_txd        // GMII��������
);

// ��������
parameter START_THRESHOLD = 16;        // ��ʼ������ֵ���ֽڣ�

// FIFO����ź�����
reg  [7:0]  fifo_din;          // FIFO��������
reg         fifo_wr_en;        // FIFOдʹ��
wire [7:0]  fifo_dout;         // FIFO�������
wire        fifo_empty;        // FIFO�ձ�־
wire        fifo_almost_empty; // FIFO�����ձ�־
wire [10:0] wr_data_count;     // д�����ݼ���
reg         fifo_rd_en;        // FIFO��ʹ��

// ��ʱ����ͬ��wr_data_count
(* ASYNC_REG = "TRUE" *) reg [7:0] sync_wr_count_1;  // ֻ��Ҫ8λ����Ϊ16�ֽ�ֻ��Ҫ8λ��ʾ
(* ASYNC_REG = "TRUE" *) reg [7:0] sync_wr_count_2;  // ֻ��Ҫ8λ����Ϊ16�ֽ�ֻ��Ҫ8λ��ʾ

//FIFO 8bit 2048��ȣ�ռ18K
fifo_generator_0 fifo_inst (
    .rst(!rst_n),                    // ��λ�źţ��ߵ�ƽ��Ч
    .wr_clk(tx_clk_out),            // дʱ��
    .rd_clk(gmii_tx_clk),           // ��ʱ��
    .din(fifo_din),                 // д����
    .wr_en(fifo_wr_en),             // дʹ��
    .rd_en(fifo_rd_en),             // ��ʹ��
    .dout(fifo_dout),               // ������
    .full(),                        // δʹ��
    .almost_full(),                 // δʹ��
    .empty(fifo_empty),             // �ձ�־
    .almost_empty(fifo_almost_empty),// �����ձ�־
    .wr_data_count(wr_data_count),  // д�����ݼ���
    .wr_rst_busy(),                 // δʹ��
    .rd_rst_busy()                  // δʹ��
);

// ==================== дʱ����(156.25MHz) ====================
reg [2:0]  byte_cnt;          // �ֽڼ�����(0-7)
reg [63:0] data_reg;          // ���ݼĴ���
reg [7:0]  keep_reg;          // keep�Ĵ���
reg        valid_reg;         // valid�Ĵ���

// д�����߼�
always @(posedge tx_clk_out or negedge rst_n) begin
    if (!rst_n) begin
        byte_cnt <= 3'd0;
        fifo_wr_en <= 1'b0;
        fifo_din <= 8'd0;
        data_reg <= 64'd0;
        keep_reg <= 8'h0;
        valid_reg <= 1'b0;
    end else begin
        // Ĭ��ֵ
        fifo_wr_en <= 1'b0;
        
        if (axis_tvalid) begin
            // �����ݵ�����¼Ĵ���
            data_reg <= axis_tdata;
            keep_reg <= axis_tkeep;
            valid_reg <= 1'b1;
            byte_cnt <= 3'd0;
        end else if (valid_reg) begin
            // �����ѻ�������ݣ�������8bit�ڵ���Ϣ
            case (byte_cnt)
                3'd0: begin
                    if (keep_reg[1]) begin  // ��[15:8]��ʼ������[7:0]
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
                    valid_reg <= 1'b0;  // �����굱ǰ����
                end
            endcase
        end
    end
end

// ��ʱ����ͬ��wr_data_count
always @(posedge gmii_tx_clk or negedge rst_n) begin
    if (!rst_n) begin
        sync_wr_count_1 <= 8'd0;
        sync_wr_count_2 <= 8'd0;
    end else begin
        sync_wr_count_1 <= wr_data_count[7:0];  // ֻȡ��8λ
        sync_wr_count_2 <= sync_wr_count_1;
    end
end

// ==================== ��ʱ����(125MHz) ====================
reg        start_send;         // ��ʼ���ͱ�־
reg        start_send1;
reg        rd_en_cnt0;          
reg [1:0]  rd_en_cnt1;  
// �������߼�
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
        
        // ��ʼ����������FIFO������������ֵ��δ��ʼ����
        if (sync_wr_count_2 >= START_THRESHOLD) begin
            start_send <= 1'b1;
            fifo_rd_en <= 1'b1; 
        end
        
        // ���Ϳ����߼�
        if (start_send) begin
            rd_en_cnt0 <=  1'b1;
            if (rd_en_cnt0 == 1'b1 ) begin
                start_send1 <= 1'b1;             //��ʱ2������
            end
            if (start_send1) begin
                gmii_tx_en <= 1'b1;             //��ʱ2������
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