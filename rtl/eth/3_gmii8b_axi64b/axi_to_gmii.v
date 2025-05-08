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
    input                clk_ila,        // ILAʱ��
    
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

ila_0 u_ila_0 (
    .clk(clk_ila),              // input wire clk
    .probe0(axis_tvalid),   // input wire [63:0]  probe0  
    .probe1(axis_tdata),  // input wire [7:0]  probe1 
    .probe2(axis_tkeep),   // input wire [0:0]  probe2 
    .probe3(tx_clk_out),     // input wire [63:0]  probe3 
    .probe4(gmii_txd),    // input wire [0:0]  probe4 
    .probe5(gmii_tx_en),     // input wire [0:0]  probe5 
    .probe6(gmii_tx_clk),   // input wire [7:0]  probe6 
    .probe7(fifo_din),   // input wire [7:0]  probe6 
    .probe8(fifo_dout),   // input wire [7:0]  probe6 
    .probe9(data_last_reg),   // input wire [7:0]  probe6 
    .probe10(keep_reg_last),   // input wire [7:0]  probe6 
    .probe11(last_valid_reg),   // input wire [7:0]  probe6 
    .probe12(data_delay_cnt)   // input wire [7:0]  probe6 
);

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
reg [2:0]  last_byte_cnt;     // ���һ���ֽڼ�����(0-7)
reg [63:0] data_reg;          // ���ݼĴ������洢�������ݰ�
reg [63:0] data_last_reg;     // ���һ�����ݼĴ������洢���������ݰ�
reg [7:0]  keep_reg;          // keep�Ĵ�����ֻ�ڷ�0xFFʱ����ֵ
reg [7:0]  keep_reg_last;     // keep�Ĵ�����ֻ�ڷ�0xFFʱ����ֵ
reg        valid_reg;         // valid�Ĵ�������ʾ��������Ҫ����
reg        last_valid_reg;    // last_valid�Ĵ�������ʾ���һ�����ݰ�
reg  [4:0]  data_delay_cnt;        // �����ӳټĴ����������ӳ�����

// д�����߼�
always @(posedge tx_clk_out or negedge rst_n) begin
    if (!rst_n) begin
        // ��λ���мĴ���
        byte_cnt <= 3'd0;
        last_byte_cnt <= 3'd0;
        fifo_wr_en <= 1'b0;
        fifo_din <= 8'd0;
        data_reg <= 64'd0;
        data_last_reg <= 64'd0;
        keep_reg <= 8'hFF;    // ��λΪ0xFF
        keep_reg_last <= 8'hFF;
        valid_reg <= 1'b0;
        last_valid_reg <= 1'b0;
        data_delay_cnt <= 5'd0;
    end else begin
        // Ĭ��ֵ
        fifo_wr_en <= 1'b0;
        
        if (axis_tvalid) begin
            // �����ݵ������tkeep���¼Ĵ���
            if (axis_tkeep == 8'hFF) begin
                // �������ݰ�ֱ��д��data_reg
                data_reg <= axis_tdata;
                keep_reg <= 8'hFF;
                valid_reg <= 1'b1;
                byte_cnt <= 3'd0;
            end else if (axis_tkeep != 8'h00 && axis_tkeep != 8'hFF) begin
                // ���������ݰ���д��data_last_reg
                data_last_reg <= axis_tdata;
                keep_reg_last <= axis_tkeep;
                last_valid_reg <= 1'b1;
                data_delay_cnt <= 5'd0;
            end
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
        end else if (last_valid_reg) begin
            // data_delay_cnt�ԼӼ���,�ȴ���һ������д��FIFO
            if (data_delay_cnt <= 5'd5) begin
                data_delay_cnt <= data_delay_cnt + 5'd1;
            end
            // �������һ�����ݰ����ȴ�data_delay_cnt=5�ſ�ʼ����
            if (data_delay_cnt >= 5'd5) begin
                data_reg <= data_last_reg;
                keep_reg <= keep_reg_last;
                valid_reg <= 1'b1;
                byte_cnt <= 3'd0;
                last_valid_reg <= 1'b0;     
            end
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
        
        // ��ʼ����������FIFO������������ֵ��ʼ����
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