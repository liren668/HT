//----------------------------------------------------------------------------------------
// File name:           ht_top.v
// Created by:          ɺ����˹��
// Created date:        2025.4
// Version:             V0.1
// Descriptions:        Hybrid_Transport
//�������ݻ�ϴ����top������������ѯ���ͺͽ��ա�
//----------------------------------------------------------------------------------------

module ht_top(
    // ϵͳ�ӿ�
    input           sys_clk_p,        // ϵͳʱ������100MHz
    input           sys_clk_n,        // ϵͳʱ�Ӹ���  
    input           sys_rst_n,        // ϵͳ��λ,�͵�ƽ��Ч

    // MDIO�ӿ�
    output          eth_mdc,          // MDIOʱ��
    inout           eth_mdio,         // MDIO���� 
    input           key,              // MDIO��λ����
    output   [1:0]  led,              // LED��������ָʾ

    // RGMII�ӿ�
    input           rgmii_rxc,        // ����ʱ��2.5MHz,25MHz,125MHz
    input           rgmii_rx_ctl,     // ���տ���
    input    [3:0]  rgmii_rxd,        // ��������
    output          rgmii_txc,        // ����ʱ��2.5MHz,25MHz,125MHz   
    output          rgmii_tx_ctl,     // ���Ϳ��� 
    output   [3:0]  rgmii_txd,        // ��������

    //HDMI�ӿ�
    // input           hdmi_rx_p,        // HDMI��������
    // input           hdmi_rx_n,        // HDMI���ո���
    // output          hdmi_tx_p,        // HDMI��������
    // output          hdmi_tx_n,        // HDMI���͸��� 

    // SFP�ӿ�
    input           q0_ck1_n_in,      // �ο�ʱ�Ӹ���156.25MHz
    input           q0_ck1_p_in,      // �ο�ʱ������
    input           rxn_in,           // ��ֽ��ո���
    input           rxp_in,           // ��ֽ�������
    output          txn_out,          // ��ַ��͸���
    output          txp_out,          // ��ַ�������
    output  [1:0]   tx_disable     // ����ʹ��    
);

// ��ӱ��ؽڵ���Ϣ����
parameter LOCAL_NODE_INFO = 8'h11;  // ���ؽڵ���Ϣ 00011001

// ����������Ͷ���
parameter DATA_TYPE_ETH = 4'h1;      // ��̫������ 0001
parameter DATA_TYPE_AUX = 4'h8;      // ������������ 1000

//������Ϣ�Ĵ���
reg [8:0] nodes_info_reg [0:15];  //������ڵ���Ϣ��0���ڸ��ڵ㣬1-15���ӽڵ㡣

//reg define
reg  [27:0] dely_500ms ;
reg  [2:0]  S_RESET;            //״̬�ź�
reg  [15:0] cnt_rst;
reg         rst_n;
reg         gtwiz_reset_rx_datapath;


//wire define�շ���
wire        stat_rx_status;
wire        tx_reset;
wire        rx_reset;
wire        gt0_rst;
wire        gtpowergood_out_0;        // GT��Դ״̬�ź�

// ʱ�Ӻ͸�λ
wire          clk_ila;                // ILA����ʱ��62.5MHz
wire          clk_ila1;               // ILAʱ��
wire          dclk;                   // �ڲ�ʱ��100MHz
wire          locked;                 // ʱ������
wire          tx_clk_out;             // ����ʱ��
wire          gt_refclk_out;          // �ο�ʱ��

// AXI���ͽӿڣ����͵�SFP��
reg           axis_to_sfp_tvalid;      // ������Ч�ź�
reg    [63:0] axis_to_sfp_tdata;       // ��������
reg           axis_to_sfp_tlast;       // ���һ�ı�־
reg    [7:0]  axis_to_sfp_tkeep;       // �ֽ���Ч��־
wire          axis_to_sfp_tready;      // ���ն�׼������

// AXI���սӿڣ���SFP���գ�
wire          sfp_to_axis_tvalid;      // ������Ч
wire   [63:0] sfp_to_axis_tdata;       // �������ݴ�SFP 
wire          sfp_to_axis_tlast;       // ���һ��
wire   [7:0]  sfp_to_axis_tkeep;       // �ֽ���Ч

// AXI���ݷ�֧�ӿ�,��AXI�ܽӿ�ѡ���֧��eth,aux,hdmi,usb�ȡ�
wire          eth_to_axis_tvalid;      // ������Ч
wire   [63:0] eth_to_axis_tdata;       // ��̫�����ݷ���axi
wire          eth_to_axis_tlast;       // ���һ��
wire   [7:0]  eth_to_axis_tkeep;       // �ֽ���Ч
reg           axis_to_eth_tvalid;      // ������Ч
reg    [63:0] axis_to_eth_tdata;       // axi���ݽ���eth
reg           axis_to_eth_tlast;       // ���һ��
reg    [7:0]  axis_to_eth_tkeep;       // �ֽ���Ч
//aux���ݷ�֧�ӿ�,������Ϣ������
wire          aux_to_axis_tvalid;      // ������Ч
wire   [63:0] aux_to_axis_tdata;       // �������ݷ���axi
wire          aux_to_axis_tlast;       // ���һ��
wire   [7:0]  aux_to_axis_tkeep;       // �ֽ���Ч
wire          aux_to_axis_tready;      // ������׼������
reg           axis_to_aux_tvalid;      // ������Ч
reg    [63:0] axis_to_aux_tdata;       // axi���ݷ���aux
reg           axis_to_aux_tlast;       // ���һ��
reg    [7:0]  axis_to_aux_tkeep;       // �ֽ���Ч

// ���Ƽ�����
reg    [3:0]  send_type_cnt;           // �������ͼ�����

//*****************************************************
//**                    main code
//*****************************************************

assign  tx_disable = 2'b00; 
assign gt0_rst = ~rst_n;    // ϵͳ��λʱGT��λ���ߵ�ƽ��λ

//ILA���� ʱ�ӻ���
BUFG BUFG_inst (
    .I            (clk_ila),      // 1-bit input: Clock input
    .O            (clk_ila1)  // 1-bit output: Clock output
);

//ILAʵ����
ila_1 u_ila_1 (
    .clk(clk_ila1),              // input wire clk
    .probe0(axis_to_sfp_tdata),   // input wire [63:0]  probe0  
    .probe1(axis_to_sfp_tvalid),  // input wire [7:0]  probe1 
    .probe2(axis_to_sfp_tlast),   // input wire [0:0]  probe2 
    .probe3(axis_to_sfp_tdata),     // input wire [63:0]  probe3 
    .probe4(axis_to_sfp_tvalid),    // input wire [0:0]  probe4 
    .probe5(axis_to_sfp_tlast),     // input wire [0:0]  probe5 
    .probe6(axis_to_sfp_tkeep),     // input wire [7:0]  probe6 
    .probe7(sfp_to_axis_tkeep),   // input wire [7:0]  probe7
    .probe8(sfp_to_axis_tready)   // input wire [0:0]  probe8
);

// ʱ�ӹ���
clk_wiz_0 u_clk_wiz_0(
    .clk_out1      (dclk),                // �ڲ�ʱ��100MHz
    .clk_out2      (clk_ila),             // ILA����ʱ��250MHz
    .reset         (~sys_rst_n),
    .locked        (locked),
    .clk_in1_p     (sys_clk_p),
    .clk_in1_n     (sys_clk_n)
);

// ��λ����
always@(posedge dclk)begin
    if(!locked)
        cnt_rst <= 16'b0;
    else if(!cnt_rst[15])
        cnt_rst <= cnt_rst + 1'b1;
    else
        cnt_rst <= cnt_rst;
end 

//��λ�źţ��͵�ƽ��Ч
always@(posedge dclk)begin
    if(!locked)
        rst_n <= 1'b0;
    else if(cnt_rst > 16'd10000 && cnt_rst <= 16'd20000)
        rst_n <= 1'b0;
    else if(cnt_rst > 16'd20000 && cnt_rst <= 16'd30000)
        rst_n <= 1'b1;
    else
        rst_n <= rst_n;
end

//500ms������
always @(posedge dclk)begin
    if(!rst_n)begin
        S_RESET <= 1'b0;
        gtwiz_reset_rx_datapath <= 1'b0;
    end
    else begin
        case(S_RESET)
        0 : begin
            if(dely_500ms == 800_000_00)begin
                dely_500ms  <= 0;
                S_RESET <= 1;
            end
            else
                dely_500ms <= dely_500ms + 1'b1;
        end
        1 : begin
            gtwiz_reset_rx_datapath <= 1'b1;
            if(!(stat_rx_status))
                S_RESET <= 2;
            else 
                S_RESET <= 3;
        end
        2 : begin
            gtwiz_reset_rx_datapath <= 1'b0;
            S_RESET <= 0;          
        end
        3 : begin
            gtwiz_reset_rx_datapath <= 1'b0;
            S_RESET <= 4;            
        end
        4 : begin
            if(dely_500ms == 800_000_00)begin
                dely_500ms <= 0;
                S_RESET <= 5;
            end
            else
                dely_500ms <= dely_500ms + 1'b1;
        end
        5 : begin
            if(!(stat_rx_status))
                S_RESET <= 1;
            else 
                S_RESET <= S_RESET;
        end
        endcase  
    end
end

// ������ѯ�߼�
always @(posedge tx_clk_out or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        axis_to_sfp_tvalid <= 1'b0;
        axis_to_sfp_tdata <= 64'h0;
        axis_to_sfp_tlast <= 1'b0;
        axis_to_sfp_tkeep <= 8'h0;
        send_type_cnt <= 4'd0;
    end else begin
        // ����LOCAL_NODE_INFO��4λ��1��λ����ѯ����
        case (send_type_cnt)
            4'd0: begin  // ��0�����ݣ�eth����
                if (LOCAL_NODE_INFO[0]) begin
                    if (eth_to_axis_tvalid) begin  // ��Ч����ʱ������������
                        axis_to_sfp_tvalid <= 1'b1;
                        axis_to_sfp_tdata <= eth_to_axis_tdata;
                        axis_to_sfp_tlast <= eth_to_axis_tlast;
                        axis_to_sfp_tkeep <= eth_to_axis_tkeep;
                    end else begin  // ��Ч����ʱ���ͽڵ���Ϣ
                        axis_to_sfp_tvalid <= 1'b1;
                        axis_to_sfp_tdata <= {56'h0, LOCAL_NODE_INFO[7:4], 4'h0};
                        axis_to_sfp_tlast <= eth_to_axis_tlast;
                        axis_to_sfp_tkeep <= 8'h00;
                    end
                end else begin
                    axis_to_sfp_tvalid <= 1'b0;
                end
                // ����һ����1λ
                send_type_cnt <= (LOCAL_NODE_INFO[1]) ? 4'd1 :
                               (LOCAL_NODE_INFO[2]) ? 4'd2 :
                               (LOCAL_NODE_INFO[3]) ? 4'd3 : 4'd0;
            end
            // 4'd1: begin  // ��1������
            //     if (LOCAL_NODE_INFO[1] && data2_valid) begin
            //         axis_to_sfp_tvalid <= 1'b1;
            //         axis_to_sfp_tdata <= {data2_data[63:8], LOCAL_NODE_INFO};
            //         axis_to_sfp_tlast <= data2_last;
            //         axis_to_sfp_tkeep <= data2_keep;
            //     end else begin
            //         axis_to_sfp_tvalid <= 1'b0;
            //     end
            //     // ����һ����1λ
            //     send_type_cnt <= (LOCAL_NODE_INFO[2]) ? 4'd2 :
            //                    (LOCAL_NODE_INFO[3]) ? 4'd3 :
            //                    (LOCAL_NODE_INFO[0]) ? 4'd0 : 4'd1;
            // end
            // 4'd2: begin  // ��2������
            //     if (LOCAL_NODE_INFO[2] && data3_valid) begin
            //         axis_to_sfp_tvalid <= 1'b1;
            //         axis_to_sfp_tdata <= {data3_data[63:8], LOCAL_NODE_INFO};
            //         axis_to_sfp_tlast <= data3_last;
            //         axis_to_sfp_tkeep <= data3_keep;
            //     end else begin
            //         axis_to_sfp_tvalid <= 1'b0;
            //     end
            //     // ����һ����1λ
            //     send_type_cnt <= (LOCAL_NODE_INFO[3]) ? 4'd3 :
            //                    (LOCAL_NODE_INFO[0]) ? 4'd0 :
            //                    (LOCAL_NODE_INFO[1]) ? 4'd1 : 4'd2;
            // end
            // 4'd3: begin  // ��3������(��������)
            //     if (LOCAL_NODE_INFO[3] && aux_to_axis_tvalid) begin
            //         axis_to_sfp_tvalid <= 1'b1;
            //         axis_to_sfp_tdata <= {aux_to_axis_tdata[63:8], LOCAL_NODE_INFO};
            //         axis_to_sfp_tlast <= aux_to_axis_tlast;
            //         axis_to_sfp_tkeep <= aux_to_axis_tkeep;
            //     end else begin
            //         axis_to_sfp_tvalid <= 1'b0;
            //     end
            //     // ����һ����1λ
            //     send_type_cnt <= (LOCAL_NODE_INFO[0]) ? 4'd0 :
            //                    (LOCAL_NODE_INFO[1]) ? 4'd1 :
            //                    (LOCAL_NODE_INFO[2]) ? 4'd2 : 4'd3;
            // end
            default: send_type_cnt <= 4'd0;
                        
        endcase
        
        // ��LOCAL_NODE_INFOȫΪ0ʱ�����Ϳ�������
        if (LOCAL_NODE_INFO[3:0] == 4'h0) begin
            axis_to_sfp_tvalid <= 1'b1;
            axis_to_sfp_tdata <= {56'h0, LOCAL_NODE_INFO[7:4],4'h0};
            axis_to_sfp_tlast <= 1'b1;
            axis_to_sfp_tkeep <= 8'h01;
        end
    end
end

// ���ն����ݷַ��߼�
always @(posedge tx_clk_out or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        axis_to_eth_tvalid <= 1'b0;
        axis_to_eth_tdata <= 64'h0;
        axis_to_eth_tlast <= 1'b0;
        axis_to_eth_tkeep <= 8'h0;

        axis_to_aux_tvalid <= 1'b0;
        axis_to_aux_tdata <= 64'h0;
        axis_to_aux_tlast <= 1'b0;
        axis_to_aux_tkeep <= 8'h0;
    end else begin
        if (sfp_to_axis_tvalid) begin
            case (sfp_to_axis_tdata[7:4])  // �����������
                DATA_TYPE_ETH: begin  // ��̫������
                    axis_to_eth_tvalid <= 1'b1;
                    axis_to_eth_tdata <= sfp_to_axis_tdata;
                    axis_to_eth_tlast <= sfp_to_axis_tlast;
                    axis_to_eth_tkeep <= sfp_to_axis_tkeep;
                end
                DATA_TYPE_AUX: begin  // ��������
                    axis_to_aux_tvalid <= 1'b1;
                    axis_to_aux_tdata <= sfp_to_axis_tdata;
                    axis_to_aux_tlast <= sfp_to_axis_tlast;
                    axis_to_aux_tkeep <= sfp_to_axis_tkeep;
                end
                default: begin
                    axis_to_eth_tvalid <= 1'b0;
                    axis_to_aux_tvalid <= 1'b0;
                end
            endcase
        end else begin
            axis_to_eth_tvalid <= 1'b0;
            axis_to_aux_tvalid <= 1'b0;
        end
    end
end

// ʵ������̫������ģ��
eth_top u_eth_top(
    // ϵͳ�ӿ�
    .dclk          (dclk),           // �ڲ�ʱ��100MHz
    .sys_rst_n     (sys_rst_n),      // ϵͳ��λ,�͵�ƽ��Ч
    .tx_clk_out    (tx_clk_out),     // ����ʱ��

    // MDIO�ӿ�
    .eth_mdc       (eth_mdc),        // MDIOʱ��
    .eth_mdio      (eth_mdio),       // MDIO���� 
    .key           (key),            // MDIO��λ����
    .led           (led),            // LED��������ָʾ

    // RGMII�ӿ�
    .rgmii_rxc     (rgmii_rxc),      // ����ʱ��2.5MHz,25MHz,125MHz
    .rgmii_rx_ctl  (rgmii_rx_ctl),   // ���տ���
    .rgmii_rxd     (rgmii_rxd),      // ��������
    .rgmii_txc     (rgmii_txc),      // ����ʱ��2.5MHz,25MHz,125MHz   
    .rgmii_tx_ctl  (rgmii_tx_ctl),   // ���Ϳ��� 
    .rgmii_txd     (rgmii_txd),      // ��������

    // AXI�ӿ�
    .eth_to_axis_tvalid (eth_to_axis_tvalid),  // ������Ч
    .eth_to_axis_tdata  (eth_to_axis_tdata),   // ����
    .eth_to_axis_tlast  (eth_to_axis_tlast),   // ���һ��
    .eth_to_axis_tkeep  (eth_to_axis_tkeep),   // �ֽ���Ч
    .eth_to_axis_tready (eth_to_axis_tready),  // ׼������
    .axis_to_eth_tvalid (axis_to_eth_tvalid),  // ������Ч
    .axis_to_eth_tdata  (axis_to_eth_tdata),   // ����
    .axis_to_eth_tlast  (axis_to_eth_tlast),   // ���һ��
    .axis_to_eth_tkeep  (axis_to_eth_tkeep)    // �ֽ���Ч
);

// ��̫��������
xxv_ethernet_0 u_xxv_ethernet_0 (
  //��ڲ������
  .gt_rxp_in_0              (rxp_in),    // input wire gt_rxp_in_0
  .gt_rxn_in_0              (rxn_in),    // input wire gt_rxn_in_0
  .gt_txp_out_0             (txp_out),   // output wire gt_txp_out_0
  .gt_txn_out_0             (txn_out),   // output wire gt_txn_out_0

  .tx_clk_out_0             (tx_clk_out),   // output wire tx_clk_out_0
  .tx_reset_0               (gt0_rst),    // input wire tx_reset_0
  .rx_clk_out_0             ( ),    // output wire rx_clk_out_0
  .rx_reset_0               (gt0_rst),    // input wire rx_reset_0
  .rx_core_clk_0            (tx_clk_out),   // input wire rx_core_clk_0
  
  .txoutclksel_in_0         (3'b101),   // input wire [2 : 0] txoutclksel_in_0
  .rxoutclksel_in_0         (3'b101),   // input wire [2 : 0] rxoutclksel_in_0
  
  .gtwiz_reset_tx_datapath_0(1'b0), // input wire gtwiz_reset_tx_datapath_0
  .gtwiz_reset_rx_datapath_0(gtwiz_reset_rx_datapath),  // input wire gtwiz_reset_rx_datapath_0
  
  .rxrecclkout_0            ( ),    // output wire rxrecclkout_0
  
  .sys_reset                (~rst_n),   // input wire sys_reset
  .user_tx_reset_0          (tx_reset), // output wire user_tx_reset_0
 
  .dclk                     (dclk),     // input wire dclk
  .gt_loopback_in_0         (3'b000),   // input wire [2 : 0] gt_loopback_in_0
  .qpllreset_in_0           (1'b0),      // input wire qpllreset_in_0

  //��ڲο�ʱ��
  .gt_refclk_p              (q0_ck1_p_in),  // input wire gt_refclk_p
  .gt_refclk_n              (q0_ck1_n_in),  // input wire gt_refclk_n
  
  .gt_refclk_out            (gt_refclk_out),    // output wire gt_refclk_out
  .gtpowergood_out_0        (gtpowergood_out_0),    // output wire gtpowergood_out_0
  .user_rx_reset_0          (rx_reset),  // output wire user_rx_reset_0
  
  //AXI4 Stream ����ӿ��ź�
  .tx_axis_tready_0        (axis_to_sfp_tready),
  .tx_axis_tvalid_0        (axis_to_sfp_tvalid),
  .tx_axis_tdata_0         (axis_to_sfp_tdata),
  .tx_axis_tlast_0         (axis_to_sfp_tlast), 
  .tx_axis_tkeep_0         (axis_to_sfp_tkeep),
  .tx_axis_tuser_0         (1'b0),
  .tx_preamblein_0         (56'b0),            // input wire [55 : 0] tx_preamblein_0
  
  //RX �����ź�
  .ctl_rx_enable_0                  (1'b1), // input wire ctl_rx_enable_0
  .ctl_rx_check_preamble_0          (1'b0), // input wire ctl_rx_check_preamble_0
  .ctl_rx_check_sfd_0               (1'b0), // input wire ctl_rx_check_sfd_0
  .ctl_rx_force_resync_0            (1'b0), // input wire ctl_rx_force_resync_0
  .ctl_rx_delete_fcs_0              (1'b1), // input wire ctl_rx_delete_fcs_0
  .ctl_rx_ignore_fcs_0              (1'b0), // input wire ctl_rx_ignore_fcs_0
  .ctl_rx_max_packet_len_0          (15'd1518), // input wire [14 : 0] ctl_rx_max_packet_len_0
  .ctl_rx_min_packet_len_0          (15'd8 ),  // input wire [7 : 0] ctl_rx_min_packet_len_0
  .ctl_rx_process_lfi_0             (1'b0), // input wire ctl_rx_process_lfi_0
  .ctl_rx_test_pattern_0            (1'b0), // input wire ctl_rx_test_pattern_0
  .ctl_rx_data_pattern_select_0     (1'b0), // input wire ctl_rx_data_pattern_select_0
  .ctl_rx_test_pattern_enable_0     (1'b0), // input wire ctl_rx_test_pattern_enable_0
  .ctl_rx_custom_preamble_enable_0  (1'b0), // input wire ctl_rx_custom_preamble_enable_0
  
  //RX ����״̬�ź�
  .stat_rx_framing_err_0            ( ),    // output wire stat_rx_framing_err_0
  .stat_rx_framing_err_valid_0      ( ),    // output wire stat_rx_framing_err_valid_0
  .stat_rx_local_fault_0            ( ),    // output wire stat_rx_local_fault_0
  .stat_rx_block_lock_0             (),    // output wire stat_rx_block_lock_0
  .stat_rx_valid_ctrl_code_0        ( ),    // output wire stat_rx_valid_ctrl_code_0
  .stat_rx_status_0                 (stat_rx_status),   // output wire stat_rx_status_0
  .stat_rx_remote_fault_0           ( ),    // output wire stat_rx_remote_fault_0
  .stat_rx_bad_fcs_0                ( ),    // output wire [1 : 0] stat_rx_bad_fcs_0
  .stat_rx_stomped_fcs_0            ( ),    // output wire [1 : 0] stat_rx_stomped_fcs_0
  .stat_rx_truncated_0              ( ),    // output wire stat_rx_truncated_0
  .stat_rx_internal_local_fault_0   ( ),    // output wire stat_rx_internal_local_fault_0
  .stat_rx_received_local_fault_0   ( ),    // output wire stat_rx_received_local_fault_0
  .stat_rx_hi_ber_0                 ( ),    // output wire stat_rx_hi_ber_0
  .stat_rx_got_signal_os_0          ( ),    // output wire stat_rx_got_signal_os_0
  .stat_rx_test_pattern_mismatch_0  ( ),    // output wire stat_rx_test_pattern_mismatch_0
  .stat_rx_total_bytes_0            ( ),    // output wire [3 : 0] stat_rx_total_bytes_0
  .stat_rx_total_packets_0          ( ),    // output wire [1 : 0] stat_rx_total_packets_0
  .stat_rx_total_good_bytes_0       ( ),    // output wire [13 : 0] stat_rx_total_good_bytes_0
  .stat_rx_total_good_packets_0     ( ),    // output wire stat_rx_total_good_packets_0
  .stat_rx_packet_bad_fcs_0         ( ),    // output wire stat_rx_packet_bad_fcs_0
  .stat_rx_packet_64_bytes_0        ( ),    // output wire stat_rx_packet_64_bytes_0
  .stat_rx_packet_65_127_bytes_0    ( ),    // output wire stat_rx_packet_65_127_bytes_0
  .stat_rx_packet_128_255_bytes_0   ( ),    // output wire stat_rx_packet_128_255_bytes_0
  .stat_rx_packet_256_511_bytes_0   ( ),    // output wire stat_rx_packet_256_511_bytes_0
  .stat_rx_packet_512_1023_bytes_0  ( ),    // output wire stat_rx_packet_512_1023_bytes_0
  .stat_rx_packet_1024_1518_bytes_0 ( ),    // output wire stat_rx_packet_1024_1518_bytes_0
  .stat_rx_packet_1519_1522_bytes_0 ( ),    // output wire stat_rx_packet_1519_1522_bytes_0
  .stat_rx_packet_1523_1548_bytes_0 ( ),    // output wire stat_rx_packet_1523_1548_bytes_0
  .stat_rx_packet_1549_2047_bytes_0 ( ),    // output wire stat_rx_packet_1549_2047_bytes_0
  .stat_rx_packet_2048_4095_bytes_0 ( ),    // output wire stat_rx_packet_2048_4095_bytes_0
  .stat_rx_packet_4096_8191_bytes_0 ( ),    // output wire stat_rx_packet_4096_8191_bytes_0
  .stat_rx_packet_8192_9215_bytes_0 ( ),    // output wire stat_rx_packet_8192_9215_bytes_0
  .stat_rx_packet_small_0           ( ),    // output wire stat_rx_packet_small_0
  .stat_rx_packet_large_0           ( ),    // output wire stat_rx_packet_large_0
  .stat_rx_unicast_0                ( ),    // output wire stat_rx_unicast_0
  .stat_rx_multicast_0              ( ),    // output wire stat_rx_multicast_0
  .stat_rx_broadcast_0              ( ),    // output wire stat_rx_broadcast_0
  .stat_rx_oversize_0               ( ),    // output wire stat_rx_oversize_0
  .stat_rx_toolong_0                ( ),    // output wire stat_rx_toolong_0
  .stat_rx_undersize_0              ( ),    // output wire stat_rx_undersize_0
  .stat_rx_fragment_0               ( ),    // output wire stat_rx_fragment_0
  .stat_rx_vlan_0                   (  ),   // output wire stat_rx_vlan_0
  .stat_rx_inrangeerr_0             ( ),    // output wire stat_rx_inrangeerr_0
  .stat_rx_jabber_0                 ( ),    // output wire stat_rx_jabber_0
  .stat_rx_bad_code_0               ( ),    // output wire stat_rx_bad_code_0
  .stat_rx_bad_sfd_0                ( ),    // output wire stat_rx_bad_sfd_0
  .stat_rx_bad_preamble_0           ( ),    // output wire stat_rx_bad_preamble_0
  
  //AXI4 Stream ���սӿ��ź�
  .rx_axis_tvalid_0        (sfp_to_axis_tvalid),
  .rx_axis_tdata_0         (sfp_to_axis_tdata), 
  .rx_axis_tlast_0         (sfp_to_axis_tlast),
  .rx_axis_tkeep_0         (sfp_to_axis_tkeep),
  .rx_axis_tuser_0         (1'b0),
  
  .tx_unfout_0                    ( ),                // output wire tx_unfout_0

  //TX ״̬�ź�
  .stat_tx_local_fault_0            ( ),        // output wire stat_tx_local_fault_0
  .stat_tx_total_bytes_0            ( ),        // output wire [3 : 0] stat_tx_total_bytes_0
  .stat_tx_total_packets_0          ( ),        // output wire stat_tx_total_packets_0
  .stat_tx_total_good_bytes_0       ( ),        // output wire [13 : 0] stat_tx_total_good_bytes_0
  .stat_tx_total_good_packets_0     ( ),        // output wire stat_tx_total_good_packets_0
  .stat_tx_bad_fcs_0                ( ),        // output wire stat_tx_bad_fcs_0
  .stat_tx_packet_64_bytes_0        ( ),        // output wire stat_tx_packet_64_bytes_0
  .stat_tx_packet_65_127_bytes_0    ( ),        // output wire stat_tx_packet_65_127_bytes_0
  .stat_tx_packet_128_255_bytes_0   ( ),        // output wire stat_tx_packet_128_255_bytes_0
  .stat_tx_packet_256_511_bytes_0   ( ),        // output wire stat_tx_packet_256_511_bytes_0
  .stat_tx_packet_512_1023_bytes_0  ( ),        // output wire stat_tx_packet_512_1023_bytes_0
  .stat_tx_packet_1024_1518_bytes_0 ( ),        // output wire stat_tx_packet_1024_1518_bytes_0
  .stat_tx_packet_1519_1522_bytes_0 ( ),        // output wire stat_tx_packet_1519_1522_bytes_0
  .stat_tx_packet_1523_1548_bytes_0 ( ),        // output wire stat_tx_packet_1523_1548_bytes_0
  .stat_tx_packet_1549_2047_bytes_0 ( ),        // output wire stat_tx_packet_1549_2047_bytes_0
  .stat_tx_packet_2048_4095_bytes_0 ( ),        // output wire stat_tx_packet_2048_4095_bytes_0
  .stat_tx_packet_4096_8191_bytes_0 ( ),        // output wire stat_tx_packet_4096_8191_bytes_0
  .stat_tx_packet_8192_9215_bytes_0 ( ),        // output wire stat_tx_packet_8192_9215_bytes_0
  .stat_tx_packet_small_0           ( ),        // output wire stat_tx_packet_small_0
  .stat_tx_packet_large_0           ( ),        // output wire stat_tx_packet_large_0
  .stat_tx_unicast_0                ( ),        // output wire stat_tx_unicast_0
  .stat_tx_multicast_0              ( ),        // output wire stat_tx_multicast_0
  .stat_tx_broadcast_0              ( ),        // output wire stat_tx_broadcast_0
  .stat_tx_vlan_0                   ( ),        // output wire stat_tx_vlan_0
  .stat_tx_frame_error_0            ( ),        // output wire stat_tx_frame_error_0
  
  //AXI4?Stream �ӿ� - TX ·�������źź�״̬�ź�
  .ctl_tx_enable_0                  (1'b1),     // input wire ctl_tx_enable_0
  .ctl_tx_send_rfi_0                (1'b0),     // input wire ctl_tx_send_rfi_0
  .ctl_tx_send_lfi_0                (1'b0),     // input wire ctl_tx_send_lfi_0
  .ctl_tx_send_idle_0               (1'b0),     // input wire ctl_tx_send_idle_0
  .ctl_tx_fcs_ins_enable_0          (1'b0),     // input wire ctl_tx_fcs_ins_enable_0
  .ctl_tx_ignore_fcs_0              (1'b0),     // input wire ctl_tx_ignore_fcs_0
  
  .ctl_tx_test_pattern_0            (1'b0),     // input wire ctl_tx_test_pattern_0
  .ctl_tx_test_pattern_enable_0     (1'b0),     // input wire ctl_tx_test_pattern_enable_0
  .ctl_tx_test_pattern_select_0     (1'b0),     // input wire ctl_tx_test_pattern_select_0
  .ctl_tx_data_pattern_select_0     (1'b0),     // input wire ctl_tx_data_pattern_select_0
  .ctl_tx_test_pattern_seed_a_0     (1'b0),     // input wire [57 : 0] ctl_tx_test_pattern_seed_a_0
  .ctl_tx_test_pattern_seed_b_0     (1'b0),     // input wire [57 : 0] ctl_tx_test_pattern_seed_b_0
  .ctl_tx_ipg_value_0               (4'd8),    // input wire [3 : 0] ctl_tx_ipg_value_0
  
  .ctl_tx_custom_preamble_enable_0  (1'b1)     // input wire ctl_tx_custom_preamble_enable_0
);
endmodule