//----------------------------------------------------------------------------------------
// File name:           rgmii_sfp_exchange
// Created by:          ɺ����˹��
// Created date:        2025.3
// Version:             V0.1
// Descriptions:        rgmii_sfp_exchange
//rgmii�ӿڵ�SFP�ӿڵ�ת������
//----------------------------------------------------------------------------------------

module rgmii_sfp_exchange(
    // ϵͳ�ӿ�
    input           sys_clk_p,        // ϵͳʱ������100MHz
    input           sys_clk_n,        // ϵͳʱ�Ӹ���  
    input           sys_rst_n,        // ϵͳ��λ,�͵�ƽ��Ч

    // MDIO�ӿ�
    output          eth_mdc,          // MDIOʱ��
    inout           eth_mdio,         // MDIO���� 
    input           key,              // MDIO��λ����
    output   [1:0]  led,              // LED��������ָʾ
    // output          eth_rst_n,        // PHY��λ,�͵�ƽ��Ч

    // RGMII�ӿ�
    input           rgmii_rxc,        // ����ʱ��2.5MHz,25MHz,125MHz
    input           rgmii_rx_ctl,     // ���տ���
    input    [3:0]  rgmii_rxd,        // ��������
    output          rgmii_txc,        // ����ʱ��2.5MHz,25MHz,125MHz   
    output          rgmii_tx_ctl,     // ���Ϳ��� 
    output   [3:0]  rgmii_txd,        // ��������

    // SFP�ӿ�
    input           q0_ck1_n_in,      // �ο�ʱ�Ӹ���156.25MHz
    input           q0_ck1_p_in,      // �ο�ʱ������
    input           rxn_in,           // ��ֽ��ո���
    input           rxp_in,           // ��ֽ�������
    output          txn_out,          // ��ַ��͸���
    output          txp_out,          // ��ַ�������
    output  [1:0]   tx_disable     // ����ʹ��    
);

//reg define
reg  [27:0] dely_500ms ;
reg  [2:0]  S_RESET;            //״̬�ź�

reg  [15:0] cnt_rst;
reg         rst_n;
reg         gtwiz_reset_rx_datapath;

//wire define
wire        stat_rx_status;
wire        tx_reset;

// ʱ�Ӻ͸�λ
wire          clk_ila;                // ILA����ʱ��62.5MHz
wire          clk_ila1;               // ILAʱ��
wire          dclk;                   // �ڲ�ʱ��100MHz
wire          locked;                 // ʱ������
wire          tx_clk_out;             // ����ʱ��
wire          gt_refclk_out;          // �ο�ʱ��
wire   [1:0]  speed_mode;             // ����ģʽ

// GMII�ӿ� 
wire          gmii_rx_clk;            // ����ʱ��2.5MHz,25MHz,125MHz
wire          gmii_rx_dv;             // ������Ч
wire   [7:0]  gmii_rxd;               // ��������
wire          gmii_tx_clk;            // ����ʱ��2.5MHz,25MHz,125MHz  
wire          gmii_tx_en;             // ������Ч
wire   [7:0]  gmii_txd;              // ��������

// RGMII��SFP��AXI�ӿ�
wire          rgmii_axis_tvalid;      // ������Ч
wire   [63:0] rgmii_axis_tdata;       // ����
wire          rgmii_axis_tlast;       // ���һ��
wire   [7:0]  rgmii_axis_tkeep;       // �ֽ���Ч
wire          rgmii_axis_tready;      // 
wire          gt0_rst;                // GT0��λ

// SFP��RGMII��AXI�ӿ�  
wire          sfp_axis_tvalid;        // ������Ч
wire   [63:0] sfp_axis_tdata;         // ����
wire          sfp_axis_tlast;         // ���һ��
wire   [7:0]  sfp_axis_tkeep;         // �ֽ���Ч
// wire          sfp_axis_tready;        // ׼������

// MDIO���ƽӿ�
wire          op_done;                // �������
wire   [15:0] op_rd_data;            // ������
wire          op_rd_ack;             // ��Ӧ��
wire          op_exec;               // ִ��
wire          op_rh_wl;              // ��дѡ��
wire   [4:0]  op_addr;               // �Ĵ�����ַ
wire   [15:0] op_wr_data;            // д����

//*****************************************************
//**                    main code
//*****************************************************

assign  tx_disable = 2'b00; 
assign gt0_rst = ~rst_n;    // ϵͳ��λʱGT��λ

//ILA����
BUFG BUFG_inst (
    .I            (clk_ila),      // 1-bit input: Clock input
    .O            (clk_ila1)  // 1-bit output: Clock output
);

//ILAʵ����
ila_1 u_ila_1 (
    .clk(clk_ila1),              // input wire clk

    .probe0(gmii_txd),         // input wire [63:0]  probe0  
    .probe1(gmii_tx_en),        // input wire [7:0]  probe1 
    .probe2(gmii_tx_clk),        // input wire [0:0]  probe2 

    .probe3(gmii_rxd),        // input wire [0:0]  probe3 
    .probe4(gmii_rx_dv),        // input wire [0:0]  probe4 
    .probe5(sfp_axis_tdata),               // input wire [0:0]  probe5 
    .probe6(sfp_axis_tlast),         // input wire [7:0]  probe6 
    .probe7(sfp_axis_tkeep),        // input wire [0:0]  probe7
    .probe8(sfp_axis_tvalid)      // input wire [0:0]  probe7

    // .probe9(gmii_tx_clk),        // input wire [0:0]  probe7
    // .probe10(gmii_tx_en),        // input wire [0:0]  probe7
    // .probe11(gmii_txd)        // input wire [7:0]  probe7
);

clk_wiz_0 u_clk_wiz_0(
    .clk_out1      (dclk),                // �ڲ�ʱ��100MHz
    .clk_out2      (clk_ila),             // ILA����ʱ��250MHz
    .reset         (~sys_rst_n),
    .locked        (locked),
    .clk_in1_p     (sys_clk_p),
    .clk_in1_n     (sys_clk_n)
);

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

// ʵ����MDIO����ģ��
mdio_top u_mdio_top(
    .dclk          (dclk),         // ʹ��clk_wiz_0�����ʱ��
    .sys_rst_n     (sys_rst_n),
    .eth_mdc       (eth_mdc),
    .eth_mdio      (eth_mdio),
    .key            (key),
    .led            (led),
    .speed_mode     (speed_mode)
);

// GMII-RGMIIת��
gmii_to_rgmii u_gmii_to_rgmii(
    // GMII�ӿ�
    .gmii_rx_clk   (gmii_rx_clk),
    .gmii_rx_dv    (gmii_rx_dv),
    .gmii_rxd      (gmii_rxd),
    .gmii_tx_clk   (gmii_tx_clk),
    .gmii_tx_en    (gmii_tx_en), 
    .gmii_txd      (gmii_txd),
    // RGMII�ӿ�
    .rgmii_rxc     (rgmii_rxc),
    .rgmii_rx_ctl  (rgmii_rx_ctl),
    .rgmii_rxd     (rgmii_rxd),
    .rgmii_txc     (rgmii_txc),
    .rgmii_tx_ctl  (rgmii_tx_ctl),
    .rgmii_txd     (rgmii_txd),
    // ����
    .speed_mode    (speed_mode)
);

// GMIIתAXI 
gmii_to_axi u_gmii_to_axi(
    .gmii_rx_clk    (gmii_rx_clk),
    .tx_clk_out     (tx_clk_out),
    .rst_n          (sys_rst_n),
    .gmii_rx_dv     (gmii_rx_dv),
    .gmii_rxd       (gmii_rxd),
    .axis_tvalid    (rgmii_axis_tvalid),
    .axis_tdata     (rgmii_axis_tdata),
    .axis_tlast     (rgmii_axis_tlast),
    .axis_tkeep     (rgmii_axis_tkeep),
    .axis_tready    (rgmii_axis_tready)
//    .clk_ila        (clk_ila)         // ILAʱ��62.5MHz
);

// AXIתGMII
axi_to_gmii u_axi_to_gmii(    
    .rst_n          (sys_rst_n),
    .tx_clk_out     (tx_clk_out), 
    .gmii_tx_clk    (gmii_tx_clk),
    .axis_tvalid    (sfp_axis_tvalid),
    .axis_tdata     (sfp_axis_tdata),
    .axis_tlast     (sfp_axis_tlast),
    .axis_tkeep     (sfp_axis_tkeep),
    // .axis_tready    (sfp_axis_tready),
    .gmii_tx_en     (gmii_tx_en),
    .gmii_txd       (gmii_txd)
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
    .tx_axis_tready_0        (rgmii_axis_tready),
    .tx_axis_tvalid_0        (rgmii_axis_tvalid),
    .tx_axis_tdata_0         (rgmii_axis_tdata),
    .tx_axis_tlast_0         (rgmii_axis_tlast), 
    .tx_axis_tkeep_0         (rgmii_axis_tkeep),
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
    .rx_axis_tvalid_0        (sfp_axis_tvalid),
    .rx_axis_tdata_0         (sfp_axis_tdata), 
    .rx_axis_tlast_0         (sfp_axis_tlast),
    .rx_axis_tkeep_0         (sfp_axis_tkeep),
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