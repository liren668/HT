//----------------------------------------------------------------------------------------
// File name:           eth_top.v
// Created by:          ɺ����˹��
// Created date:        2025.4
// Version:             V0.1
// Descriptions:        eth_top
//��̫��top��
//----------------------------------------------------------------------------------------

module eth_top(
    // ϵͳ�ӿ�
    input           dclk,           // �ڲ�ʱ��100MHz
    input           sys_rst_n,      // ϵͳ��λ,�͵�ƽ��Ч

    // MDIO�ӿ�
    output          eth_mdc,        // MDIOʱ��
    inout           eth_mdio,       // MDIO���� 
    input           key,            // MDIO��λ����
    output   [1:0]  led,            // LED��������ָʾ

    // RGMII�ӿ�
    input           rgmii_rxc,      // ����ʱ��2.5MHz,25MHz,125MHz
    input           rgmii_rx_ctl,   // ���տ���
    input    [3:0]  rgmii_rxd,      // ��������
    output          rgmii_txc,      // ����ʱ��2.5MHz,25MHz,125MHz   
    output          rgmii_tx_ctl,   // ���Ϳ��� 
    output   [3:0]  rgmii_txd,      // ��������

    // ʱ�ӽӿ�
    input           tx_clk_out,     // ����ʱ��

    // AXI�ӿ�
    output          eth_to_axis_tvalid,  // ������Ч
    output   [63:0] eth_to_axis_tdata,   // ����
    output          eth_to_axis_tlast,   // ���һ��
    output   [7:0]  eth_to_axis_tkeep,   // �ֽ���Ч
    input           eth_to_axis_tready,  // ׼������
    input           axis_to_eth_tvalid,    // ������Ч
    input    [63:0] axis_to_eth_tdata,     // ����
    input           axis_to_eth_tlast,     // ���һ��
    input    [7:0]  axis_to_eth_tkeep      // �ֽ���Ч
);

//wire define
wire   [1:0]  speed_mode;             // ����ģʽ

// GMII�ӿ� 
wire          gmii_rx_clk;            // ����ʱ��2.5MHz,25MHz,125MHz
wire          gmii_rx_dv;             // ������Ч
wire   [7:0]  gmii_rxd;               // ��������
wire          gmii_tx_clk;            // ����ʱ��2.5MHz,25MHz,125MHz  
wire          gmii_tx_en;             // ������Ч
wire   [7:0]  gmii_txd;              // ��������

//*****************************************************
//**                    main code
//*****************************************************

// ʵ����MDIO����ģ��
mdio_top u_mdio_top(
    .dclk          (dclk),         // ʹ��clk_wiz_0�����ʱ��
    .sys_rst_n     (sys_rst_n),
    .eth_mdc       (eth_mdc),
    .eth_mdio      (eth_mdio),
    .key           (key),
    .led           (led),
    .speed_mode    (speed_mode)
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

// GMII-AXIת��
gmii8b_axi64b_top u_gmii8b_axi64b_top(
    // ϵͳ�ӿ�
    .sys_rst_n     (sys_rst_n),
    .tx_clk_out    (tx_clk_out),
    
    // GMII�ӿ�
    .gmii_rx_clk   (gmii_rx_clk),
    .gmii_rx_dv    (gmii_rx_dv),
    .gmii_rxd      (gmii_rxd),
    .gmii_tx_clk   (gmii_tx_clk),
    .gmii_tx_en    (gmii_tx_en),
    .gmii_txd      (gmii_txd),
    
    // AXI�ӿ�
    .rgmii_axis_tvalid (rgmii_axis_tvalid),
    .rgmii_axis_tdata  (rgmii_axis_tdata),
    .rgmii_axis_tlast  (rgmii_axis_tlast),
    .rgmii_axis_tkeep  (rgmii_axis_tkeep),
    .rgmii_axis_tready (rgmii_axis_tready),
    .sfp_axis_tvalid   (sfp_axis_tvalid),
    .sfp_axis_tdata    (sfp_axis_tdata),
    .sfp_axis_tlast    (sfp_axis_tlast),
    .sfp_axis_tkeep    (sfp_axis_tkeep)
);

endmodule