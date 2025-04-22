//----------------------------------------------------------------------------------------
// File name:           gmii8b_axi64b_top.v
// Created by:          ɺ����˹��
// Created date:        2025.4
// Version:             V0.1
// Descriptions:        gmii8b_axi64b_exchange
//gmii8b�ӿڵ�axi64b�ӿڵĻ�����ʵ�����ݵ�ƴ�ӺͲ�֡�
//----------------------------------------------------------------------------------------

module gmii8b_axi64b_top(
    // ϵͳ�ӿ�
    input           sys_rst_n,      // ϵͳ��λ,�͵�ƽ��Ч
    input           tx_clk_out,     // ����ʱ��

    // GMII�ӿ� 
    input           gmii_rx_clk,    // ����ʱ��2.5MHz,25MHz,125MHz
    input           gmii_rx_dv,     // ������Ч
    input    [7:0]  gmii_rxd,       // ��������
    input           gmii_tx_clk,    // ����ʱ��2.5MHz,25MHz,125MHz  
    input           gmii_tx_en,     // ������Ч
    input    [7:0]  gmii_txd,       // ��������

    // AXI�ӿ�
    output          eth_to_axis_tvalid,  // ������Ч
    output   [63:0] eth_to_axis_tdata,   // ����
    output          eth_to_axis_tlast,   // ���һ��
    output   [7:0]  eth_to_axis_tkeep,   // �ֽ���Ч
    input           eth_to_axis_tready,  // ׼������
    input           axis_to_eth_tvalid,  // ������Ч
    input    [63:0] axis_to_eth_tdata,   // ����
    input           axis_to_eth_tlast,   // ���һ��
    input    [7:0]  axis_to_eth_tkeep    // �ֽ���Ч
);

//*****************************************************
//**                    main code
//*****************************************************

// GMIIתAXI 
gmii_to_axi u_gmii_to_axi(
    .gmii_rx_clk    (gmii_rx_clk),
    .tx_clk_out     (tx_clk_out),
    .rst_n          (sys_rst_n),
    .gmii_rx_dv     (gmii_rx_dv),
    .gmii_rxd       (gmii_rxd),
    .axis_tvalid    (eth_to_axis_tvalid),
    .axis_tdata     (eth_to_axis_tdata),
    .axis_tlast     (eth_to_axis_tlast),
    .axis_tkeep     (eth_to_axis_tkeep),
    .axis_tready    (eth_to_axis_tready)
);

// AXIתGMII
axi_to_gmii u_axi_to_gmii(    
    .rst_n          (sys_rst_n),
    .tx_clk_out     (tx_clk_out), 
    .gmii_tx_clk    (gmii_tx_clk),
    .axis_tvalid    (axis_to_eth_tvalid),
    .axis_tdata     (axis_to_eth_tdata),
    .axis_tlast     (axis_to_eth_tlast),
    .axis_tkeep     (axis_to_eth_tkeep),
    .gmii_tx_en     (gmii_tx_en),
    .gmii_txd       (gmii_txd)
);

endmodule