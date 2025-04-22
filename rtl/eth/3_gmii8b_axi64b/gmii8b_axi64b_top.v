//----------------------------------------------------------------------------------------
// File name:           gmii8b_axi64b_top.v
// Created by:          珊瑚伊斯特
// Created date:        2025.4
// Version:             V0.1
// Descriptions:        gmii8b_axi64b_exchange
//gmii8b接口到axi64b接口的互换，实现数据的拼接和拆分。
//----------------------------------------------------------------------------------------

module gmii8b_axi64b_top(
    // 系统接口
    input           sys_rst_n,      // 系统复位,低电平有效
    input           tx_clk_out,     // 发送时钟

    // GMII接口 
    input           gmii_rx_clk,    // 接收时钟2.5MHz,25MHz,125MHz
    input           gmii_rx_dv,     // 接收有效
    input    [7:0]  gmii_rxd,       // 接收数据
    input           gmii_tx_clk,    // 发送时钟2.5MHz,25MHz,125MHz  
    input           gmii_tx_en,     // 发送有效
    input    [7:0]  gmii_txd,       // 发送数据

    // AXI接口
    output          eth_to_axis_tvalid,  // 数据有效
    output   [63:0] eth_to_axis_tdata,   // 数据
    output          eth_to_axis_tlast,   // 最后一拍
    output   [7:0]  eth_to_axis_tkeep,   // 字节有效
    input           eth_to_axis_tready,  // 准备就绪
    input           axis_to_eth_tvalid,  // 数据有效
    input    [63:0] axis_to_eth_tdata,   // 数据
    input           axis_to_eth_tlast,   // 最后一拍
    input    [7:0]  axis_to_eth_tkeep    // 字节有效
);

//*****************************************************
//**                    main code
//*****************************************************

// GMII转AXI 
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

// AXI转GMII
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