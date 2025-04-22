//----------------------------------------------------------------------------------------
// File name:           eth_top.v
// Created by:          珊瑚伊斯特
// Created date:        2025.4
// Version:             V0.1
// Descriptions:        eth_top
//以太网top。
//----------------------------------------------------------------------------------------

module eth_top(
    // 系统接口
    input           dclk,           // 内部时钟100MHz
    input           sys_rst_n,      // 系统复位,低电平有效

    // MDIO接口
    output          eth_mdc,        // MDIO时钟
    inout           eth_mdio,       // MDIO数据 
    input           key,            // MDIO软复位触发
    output   [1:0]  led,            // LED连接速率指示

    // RGMII接口
    input           rgmii_rxc,      // 接收时钟2.5MHz,25MHz,125MHz
    input           rgmii_rx_ctl,   // 接收控制
    input    [3:0]  rgmii_rxd,      // 接收数据
    output          rgmii_txc,      // 发送时钟2.5MHz,25MHz,125MHz   
    output          rgmii_tx_ctl,   // 发送控制 
    output   [3:0]  rgmii_txd,      // 发送数据

    // 时钟接口
    input           tx_clk_out,     // 发送时钟

    // AXI接口
    output          eth_to_axis_tvalid,  // 数据有效
    output   [63:0] eth_to_axis_tdata,   // 数据
    output          eth_to_axis_tlast,   // 最后一拍
    output   [7:0]  eth_to_axis_tkeep,   // 字节有效
    input           eth_to_axis_tready,  // 准备就绪
    input           axis_to_eth_tvalid,    // 数据有效
    input    [63:0] axis_to_eth_tdata,     // 数据
    input           axis_to_eth_tlast,     // 最后一拍
    input    [7:0]  axis_to_eth_tkeep      // 字节有效
);

//wire define
wire   [1:0]  speed_mode;             // 速率模式

// GMII接口 
wire          gmii_rx_clk;            // 接收时钟2.5MHz,25MHz,125MHz
wire          gmii_rx_dv;             // 接收有效
wire   [7:0]  gmii_rxd;               // 接收数据
wire          gmii_tx_clk;            // 发送时钟2.5MHz,25MHz,125MHz  
wire          gmii_tx_en;             // 发送有效
wire   [7:0]  gmii_txd;              // 发送数据

//*****************************************************
//**                    main code
//*****************************************************

// 实例化MDIO顶层模块
mdio_top u_mdio_top(
    .dclk          (dclk),         // 使用clk_wiz_0的输出时钟
    .sys_rst_n     (sys_rst_n),
    .eth_mdc       (eth_mdc),
    .eth_mdio      (eth_mdio),
    .key           (key),
    .led           (led),
    .speed_mode    (speed_mode)
);

// GMII-RGMII转换
gmii_to_rgmii u_gmii_to_rgmii(
    // GMII接口
    .gmii_rx_clk   (gmii_rx_clk),
    .gmii_rx_dv    (gmii_rx_dv),
    .gmii_rxd      (gmii_rxd),
    .gmii_tx_clk   (gmii_tx_clk),
    .gmii_tx_en    (gmii_tx_en), 
    .gmii_txd      (gmii_txd),
    // RGMII接口
    .rgmii_rxc     (rgmii_rxc),
    .rgmii_rx_ctl  (rgmii_rx_ctl),
    .rgmii_rxd     (rgmii_rxd),
    .rgmii_txc     (rgmii_txc),
    .rgmii_tx_ctl  (rgmii_tx_ctl),
    .rgmii_txd     (rgmii_txd),
    // 配置
    .speed_mode    (speed_mode)
);

// GMII-AXI转换
gmii8b_axi64b_top u_gmii8b_axi64b_top(
    // 系统接口
    .sys_rst_n     (sys_rst_n),
    .tx_clk_out    (tx_clk_out),
    
    // GMII接口
    .gmii_rx_clk   (gmii_rx_clk),
    .gmii_rx_dv    (gmii_rx_dv),
    .gmii_rxd      (gmii_rxd),
    .gmii_tx_clk   (gmii_tx_clk),
    .gmii_tx_en    (gmii_tx_en),
    .gmii_txd      (gmii_txd),
    
    // AXI接口
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