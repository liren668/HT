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
    input           clk_ila,        //ILA调试时钟250M
    input           dclk,           //100M时钟
    input           tx_clk_out,     //从收发器过来的时钟156.25M
    input           sys_rst_n,      //系统复位

    // 节点信息接口
    input    [3:0]  node_id,        // 节点ID输入
    input    [3:0]  eth_type,       // 以太网类型输入   

    // MDIO接口
    output          eth_mdc,         //MDIO时钟
    inout           eth_mdio,        //MDIO数据
    input           key,             //按键
    output   [1:0]  led,             //LED

    // RGMII接口
    input           rgmii_rxc,       //RGMII接收时钟2.5M，25M，125M 
    input           rgmii_rx_ctl,    //RGMII接收控制
    input    [3:0]  rgmii_rxd,       //RGMII接收数据
    output          rgmii_txc,       //RGMII发送时钟2.5M，25M，125M
    output          rgmii_tx_ctl,    //RGMII发送控制
    output   [3:0]  rgmii_txd,       //RGMII发送数据


    // AXI接口
    output          eth_to_axis_tvalid, //AXI发送有效信号
    output   [63:0] eth_to_axis_tdata,  //AXI发送数据
    output          eth_to_axis_tlast,  //AXI发送结束信号
    output   [7:0]  eth_to_axis_tkeep,  //AXI发送数据有效位宽
    input           eth_to_axis_tready, //AXI接收准备信号
    input           axis_to_eth_tvalid, //AXI接收有效信号
    input    [63:0] axis_to_eth_tdata,  //AXI接收数据
    // input           axis_to_eth_tlast,  //AXI接收结束信号
    input    [7:0]  axis_to_eth_tkeep   //AXI接收数据有效位宽
);

//mdio接口信号
wire          key_pulse;        // 按键消抖后的脉冲信号
wire          dri_clk;  
wire          op_exec;
wire          op_rh_wl;
wire   [4:0]  op_addr;
wire   [15:0] op_wr_data;
wire          op_done;
wire   [15:0] op_rd_data;
wire          op_rd_ack;

// GMII接口信号
wire          gmii_rx_clk;
wire          gmii_rx_dv;
wire   [7:0]  gmii_rxd;
wire          gmii_tx_clk;
wire          gmii_tx_en;
wire   [7:0]  gmii_txd;
// wire          gmii_rx_er;       // GMII接收错误信号
// wire          gmii_tx_er;       // GMII发送错误信号

//ILA调试时钟
// wire          clk_ila1;

//----------------------------------------------------------------------------------------

//ILA调试 时钟缓存
// BUFG BUFG_inst (
//     .I            (clk_ila),      // 1-bit input: Clock input
//     .O            (clk_ila1)  // 1-bit output: Clock output
// );

// ILA
//ILA实例化
ila_0 u_ila_0 (
    .clk(clk_ila),              // input wire clk
    .probe0(axis_to_eth_tvalid),   // input wire [63:0]  probe0  
    .probe1(axis_to_eth_tdata),  // input wire [7:0]  probe1 
    .probe2(axis_to_eth_tkeep),   // input wire [0:0]  probe2 
    .probe3(tx_clk_out),     // input wire [63:0]  probe3 
    .probe4(eth_to_axis_tvalid),    // input wire [0:0]  probe4 
    .probe5(eth_to_axis_tdata),     // input wire [0:0]  probe5 
    .probe6(eth_to_axis_tkeep),   // input wire [7:0]  probe6 
    .probe7(eth_to_axis_tlast)   // input wire [7:0]  probe6 
);

//----------------------------------------------------------------------------------------
// MDIO控制模块
// 按键消抖模块
key_debounce u_key_debounce(
    .sys_clk    (dclk),
    .sys_rst_n  (sys_rst_n),
    .key        (key),
    .key_filter  (key_pulse)
);

// MDIO接口驱动
mdio_dri #(
    .PHY_ADDR    (5'h04),    // PHY地址 3'b100
    .CLK_DIV     (6'd16)     // 分频系数
    )
    u_mdio_dri(
    .clk        (dclk),
    .rst_n      (sys_rst_n),
    .op_exec    (op_exec   ),
    .op_rh_wl   (op_rh_wl  ),   
    .op_addr    (op_addr   ),   
    .op_wr_data (op_wr_data),   
    .op_done    (op_done   ),   
    .op_rd_data (op_rd_data),   
    .op_rd_ack  (op_rd_ack ),   
    .dri_clk    (dri_clk   ),  
                 
    .eth_mdc    (eth_mdc   ),   
    .eth_mdio   (eth_mdio  )   
);      

// MDIO接口读写控制    
mdio_ctrl  u_mdio_ctrl(
    .clk           (dri_clk  ),  
    .rst_n         (sys_rst_n),  
    .soft_rst_trig (key_pulse      ),  
    .op_done       (op_done  ),  
    .op_rd_data    (op_rd_data),  
    .op_rd_ack     (op_rd_ack),  
    .op_exec       (op_exec  ),  
    .op_rh_wl      (op_rh_wl ),  
    .op_addr       (op_addr  ),  
    .op_wr_data    (op_wr_data),  
    .led           (led      )
);      

//----------------------------------------------------------------------------------------
// GMII-RGMII转换模块
 
assign gmii_tx_clk = gmii_rx_clk;
// 根据速率模式设置错误信号
// assign gmii_tx_er = (speed_mode == 2'b11) ? 1'b1 : 1'b0;
// assign gmii_rx_er = (speed_mode == 2'b11) ? 1'b1 : 1'b0;

//RGMII接收
rgmii_rx u_rgmii_rx(
    .gmii_rx_clk   (gmii_rx_clk ),
    .rgmii_rxc     (rgmii_rxc   ),
    .rgmii_rx_ctl  (rgmii_rx_ctl),
    .rgmii_rxd     (rgmii_rxd   ),
    // .gmii_rx_er    (gmii_rx_er  ),
    .gmii_rx_dv    (gmii_rx_dv ),
    .gmii_rxd      (gmii_rxd   )
    );

//RGMII发送
rgmii_tx u_rgmii_tx(
    .gmii_tx_clk   (gmii_tx_clk ),
    .gmii_tx_en    (gmii_tx_en  ),
    .gmii_txd      (gmii_txd    ),
    // .gmii_tx_er    (gmii_tx_er  ),
              
    .rgmii_txc     (rgmii_txc   ),
    .rgmii_tx_ctl  (rgmii_tx_ctl),
    .rgmii_txd     (rgmii_txd   )
    );

//----------------------------------------------------------------------------------------
// GMII-AXI转换模块
// 参数定义

// GMII到AXI-Stream转换模块实例化
gmii_to_axi u_gmii_to_axi (
    .gmii_rx_clk    (gmii_rx_clk),
    .tx_clk_out     (tx_clk_out),
    .rst_n          (sys_rst_n),
    .gmii_rx_dv     (gmii_rx_dv),
    .gmii_rxd       (gmii_rxd),
    .node_id        (node_id),      // 直接连接节点ID输入
    .eth_type       (eth_type),     // 直接连接以太网类型输入
    .axis_tvalid    (eth_to_axis_tvalid),
    .axis_tdata     (eth_to_axis_tdata),
    .axis_tlast     (eth_to_axis_tlast),
    .axis_tkeep     (eth_to_axis_tkeep),
    .axis_tready    (eth_to_axis_tready)
);

// AXI到GMII转换
axi_to_gmii u_axi_to_gmii(    
    .rst_n          (sys_rst_n),
    .tx_clk_out     (tx_clk_out),
    .gmii_tx_clk    (gmii_tx_clk),
    .axis_tvalid    (axis_to_eth_tvalid),
    .axis_tdata     (axis_to_eth_tdata),
    // .axis_tlast     (axis_to_eth_tlast),
    .axis_tkeep     (axis_to_eth_tkeep),
    .gmii_tx_en     (gmii_tx_en),
    .gmii_txd       (gmii_txd)
);
endmodule