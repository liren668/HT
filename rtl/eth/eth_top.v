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
    input           clk_ila,        //ILA����ʱ��250M
    input           dclk,           //100Mʱ��
    input           tx_clk_out,     //���շ���������ʱ��156.25M
    input           sys_rst_n,      //ϵͳ��λ

    // �ڵ���Ϣ�ӿ�
    input    [3:0]  node_id,        // �ڵ�ID����
    input    [3:0]  eth_type,       // ��̫����������   

    // MDIO�ӿ�
    output          eth_mdc,         //MDIOʱ��
    inout           eth_mdio,        //MDIO����
    input           key,             //����
    output   [1:0]  led,             //LED

    // RGMII�ӿ�
    input           rgmii_rxc,       //RGMII����ʱ��2.5M��25M��125M 
    input           rgmii_rx_ctl,    //RGMII���տ���
    input    [3:0]  rgmii_rxd,       //RGMII��������
    output          rgmii_txc,       //RGMII����ʱ��2.5M��25M��125M
    output          rgmii_tx_ctl,    //RGMII���Ϳ���
    output   [3:0]  rgmii_txd,       //RGMII��������


    // AXI�ӿ�
    output          eth_to_axis_tvalid, //AXI������Ч�ź�
    output   [63:0] eth_to_axis_tdata,  //AXI��������
    output          eth_to_axis_tlast,  //AXI���ͽ����ź�
    output   [7:0]  eth_to_axis_tkeep,  //AXI����������Чλ��
    input           eth_to_axis_tready, //AXI����׼���ź�
    input           axis_to_eth_tvalid, //AXI������Ч�ź�
    input    [63:0] axis_to_eth_tdata,  //AXI��������
    // input           axis_to_eth_tlast,  //AXI���ս����ź�
    input    [7:0]  axis_to_eth_tkeep   //AXI����������Чλ��
);

//mdio�ӿ��ź�
wire          key_pulse;        // ����������������ź�
wire          dri_clk;  
wire          op_exec;
wire          op_rh_wl;
wire   [4:0]  op_addr;
wire   [15:0] op_wr_data;
wire          op_done;
wire   [15:0] op_rd_data;
wire          op_rd_ack;

// GMII�ӿ��ź�
wire          gmii_rx_clk;
wire          gmii_rx_dv;
wire   [7:0]  gmii_rxd;
wire          gmii_tx_clk;
wire          gmii_tx_en;
wire   [7:0]  gmii_txd;
// wire          gmii_rx_er;       // GMII���մ����ź�
// wire          gmii_tx_er;       // GMII���ʹ����ź�

//ILA����ʱ��
// wire          clk_ila1;

//----------------------------------------------------------------------------------------

//ILA���� ʱ�ӻ���
// BUFG BUFG_inst (
//     .I            (clk_ila),      // 1-bit input: Clock input
//     .O            (clk_ila1)  // 1-bit output: Clock output
// );

// ILA
//ILAʵ����
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
// MDIO����ģ��
// ��������ģ��
key_debounce u_key_debounce(
    .sys_clk    (dclk),
    .sys_rst_n  (sys_rst_n),
    .key        (key),
    .key_filter  (key_pulse)
);

// MDIO�ӿ�����
mdio_dri #(
    .PHY_ADDR    (5'h04),    // PHY��ַ 3'b100
    .CLK_DIV     (6'd16)     // ��Ƶϵ��
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

// MDIO�ӿڶ�д����    
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
// GMII-RGMIIת��ģ��
 
assign gmii_tx_clk = gmii_rx_clk;
// ��������ģʽ���ô����ź�
// assign gmii_tx_er = (speed_mode == 2'b11) ? 1'b1 : 1'b0;
// assign gmii_rx_er = (speed_mode == 2'b11) ? 1'b1 : 1'b0;

//RGMII����
rgmii_rx u_rgmii_rx(
    .gmii_rx_clk   (gmii_rx_clk ),
    .rgmii_rxc     (rgmii_rxc   ),
    .rgmii_rx_ctl  (rgmii_rx_ctl),
    .rgmii_rxd     (rgmii_rxd   ),
    // .gmii_rx_er    (gmii_rx_er  ),
    .gmii_rx_dv    (gmii_rx_dv ),
    .gmii_rxd      (gmii_rxd   )
    );

//RGMII����
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
// GMII-AXIת��ģ��
// ��������

// GMII��AXI-Streamת��ģ��ʵ����
gmii_to_axi u_gmii_to_axi (
    .gmii_rx_clk    (gmii_rx_clk),
    .tx_clk_out     (tx_clk_out),
    .rst_n          (sys_rst_n),
    .gmii_rx_dv     (gmii_rx_dv),
    .gmii_rxd       (gmii_rxd),
    .node_id        (node_id),      // ֱ�����ӽڵ�ID����
    .eth_type       (eth_type),     // ֱ��������̫����������
    .axis_tvalid    (eth_to_axis_tvalid),
    .axis_tdata     (eth_to_axis_tdata),
    .axis_tlast     (eth_to_axis_tlast),
    .axis_tkeep     (eth_to_axis_tkeep),
    .axis_tready    (eth_to_axis_tready)
);

// AXI��GMIIת��
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