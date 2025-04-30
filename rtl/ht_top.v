//----------------------------------------------------------------------------------------
// File name:           ht_top.v
// Created by:          珊瑚伊斯特
// Created date:        2025.4
// Version:             V0.1
// Descriptions:        Hybrid_Transport
//各种数据混合传输的top。各种数据轮询发送和接收。
//----------------------------------------------------------------------------------------

module ht_top(
    // 系统接口
    input           sys_clk_p,        // 系统时钟正端100MHz
    input           sys_clk_n,        // 系统时钟负端  
    input           sys_rst_n,        // 系统复位,低电平有效

    // MDIO接口
    output          eth_mdc,          // MDIO时钟
    inout           eth_mdio,         // MDIO数据 
    input           key,              // MDIO软复位触发
    output   [1:0]  led,              // LED连接速率指示

    // RGMII接口
    input           rgmii_rxc,        // 接收时钟2.5MHz,25MHz,125MHz
    input           rgmii_rx_ctl,     // 接收控制
    input    [3:0]  rgmii_rxd,        // 接收数据
    output          rgmii_txc,        // 发送时钟2.5MHz,25MHz,125MHz   
    output          rgmii_tx_ctl,     // 发送控制 
    output   [3:0]  rgmii_txd,        // 发送数据

    //HDMI接口
    // input           hdmi_rx_p,        // HDMI接收正端
    // input           hdmi_rx_n,        // HDMI接收负端
    // output          hdmi_tx_p,        // HDMI发送正端
    // output          hdmi_tx_n,        // HDMI发送负端 

    // SFP接口
    input           q0_ck1_n_in,      // 参考时钟负端156.25MHz
    input           q0_ck1_p_in,      // 参考时钟正端
    input           rxn_in,           // 差分接收负端
    input           rxp_in,           // 差分接收正端
    output          txn_out,          // 差分发送负端
    output          txp_out,          // 差分发送正端
    output  [1:0]   tx_disable     // 发送使能    
);

// 添加本地节点信息定义
parameter LOCAL_NODE_INFO = 8'h11;  // 本地节点信息 00011001

// 添加数据类型定义
parameter DATA_TYPE_ETH = 4'h1;      // 以太网类型 0001
parameter DATA_TYPE_AUX = 4'h8;      // 辅助数据类型 1000

//交互信息寄存器
reg [8:0] nodes_info_reg [0:15];  //保存各节点信息，0用于根节点，1-15号子节点。

//reg define
reg  [27:0] dely_500ms ;
reg  [2:0]  S_RESET;            //状态信号
reg  [15:0] cnt_rst;
reg         rst_n;
reg         gtwiz_reset_rx_datapath;


//wire define收发器
wire        stat_rx_status;
wire        tx_reset;
wire        rx_reset;
wire        gt0_rst;
wire        gtpowergood_out_0;        // GT电源状态信号

// 时钟和复位
wire          clk_ila;                // ILA调试时钟62.5MHz
wire          clk_ila1;               // ILA时钟
wire          dclk;                   // 内部时钟100MHz
wire          locked;                 // 时钟锁定
wire          tx_clk_out;             // 发送时钟
wire          gt_refclk_out;          // 参考时钟

// AXI发送接口（发送到SFP）
reg           axis_to_sfp_tvalid;      // 数据有效信号
reg    [63:0] axis_to_sfp_tdata;       // 发送数据
reg           axis_to_sfp_tlast;       // 最后一拍标志
reg    [7:0]  axis_to_sfp_tkeep;       // 字节有效标志
wire          axis_to_sfp_tready;      // 接收端准备就绪

// AXI接收接口（从SFP接收）
wire          sfp_to_axis_tvalid;      // 数据有效
wire   [63:0] sfp_to_axis_tdata;       // 接收数据从SFP 
wire          sfp_to_axis_tlast;       // 最后一拍
wire   [7:0]  sfp_to_axis_tkeep;       // 字节有效

// AXI数据分支接口,从AXI总接口选择分支，eth,aux,hdmi,usb等。
wire          eth_to_axis_tvalid;      // 数据有效
wire   [63:0] eth_to_axis_tdata;       // 以太网数据发送axi
wire          eth_to_axis_tlast;       // 最后一拍
wire   [7:0]  eth_to_axis_tkeep;       // 字节有效
reg           axis_to_eth_tvalid;      // 数据有效
reg    [63:0] axis_to_eth_tdata;       // axi数据接收eth
reg           axis_to_eth_tlast;       // 最后一拍
reg    [7:0]  axis_to_eth_tkeep;       // 字节有效
//aux数据分支接口,用于信息交互。
wire          aux_to_axis_tvalid;      // 数据有效
wire   [63:0] aux_to_axis_tdata;       // 辅助数据发送axi
wire          aux_to_axis_tlast;       // 最后一拍
wire   [7:0]  aux_to_axis_tkeep;       // 字节有效
wire          aux_to_axis_tready;      // 发送器准备就绪
reg           axis_to_aux_tvalid;      // 数据有效
reg    [63:0] axis_to_aux_tdata;       // axi数据发送aux
reg           axis_to_aux_tlast;       // 最后一拍
reg    [7:0]  axis_to_aux_tkeep;       // 字节有效

// 控制计数器
reg    [3:0]  send_type_cnt;           // 发送类型计数器

//*****************************************************
//**                    main code
//*****************************************************

assign  tx_disable = 2'b00; 
assign gt0_rst = ~rst_n;    // 系统复位时GT复位，高电平复位

//ILA调试 时钟缓存
BUFG BUFG_inst (
    .I            (clk_ila),      // 1-bit input: Clock input
    .O            (clk_ila1)  // 1-bit output: Clock output
);

//ILA实例化
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

// 时钟管理
clk_wiz_0 u_clk_wiz_0(
    .clk_out1      (dclk),                // 内部时钟100MHz
    .clk_out2      (clk_ila),             // ILA调试时钟250MHz
    .reset         (~sys_rst_n),
    .locked        (locked),
    .clk_in1_p     (sys_clk_p),
    .clk_in1_n     (sys_clk_n)
);

// 复位控制
always@(posedge dclk)begin
    if(!locked)
        cnt_rst <= 16'b0;
    else if(!cnt_rst[15])
        cnt_rst <= cnt_rst + 1'b1;
    else
        cnt_rst <= cnt_rst;
end 

//复位信号，低电平有效
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

//500ms计数器
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

// 发送轮询逻辑
always @(posedge tx_clk_out or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        axis_to_sfp_tvalid <= 1'b0;
        axis_to_sfp_tdata <= 64'h0;
        axis_to_sfp_tlast <= 1'b0;
        axis_to_sfp_tkeep <= 8'h0;
        send_type_cnt <= 4'd0;
    end else begin
        // 根据LOCAL_NODE_INFO低4位中1的位置轮询发送
        case (send_type_cnt)
            4'd0: begin  // 第0类数据，eth数据
                if (LOCAL_NODE_INFO[0]) begin
                    if (eth_to_axis_tvalid) begin  // 有效数据时发送正常数据
                        axis_to_sfp_tvalid <= 1'b1;
                        axis_to_sfp_tdata <= eth_to_axis_tdata;
                        axis_to_sfp_tlast <= eth_to_axis_tlast;
                        axis_to_sfp_tkeep <= eth_to_axis_tkeep;
                    end else begin  // 无效数据时发送节点信息
                        axis_to_sfp_tvalid <= 1'b1;
                        axis_to_sfp_tdata <= {56'h0, LOCAL_NODE_INFO[7:4], 4'h0};
                        axis_to_sfp_tlast <= eth_to_axis_tlast;
                        axis_to_sfp_tkeep <= 8'h00;
                    end
                end else begin
                    axis_to_sfp_tvalid <= 1'b0;
                end
                // 找下一个置1位
                send_type_cnt <= (LOCAL_NODE_INFO[1]) ? 4'd1 :
                               (LOCAL_NODE_INFO[2]) ? 4'd2 :
                               (LOCAL_NODE_INFO[3]) ? 4'd3 : 4'd0;
            end
            // 4'd1: begin  // 第1类数据
            //     if (LOCAL_NODE_INFO[1] && data2_valid) begin
            //         axis_to_sfp_tvalid <= 1'b1;
            //         axis_to_sfp_tdata <= {data2_data[63:8], LOCAL_NODE_INFO};
            //         axis_to_sfp_tlast <= data2_last;
            //         axis_to_sfp_tkeep <= data2_keep;
            //     end else begin
            //         axis_to_sfp_tvalid <= 1'b0;
            //     end
            //     // 找下一个置1位
            //     send_type_cnt <= (LOCAL_NODE_INFO[2]) ? 4'd2 :
            //                    (LOCAL_NODE_INFO[3]) ? 4'd3 :
            //                    (LOCAL_NODE_INFO[0]) ? 4'd0 : 4'd1;
            // end
            // 4'd2: begin  // 第2类数据
            //     if (LOCAL_NODE_INFO[2] && data3_valid) begin
            //         axis_to_sfp_tvalid <= 1'b1;
            //         axis_to_sfp_tdata <= {data3_data[63:8], LOCAL_NODE_INFO};
            //         axis_to_sfp_tlast <= data3_last;
            //         axis_to_sfp_tkeep <= data3_keep;
            //     end else begin
            //         axis_to_sfp_tvalid <= 1'b0;
            //     end
            //     // 找下一个置1位
            //     send_type_cnt <= (LOCAL_NODE_INFO[3]) ? 4'd3 :
            //                    (LOCAL_NODE_INFO[0]) ? 4'd0 :
            //                    (LOCAL_NODE_INFO[1]) ? 4'd1 : 4'd2;
            // end
            // 4'd3: begin  // 第3类数据(辅助数据)
            //     if (LOCAL_NODE_INFO[3] && aux_to_axis_tvalid) begin
            //         axis_to_sfp_tvalid <= 1'b1;
            //         axis_to_sfp_tdata <= {aux_to_axis_tdata[63:8], LOCAL_NODE_INFO};
            //         axis_to_sfp_tlast <= aux_to_axis_tlast;
            //         axis_to_sfp_tkeep <= aux_to_axis_tkeep;
            //     end else begin
            //         axis_to_sfp_tvalid <= 1'b0;
            //     end
            //     // 找下一个置1位
            //     send_type_cnt <= (LOCAL_NODE_INFO[0]) ? 4'd0 :
            //                    (LOCAL_NODE_INFO[1]) ? 4'd1 :
            //                    (LOCAL_NODE_INFO[2]) ? 4'd2 : 4'd3;
            // end
            default: send_type_cnt <= 4'd0;
                        
        endcase
        
        // 当LOCAL_NODE_INFO全为0时，发送空闲数据
        if (LOCAL_NODE_INFO[3:0] == 4'h0) begin
            axis_to_sfp_tvalid <= 1'b1;
            axis_to_sfp_tdata <= {56'h0, LOCAL_NODE_INFO[7:4],4'h0};
            axis_to_sfp_tlast <= 1'b1;
            axis_to_sfp_tkeep <= 8'h01;
        end
    end
end

// 接收端数据分发逻辑
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
            case (sfp_to_axis_tdata[7:4])  // 检查数据类型
                DATA_TYPE_ETH: begin  // 以太网数据
                    axis_to_eth_tvalid <= 1'b1;
                    axis_to_eth_tdata <= sfp_to_axis_tdata;
                    axis_to_eth_tlast <= sfp_to_axis_tlast;
                    axis_to_eth_tkeep <= sfp_to_axis_tkeep;
                end
                DATA_TYPE_AUX: begin  // 辅助数据
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

// 实例化以太网控制模块
eth_top u_eth_top(
    // 系统接口
    .dclk          (dclk),           // 内部时钟100MHz
    .sys_rst_n     (sys_rst_n),      // 系统复位,低电平有效
    .tx_clk_out    (tx_clk_out),     // 发送时钟

    // MDIO接口
    .eth_mdc       (eth_mdc),        // MDIO时钟
    .eth_mdio      (eth_mdio),       // MDIO数据 
    .key           (key),            // MDIO软复位触发
    .led           (led),            // LED连接速率指示

    // RGMII接口
    .rgmii_rxc     (rgmii_rxc),      // 接收时钟2.5MHz,25MHz,125MHz
    .rgmii_rx_ctl  (rgmii_rx_ctl),   // 接收控制
    .rgmii_rxd     (rgmii_rxd),      // 接收数据
    .rgmii_txc     (rgmii_txc),      // 发送时钟2.5MHz,25MHz,125MHz   
    .rgmii_tx_ctl  (rgmii_tx_ctl),   // 发送控制 
    .rgmii_txd     (rgmii_txd),      // 发送数据

    // AXI接口
    .eth_to_axis_tvalid (eth_to_axis_tvalid),  // 数据有效
    .eth_to_axis_tdata  (eth_to_axis_tdata),   // 数据
    .eth_to_axis_tlast  (eth_to_axis_tlast),   // 最后一拍
    .eth_to_axis_tkeep  (eth_to_axis_tkeep),   // 字节有效
    .eth_to_axis_tready (eth_to_axis_tready),  // 准备就绪
    .axis_to_eth_tvalid (axis_to_eth_tvalid),  // 数据有效
    .axis_to_eth_tdata  (axis_to_eth_tdata),   // 数据
    .axis_to_eth_tlast  (axis_to_eth_tlast),   // 最后一拍
    .axis_to_eth_tkeep  (axis_to_eth_tkeep)    // 字节有效
);

// 以太网核例化
xxv_ethernet_0 u_xxv_ethernet_0 (
  //光口差分数据
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

  //光口参考时钟
  .gt_refclk_p              (q0_ck1_p_in),  // input wire gt_refclk_p
  .gt_refclk_n              (q0_ck1_n_in),  // input wire gt_refclk_n
  
  .gt_refclk_out            (gt_refclk_out),    // output wire gt_refclk_out
  .gtpowergood_out_0        (gtpowergood_out_0),    // output wire gtpowergood_out_0
  .user_rx_reset_0          (rx_reset),  // output wire user_rx_reset_0
  
  //AXI4 Stream 发射接口信号
  .tx_axis_tready_0        (axis_to_sfp_tready),
  .tx_axis_tvalid_0        (axis_to_sfp_tvalid),
  .tx_axis_tdata_0         (axis_to_sfp_tdata),
  .tx_axis_tlast_0         (axis_to_sfp_tlast), 
  .tx_axis_tkeep_0         (axis_to_sfp_tkeep),
  .tx_axis_tuser_0         (1'b0),
  .tx_preamblein_0         (56'b0),            // input wire [55 : 0] tx_preamblein_0
  
  //RX 控制信号
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
  
  //RX 发送状态信号
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
  
  //AXI4 Stream 接收接口信号
  .rx_axis_tvalid_0        (sfp_to_axis_tvalid),
  .rx_axis_tdata_0         (sfp_to_axis_tdata), 
  .rx_axis_tlast_0         (sfp_to_axis_tlast),
  .rx_axis_tkeep_0         (sfp_to_axis_tkeep),
  .rx_axis_tuser_0         (1'b0),
  
  .tx_unfout_0                    ( ),                // output wire tx_unfout_0

  //TX 状态信号
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
  
  //AXI4?Stream 接口 - TX 路径控制信号和状态信号
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