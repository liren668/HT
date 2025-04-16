//----------------------------------------------------------------------------------------
// File name:           mdio_top.v
// Created by:          珊瑚伊斯特
// Created date:        2025.4
// Version:             V0.1
// Descriptions:        mdio_top，mdio接口驱动和读写控制。
//2025.4.16增加速率状态输出，并可以根据上层要求配置速率 
//----------------------------------------------------------------------------------------
module mdio_top(
    input          dclk,      // 从clk_wiz_0获取的100MHz时钟
    input          sys_rst_n, // 系统复位，低电平有效
    // MDIO接口
    output         eth_mdc,   // PHY管理接口的时钟信号
    inout          eth_mdio,  // PHY管理接口的双向数据信号
    
    input          key,       // 触摸按键
    input  [1:0]   speed_set, // 速率配置输入 00:10M 01:100M 10:1000M
    output         speed_flag,// 速率配置完成标志
    output         [1:0]  led,  // LED连接速率指示
    output         [1:0]  speed_mode  // 连接速率输出
    );
    
// wire define
wire          sys_clk;     // 系统时钟
wire          op_exec;     // 触发开始信号
wire          op_rh_wl;    // 低电平写，高电平读
wire  [4:0]   op_addr;     // 寄存器地址
wire  [15:0]  op_wr_data;  // 写入寄存器的数据
wire          op_done;     // 读写完成
wire  [15:0]  op_rd_data;  // 读出的数据
wire          op_rd_ack;   // 读应答信号 0:应答 1:未应答
wire          dri_clk;     // 驱动时钟

assign speed_mode = led; // 新增speed_mode赋值
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
    .soft_rst_trig (key      ),  
    .op_done       (op_done  ),  
    .op_rd_data    (op_rd_data),  
    .op_rd_ack     (op_rd_ack),  
    .op_exec       (op_exec  ),  
    .op_rh_wl      (op_rh_wl ),  
    .op_addr       (op_addr  ),  
    .op_wr_data    (op_wr_data),  
    .led           (led      )
);      

endmodule