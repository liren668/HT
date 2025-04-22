//****************************************Copyright (c)***********************************//
//版权所有，盗版必究。
//Copyright(C) 珊瑚伊斯特 2025.3
//All rights reserved                                  
//----------------------------------------------------------------------------------------
// File name:           rgmii_tx
// Last modified Date:  
// Last Version:        
// Descriptions:        RGMII发送模块
//----------------------------------------------------------------------------------------
// Created by:          珊瑚伊斯特
// Created date:        2025.3.5
// Version:             V0.1
// Descriptions:        The original version
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module rgmii_tx(
    //GMII发送端口
    input              gmii_tx_clk , //GMII发送时钟    
    input              gmii_tx_en  , //GMII输出数据有效信号
    input       [7:0]  gmii_txd    , //GMII输出数据   
    // input              gmii_tx_er  , //GMII发送数据错误信号
    //RGMII发送端口
    output             rgmii_txc   , //RGMII发送数据时钟    
    output             rgmii_tx_ctl, //RGMII输出数据有效信号
    output      [3:0]  rgmii_txd     //RGMII输出数据     
    );

//*****************************************************
//**                    main code
//*****************************************************

assign rgmii_txc = gmii_tx_clk;

//输出双沿采样寄存器 (rgmii_tx_ctl)
ODDRE1 #(
    .IS_C_INVERTED     (1'b0),            // Optional inversion for C
    .IS_D1_INVERTED    (1'b0),            // Unsupported, do not use
    .IS_D2_INVERTED    (1'b0),            // Unsupported, do not use
    .SIM_DEVICE        ("ULTRASCALE"),    // Set the device version
    .SRVAL             (1'b0)             // Initializes the ODDRE1 Flip-Flops
)
ODDRE1_tx_ctl (
    .Q     (rgmii_tx_ctl),                     // 1-bit output
    .C     (gmii_tx_clk),                      // clock input
    .D1    (gmii_tx_en),                       // 上升沿数据使能
    .D2    (gmii_tx_en),                        // 下降沿数据使能 - 10M/100M/无效
    .SR    (1'b0)                              // reset
);

//RGMII发送数据
genvar i;
generate for (i=0; i<4; i=i+1)
    begin : txdata_bus
        ODDRE1 #(
            .IS_C_INVERTED(1'b0),      // Optional inversion for C
            .IS_D1_INVERTED(1'b0),     // Unsupported, do not use
            .IS_D2_INVERTED(1'b0),     // Unsupported, do not use
            .SIM_DEVICE("ULTRASCALE"), // Set the device version
            .SRVAL(1'b0)               // Initializes the ODDRE1 Flip-Flops
        )
        ODDRE1_inst (
            .Q     (rgmii_txd[i]),     // 1-bit output
            .C     (gmii_tx_clk),      // clock input
            .D1    (gmii_txd[i]),      // 低4位数据
            .D2    (gmii_txd[4+i]),    // 高4位数据
            .SR    (1'b0)              // reset
        );             
    end
endgenerate

endmodule