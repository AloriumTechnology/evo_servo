//===========================================================================
//  Copyright(c) Alorium Technology Group Inc., 2020
//  ALL RIGHTS RESERVED
//===========================================================================
//
// File name:  : evo_xb.sv
// Author      : Steve Phillips
// Contact     : support@aloriumtech.com
// Description : 
// 
// The evo_xb module is a wrapper module to allow IP blocks and XB blocks 
// to be instantiated and integrated into tthe Evo FPGA design
//
//===========================================================================


module evo_xb
  #(parameter NUM_SERVOS = 13 // BRC
  //#(parameter NUM_SERVOS = 32
    )
   (// Basic clock and reset
    input                             clk,
    input                             reset_n,
    // Other clocks and reset
    input                             pwr_on_nrst,
    input                             pll_locked,
    input                             clk_bsp,
    input                             clk_60,
    input                             clk_120,
    input                             clk_16,
    input                             clk_32,
    input                             en16mhz,
    input                             en1mhz,
    input                             en128khz,
    // PMUX connections
    output logic [PORT_D_DWIDTH-1:0]  port_d_pmux_dir_o,
    output logic [PORT_D_DWIDTH-1:0]  port_d_pmux_out_o,
    output logic [PORT_D_DWIDTH-1:0]  port_d_pmux_en_o,
    input logic [PORT_D_DWIDTH-1:0]   port_d_pmux_in_i,
    
    output logic [PORT_E_DWIDTH-1:0]  port_e_pmux_dir_o,
    output logic [PORT_E_DWIDTH-1:0]  port_e_pmux_out_o,
    output logic [PORT_E_DWIDTH-1:0]  port_e_pmux_en_o,
    input logic [PORT_E_DWIDTH-1:0]   port_e_pmux_in_i,
    
    output logic [PORT_F_DWIDTH-1:0]  port_f_pmux_dir_o,
    output logic [PORT_F_DWIDTH-1:0]  port_f_pmux_out_o,
    output logic [PORT_F_DWIDTH-1:0]  port_f_pmux_en_o,
    input logic [PORT_F_DWIDTH-1:0]   port_f_pmux_in_i,
    
    output logic [PORT_G_DWIDTH-1:0]  port_g_pmux_dir_o,
    output logic [PORT_G_DWIDTH-1:0]  port_g_pmux_out_o,
    output logic [PORT_G_DWIDTH-1:0]  port_g_pmux_en_o,
    input logic [PORT_G_DWIDTH-1:0]   port_g_pmux_in_i,
    
    output logic [PORT_Z_DWIDTH-1:0]  port_z_pmux_dir_o,
    output logic [PORT_Z_DWIDTH-1:0]  port_z_pmux_out_o,
    output logic [PORT_Z_DWIDTH-1:0]  port_z_pmux_en_o,
    input logic [PORT_Z_DWIDTH-1:0]   port_z_pmux_in_i,

    // Interface to evo_i2c_ctrl (Avalon MM Slave)
    input logic [CSR_AWIDTH-1:0]      avs_csr_address,
    input logic                       avs_csr_read, 
    output logic                      avs_csr_waitresponse,
    output logic                      avs_csr_readdatavalid,
    output logic                      avs_csr_waitrequest,
    input logic                       avs_csr_write,
    input logic [CSR_DWIDTH-1:0]      avs_csr_writedata,
    output logic [CSR_DWIDTH-1:0]     avs_csr_readdata
    );       
   
   // CSR slave output from the evo_xb_info module
   logic                              avs_info_csr_readdatavalid;
   logic                              avs_info_csr_waitrequest;
   logic [CSR_DWIDTH-1:0]             avs_info_csr_readdata;
   // CSR slave output from the evo_servo module
   logic                              avs_servo_csr_readdatavalid;
   logic                              avs_servo_csr_waitrequest;
   logic [CSR_DWIDTH-1:0]             avs_servo_csr_readdata;
   
   // OR together the slave outputs of any CSR slaves
   always_comb avs_csr_readdatavalid = avs_info_csr_readdatavalid |
                                       avs_servo_csr_readdatavalid;
   always_comb avs_csr_waitrequest   = avs_info_csr_waitrequest   |
                                       avs_servo_csr_waitrequest;
   always_comb avs_csr_readdata      = avs_info_csr_readdata      |
                                       avs_servo_csr_readdata;

   // Tie off waitresponse. Not used.
   always_comb avs_csr_waitresponse  = 1'h0;

   // The pin control triplet coming from evo_servo to evo_eb_pmux
   logic [NUM_SERVOS-1:0]             servo_pmux_dir;
   logic [NUM_SERVOS-1:0]             servo_pmux_out;
   logic [NUM_SERVOS-1:0]             servo_pmux_en;
   
   //----------------------------------------------------------------------
   // Instance Name:  evo_xb_info_inst
   // Module Type:    evo_xb_info
   //
   //----------------------------------------------------------------------
   evo_xb_info
   evo_xb_info_inst
     (
      .clk                            (clk),
      .rstn                           (reset_n),
      // CSR bus (Avalon MM Slave)
      .avs_csr_address                (avs_csr_address),
      .avs_csr_read                   (avs_csr_read),
      .avs_csr_readdatavalid          (avs_info_csr_readdatavalid),
      .avs_csr_waitrequest            (avs_info_csr_waitrequest),
      .avs_csr_write                  (avs_csr_write),
      .avs_csr_writedata              (avs_csr_writedata),
      .avs_csr_readdata               (avs_info_csr_readdata)
      );

   //----------------------------------------------------------------------
   // Instance Name:  evo_xb_pmux_inst
   // Module Type:    evo_xb_pmux
   //
   //----------------------------------------------------------------------

   // How many PMUX inputs per port. If multiple XBs will be able
   // to control any one pin then these values should be adjusted
   // accordanly
   localparam PORT_D_PMUX_WIDTH = 1;
   localparam PORT_E_PMUX_WIDTH = 1;
   localparam PORT_F_PMUX_WIDTH = 1;
   localparam PORT_G_PMUX_WIDTH = 1;
   localparam PORT_Z_PMUX_WIDTH = 1;
   
   // Create input busses for port pmux inputs, setting width to be a
   // multiple of the PRT_DWIDTH for that port. The multiple is the
   // max number of XBs that want to contol a single pin in that port
   logic [(PORT_D_DWIDTH*PORT_D_PMUX_WIDTH)-1:0] port_d_pmux_dir_i,
                                                 port_d_pmux_out_i,
                                                 port_d_pmux_en_i;
   logic [(PORT_E_DWIDTH*PORT_E_PMUX_WIDTH)-1:0] port_e_pmux_dir_i,
                                                 port_e_pmux_out_i,
                                                 port_e_pmux_en_i;
   logic [(PORT_F_DWIDTH*PORT_F_PMUX_WIDTH)-1:0] port_f_pmux_dir_i,
                                                 port_f_pmux_out_i,
                                                 port_f_pmux_en_i;
   logic [(PORT_G_DWIDTH*PORT_G_PMUX_WIDTH)-1:0] port_g_pmux_dir_i,
                                                 port_g_pmux_out_i,
                                                 port_g_pmux_en_i;
   logic [(PORT_Z_DWIDTH*PORT_Z_PMUX_WIDTH)-1:0] port_z_pmux_dir_i,
                                                 port_z_pmux_out_i,
                                                 port_z_pmux_en_i;

   // Assign PMUX connections from XB modules to the desired ports and
   // pins. Width of these busses will always be in multiple of the
   // Port width.
   always_comb begin
      port_d_pmux_dir_i = {{PORT_D_DWIDTH-NUM_SERVOS{1'h0}},servo_pmux_dir[NUM_SERVOS-1:0]}; // BRC
      port_d_pmux_out_i = {{PORT_D_DWIDTH-NUM_SERVOS{1'h0}},servo_pmux_out[NUM_SERVOS-1:0]};
      port_d_pmux_en_i  = {{PORT_D_DWIDTH-NUM_SERVOS{1'h0}},servo_pmux_en[NUM_SERVOS-1:0]};
      //port_d_pmux_dir_i = 'h0;
      //port_d_pmux_out_i = 'h0;
      //port_d_pmux_en_i  = 'h0;
      port_e_pmux_dir_i = 'h0; // BRC
      port_e_pmux_out_i = 'h0;
      port_e_pmux_en_i  = 'h0;
      //port_e_pmux_dir_i = {{PORT_E_DWIDTH-NUM_SERVOS{1'h0}},servo_pmux_dir[NUM_SERVOS-1:0]};
      //port_e_pmux_out_i = {{PORT_E_DWIDTH-NUM_SERVOS{1'h0}},servo_pmux_out[NUM_SERVOS-1:0]};
      //port_e_pmux_en_i  = {{PORT_E_DWIDTH-NUM_SERVOS{1'h0}},servo_pmux_en[NUM_SERVOS-1:0]};
      port_f_pmux_dir_i = 'h0;
      port_f_pmux_out_i = 'h0;
      port_f_pmux_en_i  = 'h0;
      port_g_pmux_dir_i = 'h0;
      port_g_pmux_out_i = 'h0;
      port_g_pmux_en_i  = 'h0;
      port_z_pmux_dir_i = 'h0;
      port_z_pmux_out_i = 'h0;
      port_z_pmux_en_i  = 'h0;
   end

   evo_xb_pmux
     #(
       .D_MUX_WIDTH (PORT_D_PMUX_WIDTH),
       .E_MUX_WIDTH (PORT_E_PMUX_WIDTH),
       .F_MUX_WIDTH (PORT_F_PMUX_WIDTH),
       .G_MUX_WIDTH (PORT_G_PMUX_WIDTH),
       .Z_MUX_WIDTH (PORT_Z_PMUX_WIDTH)
       )
   evo_xb_pmux_inst
     (// PMUX connections from XB/IP blocks
      .port_d_dir_i (port_d_pmux_dir_i),
      .port_d_out_i (port_d_pmux_out_i),
      .port_d_en_i  (port_d_pmux_en_i),
      .port_e_dir_i (port_e_pmux_dir_i),
      .port_e_out_i (port_e_pmux_out_i),
      .port_e_en_i  (port_e_pmux_en_i),
      .port_f_dir_i (port_f_pmux_dir_i),
      .port_f_out_i (port_f_pmux_out_i),
      .port_f_en_i  (port_f_pmux_en_i),
      .port_g_dir_i (port_g_pmux_dir_i),
      .port_g_out_i (port_g_pmux_out_i),
      .port_g_en_i  (port_g_pmux_en_i),
      .port_z_dir_i (port_z_pmux_dir_i),
      .port_z_out_i (port_z_pmux_out_i),
      .port_z_en_i  (port_z_pmux_en_i),
      // PMUX connections to ports
      .port_d_dir_o (port_d_pmux_dir_o),
      .port_d_out_o (port_d_pmux_out_o),
      .port_d_en_o  (port_d_pmux_en_o),
      .port_e_dir_o (port_e_pmux_dir_o),
      .port_e_out_o (port_e_pmux_out_o),
      .port_e_en_o  (port_e_pmux_en_o),
      .port_f_dir_o (port_f_pmux_dir_o),
      .port_f_out_o (port_f_pmux_out_o),
      .port_f_en_o  (port_f_pmux_en_o),
      .port_g_dir_o (port_g_pmux_dir_o),
      .port_g_out_o (port_g_pmux_out_o),
      .port_g_en_o  (port_g_pmux_en_o),
      .port_z_dir_o (port_z_pmux_dir_o),
      .port_z_out_o (port_z_pmux_out_o),
      .port_z_en_o  (port_z_pmux_en_o)
      );
   
   //======================================================================
   //
   // INSTANTIATE YOUR CUSTOM MODULES HERE
   //
   //======================================================================
   
   //======================================================================
   // Instance Name:  evo_servo_inst
   // Module Type:    evo_servo
   //----------------------------------------------------------------------
   // The evo_servo is a wrapper around the xlr8_servo module. In this 
   // case we are connecting up the E pins to the servo
   //======================================================================
   evo_servo
     #(.NUM_SERVOS (NUM_SERVOS),
       .EVO_SERVO_ADDR (EVO_SERVO_ADDR)
       )
   evo_servo_inst
     (
      .clk                            (clk),
      .reset_n                        (reset_n),
      .en1mhz                         (en1mhz),
      .pmux_dir_o                     (servo_pmux_dir),
      .pmux_out_o                     (servo_pmux_out),
      .pmux_en_o                      (servo_pmux_en),
      .pmux_in_i                      (port_d_pmux_in_i[NUM_SERVOS-1:0]),
      //.pmux_in_i                      (port_e_pmux_in_i[NUM_SERVOS-1:0]),
      // CSR bus (Avalon MM Slave)
      .avs_csr_address                (avs_csr_address),
      .avs_csr_read                   (avs_csr_read),
      .avs_csr_readdatavalid          (avs_servo_csr_readdatavalid),
      .avs_csr_waitrequest            (avs_servo_csr_waitrequest),
      .avs_csr_write                  (avs_csr_write),
      .avs_csr_writedata              (avs_csr_writedata),
      .avs_csr_readdata               (avs_servo_csr_readdata),
      .priv_wr_pw                     (1'b0),
      .priv_index                     (5'b0),
      .priv_pw                        (16'b0)
      );
   
   
endmodule // evo_xb
