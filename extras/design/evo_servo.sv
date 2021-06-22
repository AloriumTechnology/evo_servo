//===========================================================================
//  Copyright(c) Alorium Technology Group Inc., 2019
//  ALL RIGHTS RESERVED
//===========================================================================
//
// File name:  : evo_servo.sv
// Author      : 
// Contact     : support@aloriumtech.com
// Description : 
// 
// XB Servo module integration under EVO
//
//===========================================================================

module evo_servo
  #(parameter CSR_AWIDTH     = 12,
    parameter CSR_DWIDTH     = 32,
    parameter NUM_SERVOS     = 32,
    parameter EVO_SERVO_ADDR = 12'h0
    )
   (
    input                         clk,
    input                         reset_n,
    input                         en1mhz, 
    // PMUX connections
    output logic [NUM_SERVOS-1:0] pmux_dir_o, // To Port logic
    output logic [NUM_SERVOS-1:0] pmux_out_o, // To Port logic
    output logic [NUM_SERVOS-1:0] pmux_en_o, // To Port logic
    input logic [NUM_SERVOS-1:0]  pmux_in_i,

    // Interface to evo_i2c_ctrl (Avalon MM Slave)
    input logic [CSR_AWIDTH-1:0]  avs_csr_address,
    input logic                   avs_csr_read, 
    output logic                  avs_csr_waitresponse,
    output logic                  avs_csr_readdatavalid,
    output logic                  avs_csr_waitrequest,
    input logic                   avs_csr_write,
    input logic [CSR_DWIDTH-1:0]  avs_csr_writedata,
    output logic [CSR_DWIDTH-1:0] avs_csr_readdata,
    input logic [4:0]             priv_index,
    input logic                   priv_wr_pw,
    input logic [15:0]            priv_pw
    );                    
   
   localparam SERVO_CTL_ADDR = 12'h0;
   localparam SERVO_PWL_ADDR = 12'h1;
   localparam SERVO_PWH_ADDR = 12'h2;

   // locals
   logic [7:0]                        dbus_adr;
   logic                              dbus_re;
   logic                              dbus_we;
   logic [7:0]                        dbus_data_in;                        
   logic [7:0]                        dbus_data_out;                        

   logic                              csr_sel;
   logic                              csr_re, csr_re_f;
   logic                              csr_we;
   
   // --------------------------------------------------------------------------
   // Type definitions
   // --------------------------------------------------------------------------
   typedef struct packed
                  {
                     logic pwh;
                     logic pwl;
                     logic ctl;
                  } disable_t;
   disable_t      disable_f;

   typedef struct packed
                  {
                     logic [7:0] pwh;
                     logic [7:0] pwl;
                     logic [7:0] ctl;
                  } csr_data_t;
   csr_data_t     csr_data;
   csr_data_t     csr_data_f;
                     
   // --------------------------------------------------------------------------
   // State type definitions
   // --------------------------------------------------------------------------
   // CSR SM
   typedef enum                  logic [2:0]
                                 {CSR_IDLE     = 3'h1,
                                  CSR_PWH      = 3'h2,
                                  CSR_PWL      = 3'h3,
                                  CSR_CTL      = 3'h4,
                                  CSR_RESPONSE = 3'h5
                                  } csr_t;
   
   csr_t                          csr_ns;
   csr_t                          csr_ps_f;

   logic                         csr_idle_st;
   logic                         csr_pwh_st;
   logic                         csr_pwl_st;
   logic                         csr_ctl_st;
   logic                         csr_response_st;

   
   
   // Pinmux internals
   logic [NUM_SERVOS-1:0]             servos_en;
   logic [NUM_SERVOS-1:0]             servos_out;
   
   // PMUX connections ----------------------------------------
   always_comb pmux_en_o[NUM_SERVOS-1:0]  = servos_en[NUM_SERVOS-1:0];
   always_comb pmux_dir_o[NUM_SERVOS-1:0] = {NUM_SERVOS{1'b1}};
   always_comb pmux_out_o[NUM_SERVOS-1:0] = servos_out[NUM_SERVOS-1:0];


   //---------------------------------------------------------------------------
   // CSR and DBUS control
   
   always_comb csr_sel = avs_csr_address[CSR_AWIDTH-1:0] == EVO_SERVO_ADDR;
   always_comb csr_re  = avs_csr_read  && csr_sel;
   always_comb csr_we  = avs_csr_write && csr_sel;

   //---------------------------------------------------------------------------
   // CSR State Machine
   always_ff @(posedge clk or negedge reset_n) begin
      if (!reset_n) csr_ps_f <= CSR_IDLE;
      else csr_ps_f <= csr_ns;
   end
   
   always_comb begin: csr_ns_ctrl
      csr_ns = csr_ps_f;
      unique case (csr_ps_f)
        //----------------------------------------------------------------------
        // IDLE state, all quiet
        //----------------------------------------------------------------------
        CSR_IDLE:     csr_ns = (csr_re || csr_we) ? CSR_PWH : CSR_IDLE;
        CSR_PWH:      csr_ns = CSR_PWL;
        CSR_PWL:      csr_ns = CSR_CTL;
        CSR_CTL:      csr_ns = csr_re_f ? CSR_RESPONSE : CSR_IDLE;
        CSR_RESPONSE: csr_ns = CSR_IDLE;
        default:      csr_ns = CSR_IDLE;
      endcase // unique case (csr_ps_f)
   end // block: csr_ns_ctrl
   // State shorthand (*_st)
   always_comb csr_idle_st     = (csr_ps_f == CSR_IDLE);
   always_comb csr_pwh_st      = (csr_ps_f == CSR_PWH);
   always_comb csr_pwl_st      = (csr_ps_f == CSR_PWL);
   always_comb csr_ctl_st      = (csr_ps_f == CSR_CTL);
   always_comb csr_response_st = (csr_ps_f == CSR_RESPONSE);

   //---------------------------------------------------------------------------
   // DBUS control
   always_comb begin
      // default values
      dbus_adr     = SERVO_CTL_ADDR;
      dbus_re      = 1'b0;
      dbus_we      = 1'b0;
      dbus_data_in = 8'h00;
      // . . . . . . . . . . . . . . . . . . . . . .Access PWH
      if (csr_pwh_st) begin
         dbus_adr     = SERVO_PWH_ADDR;
         dbus_re      = csr_re_f; 
         dbus_we      = !csr_re_f && !disable_f.pwh;
         dbus_data_in = csr_data_f.pwh;
      end
      // . . . . . . . . . . . . . . . . . . . . . .Access PWL
      else if (csr_pwl_st) begin
         dbus_adr     = SERVO_PWL_ADDR;
         dbus_re      = csr_re_f; 
         dbus_we      = !csr_re_f && !disable_f.pwl;
         dbus_data_in = csr_data_f.pwl;
      end
      // . . . . . . . . . . . . . . . . . . . . . .Access CTL
      else if (csr_ctl_st) begin
         dbus_adr     = SERVO_CTL_ADDR;
         dbus_re      = csr_re_f; 
         dbus_we      = !csr_re_f && !disable_f.ctl;
         dbus_data_in = csr_data_f.ctl;
      end
   end

   always_comb csr_data = avs_csr_writedata[23:0];
   always_ff @(posedge clk or negedge reset_n) begin
      // Set csr_re_f at start of read access and hold until response sent
      csr_re_f <=  (!reset_n)                 ? 1'h0 : // Reset
                   (csr_re & csr_idle_st)     ? 1'h1 : // Set
                   (csr_response_st)          ? 1'h0 : // Clear
                                             csr_re_f; // Hold
      // Latch the disable bits every write access
      disable_f <= (!reset_n) ? 3'h0 :                     // Reset
                   (csr_we)   ? avs_csr_writedata[26:24] : // Load
                                disable_f;                 // Hold
      // On Write, capture write data crom avd_csr
      // On Read, capture Servo CSR read data from servo_inst
      csr_data_f.pwh <= (!reset_n)               ? 8'h0 :          // Reset
                        (csr_we)                 ? csr_data.pwh :  // Load CSR
                        (csr_re_f && csr_pwh_st) ? dbus_data_out : // Load DBUS 
                                                   csr_data_f.pwh; // Hold
      csr_data_f.pwl <= (!reset_n)               ? 8'h0 :          // Reset
                        (csr_we)                 ? csr_data.pwl :  // Load CSR
                        (csr_re_f && csr_pwl_st) ? dbus_data_out : // Load DBUS 
                                                   csr_data_f.pwl; // Hold
      csr_data_f.ctl <= (!reset_n)               ? 8'h0 :          // Reset
                        (csr_we)                 ? csr_data.ctl :  // Load CSR
                        (csr_re_f && csr_ctl_st) ? dbus_data_out : // Load DBUS 
                                                   csr_data_f.ctl; // Hold
   end // 

   // Tie off unused outputs
   always_comb avs_csr_waitresponse  = 1'h0;
   always_comb avs_csr_waitrequest   = 1'h0;

   // Delay read data until CSR_RESPONSE state
   always_ff @(posedge clk or negedge reset_n) begin
      if (!reset_n)  begin
         avs_csr_readdatavalid = 1'b0;
         avs_csr_readdata = {CSR_DWIDTH{1'b0}};
      end else if (csr_re_f && csr_response_st) begin //
         avs_csr_readdatavalid = 1'b1;
         avs_csr_readdata = csr_data_f;
      end else begin //
         avs_csr_readdatavalid = 1'b0;
         avs_csr_readdata = {CSR_DWIDTH{1'b0}};
      end
   end // always_ff @ (posedge clk or negedge reset_n)
   
   xlr8_servo 
     #(.NUM_SERVOS             (NUM_SERVOS),
       .SVCR_ADDR              (SERVO_CTL_ADDR),
       .SVPWL_ADDR             (SERVO_PWL_ADDR),
       .SVPWH_ADDR             (SERVO_PWH_ADDR)
       )
   servo_inst 
     (
      // Outputs
      .dbus_out              (dbus_data_out),
      .io_out_en             (),
      .servos_en             (servos_en[NUM_SERVOS-1:0]),
      .servos_out            (servos_out[NUM_SERVOS-1:0]),
      // Inputs
      .clk                   (clk),
      .en1mhz                (en1mhz),
      .rstn                  (reset_n),
      .adr                   (dbus_adr[5:0]),
      .dbus_in               (dbus_data_in[7:0]),
      .iore                  (dbus_re),
      .iowe                  (dbus_we),
      .ramadr                (8'b0),
      .ramre                 (1'b0),
      .ramwe                 (1'b0),
      .dm_sel                (1'b0),
      .priv_wr_pw            (priv_wr_pw),
      .priv_index            (priv_index),
      .priv_pw               (priv_pw)
      );
   

endmodule

