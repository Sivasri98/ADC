/*
 * Ac.h
 *
 *  Created on: 18-Nov-2024
 *      Author: Admin
 */

#ifndef AC_H_
#define AC_H_

#include <stdlib.h>
#include "F28x_Project.h"
#include "math.h"

void calculate_vout_lpf(void);
void calculate_i_in_lpf(void);
void calculate_vin_lpf(void);
void  init_dac(void);
void dac_out(void);

#define PWM_En  GpioDataRegs.GPDCLEAR.bit.GPIO112=1;
#define PWM_Dis GpioDataRegs.GPDSET.bit.GPIO112=1;
#define M_PI        3.14159265358979323846  /* pi */
#define M_SQRT2     1.41421356237309504880  /* sqrt(2) */
char mode=0;


Uint16  duty_1=1,duty_2=2,duty_3=3,duty_4=5,duty_5=7,zss_duty_min=380;

float DAC_A0=0,DAC_B0=1,DAC_C0=0;
char dq_o=1;

float va_mul_factor=0.000724007085;

float va_mul_factor_32G=123.74650386886600,ia_mul_factor_32G=4.049372238522480,vo_mul_factor_32G=0.0461999646876;


float v_temp_1g[3],v_temp_32g[3],v_1g[3],v_32g[3];

Uint16 filter=0;

float kp_i=0.11,ki_i=15.7,P_i=0,I_i=0,ki_intgl_min=-5,ki_intgl_max=5,dt=0.00002,PI_i=0,I_Err=0;
float v_kp=25,v_ki=10,v_p=0,v_i=0,v_intgl_min=-5,v_intgl_max=5,v_pi_v=0,v_err=0;
float I_vv[10],kv_m_ftr=15;
char I_loop=0,condition=0;
float f_cor=1000;
float V_Err_pre=0,V_Err_wdng_ftr=0,err_limit=0.0;

Uint16 epwm5_period_value = 3999, sampling_samples_count = 32, i_max=0;
float duty=0,duty_min = 0,duty_max = 1,duty_ctrl=0.75,angle=0;

float v_in_alpha_1=1.62080002,v_in_alpha_2=1.6375,v_in_beta_1=720,v_in_beta_2=192.3076923;
float v_out_alpha_1=1.602,v_out_alpha_2=1.5931,v_out_beta_1=769.2307692,v_out_beta_2=196.0784314;
float i_in_alpha_1=1.61861897,i_in_alpha_2=1.625,i_in_beta_1=79.3650818,i_in_beta_2=40.0,i_in_alpha_3=1.62349999,i_in_beta_3=40.0;
float I_Sett=3,v_set=415, I_Set=0,f_f_gain=1,duty_m_f=0,I_Set_max=22;

float x0_1g[3],y0_1g[3];
float x0_32g[3],y0_32g[3];
float x1_1g[3],y1_1g[3];
float x1_32g[3],y1_32g[3];
float x2_1g[3],y2_1g[3];
float x2_32g[3],y2_32g[3];

float v_out,v_out_temp1=0,v_out_temp=0,v_out_in=0;

Uint16  epwm1_period_value = 3999,Vout_temp_cnt=0,epwm2_period_value=62499;
char test_case=3,vo_filt=0;
Uint16 epwm1_duty_cycle_a = 1999,epwm1_duty_cycle_b = 1999,epwm2_duty=31250,Single_phase_deadband_1=200,Single_phase_deadband_2=200;
Uint16 high_frq_delay=2,delay_cnt=0;
Uint16 Sample_count=0;


//==============PLL=========================
float v_alpha=0, v_beta=0,vd=0,vq=0,theta_out=0,omega=0.0f,omega_nominal=314.159265358979323846f,Ts_omega=0.006283185,Kp_pll=0.566f,Ki_pll=17.3579f,I_pll=0;
float cos_theta=0,sin_theta=0,vd_sin=0,vd_cos=0,Ts_omega_pre=0.006283185;
float v1_out=0, v2_out=0, v1_out_pre=0.0f,v1_out_pre_pre=0, v2_out_pre=0.0f, x1_pre=0.0f, theta=0, vq_old=0, vq_act=0,x1=0,vq_filtered=0.0f;
char pll_cnt,grid_ref=0;
float vin_pll=0,PI_pll=0,I_pll_max=628.0f,I_pll_min=-628.0,v_max=325,vout_pll=0,vin_pll32=0,vin_pll1=0;
float v2_filtered=0.0f;
#define PI_max 100.0f  // Adjust based on your system
float alpha_lpf_1=6.28279E-05,alpha_lpf_2=0.999874344,omega_t=6.28319E-05;
float y1_pre=0,y1_pre_pre=0,y1=0,v_lpf_pre=0,v_lpf_pre_pre=0;
char lpf=0,lpf_order=0,soft_start=0,pll_duty=9;
float  I_in=0,I_in_pre=0,I_L_pre=0,alpha2=0.00100000005,alpha22=0.881;
Uint16 delay=0;

//===============PFC====================================
//===Duty Hysterisis====================

//float HYSTERESIS = 0.008;         //  % Hysteresis band ( 0.02)
Uint16  fc_fundamental = 50;      //    % Fundamental frequency (Hz)
Uint16 fc_ripple = 12500;         //   % Switching ripple frequency (Hz)
Uint16  fs = 50000;               //    % Sampling frequency (Hz)




float  alpha_fundamental=0.993736513, alpha_ripple=0.207879576,duty_temp,duty_pree=0,mul_fact=3,duty_temp_pre=0,mul_fact1=500,duty_temp1=0;

float duty_filtered=0, duty_intermediate=0, pos_threshold=0, neg_threshold=0;


char hyst_cnt_flg=0,hyst_cnt=0,hyst_cnt_set=50,db_flag=0,p_trans_flg=0,n_trans_flg=0;
char duty_temp_flg=1,duty_state=1;
float duty_temp_max=0,duty_temp_min=0,n_duty=0,p_duty=0;
char duty_temp_cnt_set=3,p_duty_temp_cnt=0,duty_temp_cnt=0,p_flag_pre=0,n_flag_pre=0,duty_temp_cnt_flg=0;
Uint16 dead_band_cnt=0,dead_band_cnt_set=3;
//===================END====END==============================================

//===================duty_pll=======================================
float  duty_err=0,duty_err_pre=0,duty_out_alpha=0,duty_out_alpha_pre=0,duty_out_beta=0,
        duty_out_beta_pre=0,duty_in=0,kr_d=0.8,duty_out=0,kp_r=0.08,kr_r=0;
char pr_control_flg=1,v_control=1,duty_lpf_flg=0,duty_cnt_flg=500, db_flag_h=0;
float duty1=0.01,duty2=0,duty_out1=0,duty_out2=0,duty_lpf=0;
float duty12=0,duty22=0,duty_out12=0,duty_out22=0,b_delay=0.05;
Uint16 p_db_cnt=0,n_db_cnt=0,p_db_cnt_set=6,n_db_cnt_set=6,p_on_cnt_flag=0,n_on_cnt_flag=0;
float sin_degree_2_set=0.5,sin_degree_3_set=0.5,sin_degree_2_max=50,sin_degree_3_max=50;
char off_flg3=0,off_flg2=0,off_flg1=0,on_flg1=0,on_flg2=0,on_flg3=0,duty_flg=0;

float POSITIVE_THRESHOLD_HIGH=     3 ;   // Higher threshold for positive (definitely in positive region)
float POSITIVE_THRESHOLD_LOW=      0.01256 ;   // Lower threshold for positive (entry to deadband)
float NEGATIVE_THRESHOLD_HIGH=    -3;   // Higher threshold for negative (definitely in negative region)
float NEGATIVE_THRESHOLD_LOW=     -0.01256 ;   // Lower threshold for negative (entry to deadband)
float DEADBAND_COUNT_SET=        6 ;    // Number of consecutive detections required
int16 HYSTERESIS = 5;
float previous_readings[2]={0,0},duty_a=0,duty_b=0,duty_out_a=0,duty_out_b=0;
char duty_lpf_flg1=0,duty_mvg=0;

//======================end=============================================
//====================IL_PLL================
float IL_err=0,IL_err_pre=0,k_il=1,
              IL_out_beta=0,IL_out_alpha=0,IL_out_alpha_pre=0,IL_out_beta_pre=0;
// % Normalized coefficients for 10kHz cutoff at 50kHz sampling
float b0 = 0.067455,b1 = 0.134911,b2 = 0.067455,a1 = -1.142980,a2 = 0.412802;

//===================End======================================

Uint16 zcd_cnt=0,zcd_hi_flag=0;
char   zcd_flag=0,P_half_on_flg=0,N_half_on_flg=0;
float  duty_db=0,I_L=0,Pll_sin=0,Pll_sin_pre=0,v_in=0,Pll_sin_db=0,f_f=0,duty_db_n=0,db_pll=0;

float duty_pre=0,sin_degree=0.001745328,sin_degree_2=3.0f,sin_degree_3=-4.0f,sin_degree_4=2.0f,sin_degree_5=-2.0f,alpha=0.1;
char duty_db_cnt=0,duty_db_flg=1,p_duty_db_flg=1,n_duty_db_flg=0,p_flag=0,n_flag=0;
char p_on_flag=0,n_on_flag=1;
float cos_theta_pre=0;
int16 sin_degree1=3;
Uint16 zcd_count=0,falling_transition_cnt=0,duty_cnt=0;
float d_pll_out1=0.01;
char p_raising_transition=0,n_raising_transition=0,n_falling_transition=0,p_falling_transition=0,start=0,raising_transition_cnt=0;
float raising_transition_cnt_set2=0.00012,raising_transition_cnt_set1=10;
//================

/*Sampling freq and switching freq automation variable*/
Uint32 epwm1_switching_freq = 25000 ,epwm5_sampling_freq = 50000;
Uint32 epwm5_sampling_sample_count=0,epwm5_sampling_count=0;
float sin_a=0,sin_a_pre=0,m_index=0.8125;
char Leg_a=0, Leg_b=0,Pwm_fault_flg=0,set_p_cycle=0,set_n_cycle=0;

//====================calibrION===========
char cal_flag=0,cal_flag_cnt=0;
float I_L_1g_offset=0, I_L_32g_offset=0;
//=======================================

// ====== Hysteresis Threshold Definitions ======
float  DEADBAND_HYSTERESIS=0.05f;   // Adjust based on system noise/requirements
float deadband_enter_threshold = 0;          // Default threshold to enter dead-band
float deadband_exit_threshold  = 0; // Must exceed this to exit dead-band
float DEADBAND_HYSTERIS=0.05f;

//==================Protection=============
char fault_flag=0,IL_filt=0;
float I_L_set=22,v_out_set=500;
float alpha3=0.754762725, alpha4=0.509525449,I_L_in=0,I_L_in_pre=0;
float I_L_in2=0,I_L_in1=0,I_L_out2=0,I_L_out1=0,I_L_out=0;

/*Function Declaration*/
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm5_isr(void);

__interrupt void adca0_isr(void);
__interrupt void adca3_isr(void);
__interrupt void adca4_isr(void);
__interrupt void adcc2_isr(void);

__interrupt void cpu_timer0_isr(void);

void initADCA();
void initEPwm5();
void initEPwm1();
//void initEPwm2();

#endif /* AC_H_ */
