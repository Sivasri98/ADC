/*
 * sogi_pll.c
 *
 *  Created on: 30-May-2025
 *      Author: VOLTECH
 */

#ifndef SOGIPLL_C_
#define SOGIPLL_C_

void sogi_pll()
{
    //========== SOGI Core ==================
    x1 = (vin_pll - v1_out_pre) - v2_out_pre;

    //==== Trapezoidal integration (SOGI)
    v1_out = Ts_omega * (x1 + x1_pre) * 0.5 + v1_out_pre;
    v2_out = Ts_omega * (v1_out + v1_out_pre) * 0.5 + v2_out_pre;

    v_alpha = v1_out;     // Check sign here (flip if needed)

    //=============High Pass Filter DC Offset Removal========
    y1 = alpha_lpf_1 * ((vin_pll - v1_out) + v_lpf_pre) + alpha_lpf_2 * y1_pre;

    v_lpf_pre = (vin_pll - v1_out);

    y1_pre = y1;

    switch (lpf)
    {
    case 0:

        v_beta = v2_out - y1;      // Ensure this lags v_alpha by 90°
        break;
    case 1:
        v_beta = v2_out;      // Ensure this lags v_alpha by 90°
        break;
    }
    //========== Park Transform ==============

    vd = v_alpha * cos_theta + v_beta * sin_theta;
    vq = -v_alpha * sin_theta + v_beta * cos_theta;

    vq_act = vq;
    //========== PI Controller ===============
    I_pll += Ki_pll * dt * vq_act;

    if (I_pll > I_pll_max)
        I_pll = I_pll_max;
    if (I_pll < I_pll_min)
        I_pll = I_pll_min;

    PI_pll = Kp_pll * vq_act + I_pll;

    //========== Frequency & Phase Update ====
    omega = omega_nominal + PI_pll;
    theta += omega * dt;

    // Wrap theta to [0, 2pi)
    theta = fmod(theta, 2 * M_PI);  // Wrap phases
    if (theta > 2 * M_PI)
        theta -= 2 * M_PI;
    if (theta < 0)
        theta += 2 * M_PI;

    cos_theta = cos(theta);
    sin_theta = sin(theta);
    Ts_omega = omega * dt;

    vd_cos = vd * cos_theta - vq * sin_theta;
    vd_sin = vd * sin_theta + vq * cos_theta;
    //========== Update Previous Values ======
    x1_pre = x1;
    v1_out_pre = v1_out;
    v2_out_pre = v2_out;
}
//============PLL_END=======================
void sogi_pll_duty()
{
    //========== SOGI Core ==================
    duty_err = kr_d * (duty_in - duty_out_alpha_pre) - duty_out_beta_pre;

    //==== Trapezoidal integration (SOGI)
    duty_out_alpha = Ts_omega_pre * (duty_err + duty_err_pre) * 0.5
            + duty_out_alpha_pre;
    duty_out_beta = Ts_omega_pre * (duty_out_alpha + duty_out_alpha_pre) * 0.5
            + duty_out_beta_pre;

    duty_out = kp_r * duty_in + kr_r * duty_out_alpha;

    //========== Update Previous Values ======
    duty_err_pre = duty_err;
    duty_out_alpha_pre = duty_out_alpha;
    duty_out_beta_pre = duty_out_beta;
    Ts_omega_pre = Ts_omega;
}
void sogi_pll_IL()
{
    //========== SOGI Core ==================
    IL_err = k_il * (I_L_in - IL_out_alpha_pre) - IL_out_beta_pre;

    //==== Trapezoidal integration (SOGI)
    IL_out_alpha = Ts_omega_pre * (IL_err + IL_err_pre) * 0.5
            + IL_out_alpha_pre;
    IL_out_beta = Ts_omega_pre * (IL_out_alpha + IL_out_alpha_pre) * 0.5
            + IL_out_beta_pre;

    I_L = IL_out_alpha;

    //========== Update Previous Values ======
    IL_err_pre = IL_err;
    IL_out_alpha_pre = IL_out_alpha;
    IL_out_beta_pre = IL_out_beta;

}
void low_pass_duty()
{

    if (duty_lpf_flg1 == 1)
    {
        duty = 0.2066 * duty_lpf + 0.4131 * duty_a + 0.2066 * duty_b
                - (-0.3695) * duty_out_a - 0.1958 * duty_out_b;
        duty_b = duty_a;
        duty_a = duty_lpf;
        duty_out_b = duty_out_a;
        duty_out_a = duty;
    }

    if ((duty < 1.1 * POSITIVE_THRESHOLD_HIGH)
            && (duty > 1.1 * NEGATIVE_THRESHOLD_HIGH))
    {

        duty = 0.000314061 * duty_lpf - 0.00031406 * duty_b
                + 1.999330534 * duty_out_a - 0.99937094 * duty_out_b;
        duty_b = duty_a;
        duty_a = duty_lpf;
        duty_out_b = duty_out_a;
        duty_out_a = duty;
    }

//duty_temp1= mul_fact1* 0.2066*duty_lpf +mul_fact1*0.4131*duty12 +mul_fact1*0.2066*duty22 +0.3695*duty_out12- 0.1958*duty_out22;
//
//duty22 = duty12;
//duty12 =duty_lpf ;
//duty_out22 =duty_out12;
//duty_out12=duty_temp1;

}
#endif /* SOGIPLL_C_ */

