/*
 * kalman_filter.c
 *
 *  Created on: Sep 30, 2024
 *      Author: mskwok
 */

#include "kalman_filter.h"

uint16_t kalman_filter(uint16_t adc_value)
{
	// https://blog.embeddedexpert.io/?p=791

    float x_k1_k1, x_k_k1;
    static float adc_old_value;
    float Z_k;
    static float P_k1_k1;

    // Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse
    // R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better
    static float Q = 0.001; // 0.0001;
    static float R = 0.005; // 0.005;
    static float Kg = 0;
    static float P_k_k1 = 1;

    float kalman_adc;
    static float kalman_adc_old = 0;
    Z_k = adc_value;
    x_k1_k1 = kalman_adc_old;

    x_k_k1 = x_k1_k1;
    P_k_k1 = P_k1_k1 + Q;

    Kg = P_k_k1 / (P_k_k1 + R);

    kalman_adc = x_k_k1 + Kg * (Z_k - kalman_adc_old);
    P_k1_k1 = (1 - Kg) * P_k_k1;
    P_k_k1 = P_k1_k1;

    adc_old_value = adc_value;
    kalman_adc_old = kalman_adc;

    return kalman_adc;
}
