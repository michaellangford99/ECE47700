#include "stm32f4xx.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "system.h"
#include "pwm.h"
#include "motors.h"

//TODO
/*

init motors

update_safety
    pass in pertinent radio axes
    convert from radio range to 0-65535


set motors

convert motor range into range usuable by pwm code

(uint16_t)((((uint32_t)65535)*(uint32_t)(RX_USART_get_channels()->ch2 - CRSF_CHANNEL_VALUE_MIN))/(uint32_t)(CRSF_CHANNEL_VALUE_MAX-CRSF_CHANNEL_VALUE_MIN));

*/
/*
void update_motors()
{
    float motor_output[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    motor_output[0] = rx_throttle;
    motor_output[1] = rx_throttle;
    motor_output[2] = rx_throttle;
    motor_output[3] = rx_throttle;

    if (rx_throttle > 0.05f)
    {

        motor_output[0] += yaw_pid_response;
        motor_output[1] += -yaw_pid_response;
        motor_output[2] += -yaw_pid_response;
        motor_output[3] += yaw_pid_response;

        motor_output[0] += pitch_pid_response;
        motor_output[1] += pitch_pid_response;
        motor_output[2] += -pitch_pid_response;
        motor_output[3] += -pitch_pid_response;

        motor_output[0] += -roll_pid_response;
        motor_output[1] += roll_pid_response;
        motor_output[2] += -roll_pid_response;
        motor_output[3] += roll_pid_response;

    }
    else
        gyro_angle_z = 0;

    //clamp motor values

    motor_output[0] = max(0.0f, motor_output[0]);
    motor_output[1] = max(0.0f, motor_output[1]);
    motor_output[2] = max(0.0f, motor_output[2]);
    motor_output[3] = max(0.0f, motor_output[3]);

    motor_output[0] = min(1.0f, motor_output[0]);
    motor_output[1] = min(1.0f, motor_output[1]);
    motor_output[2] = min(1.0f, motor_output[2]);
    motor_output[3] = min(1.0f, motor_output[3]);

    float filtered_motor_output[4];

    for (int i = 0; i < 4; i++)
    {
        filtered_motor_output[i] = update_filter(&(motor_filter[i]), motor_output[i]);
    }

    pwm_output_t pwm_output;

    pwm_output.duty_cycle_ch0 = (uint16_t)(filtered_motor_output[0]*65535.0f);
    pwm_output.duty_cycle_ch1 = (uint16_t)(filtered_motor_output[1]*65535.0f);
    pwm_output.duty_cycle_ch2 = (uint16_t)(filtered_motor_output[2]*65535.0f);
    pwm_output.duty_cycle_ch3 = (uint16_t)(filtered_motor_output[3]*65535.0f);

    set_PWM_duty_cycle(pwm_output);
}*/