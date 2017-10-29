#include <mbed.h>

#include <mat.h>
#include <prism_connect.h>
#include <dc_motor.h>
#include <QE.h>
/*use for robot configure*/
#define wheel1_pwm_pin PB_13
#define wheel1_A_pin PB_14
#define wheel1_B_pin PB_15

#define wheel2_pwm_pin PB_5
#define wheel2_A_pin PB_3
#define wheel2_B_pin PA_10

#define wheel3_pwm_pin PB_10
#define wheel3_A_pin PA_8
#define wheel3_B_pin PA_9

Serial pc(USBTX, USBRX);

dc_motor wheel1(wheel1_A_pin,wheel1_B_pin,wheel1_pwm_pin);
dc_motor wheel2(wheel2_A_pin,wheel2_B_pin,wheel2_pwm_pin);
dc_motor wheel3(wheel3_A_pin,wheel3_B_pin,wheel3_pwm_pin);

QE wheel1_encoder (PC_3,PC_2);
QE wheel2_encoder (PA_14,PA_13);
QE wheel3_encoder (PC_12,PC_10);

DigitalIn btn(USER_BUTTON);
