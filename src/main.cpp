#include <mbed.h>
#include <mat.h>
#include <prism_connect.h>
#include <dc_motor.h>
#include <conf.h>
#include <QE.h>
/*
Prism group
software for omni robot project
start dev at 29/10/2017 (DD/MM/YYYY)
If you develop this file , sign your name here.
Dhamdhawach Horsuwan
*/
Serial pc(USBTX, USBRX);

dc_motor wheel1(wheel1_A_pin,wheel1_B_pin,wheel1_pwm_pin);
dc_motor wheel2(wheel2_A_pin,wheel2_B_pin,wheel2_pwm_pin);
dc_motor wheel3(wheel3_A_pin,wheel3_B_pin,wheel3_pwm_pin);

QE wheel1_encoder (PC_3,PC_2);
QE wheel2_encoder (PA_14,PA_13);
QE wheel3_encoder (PC_12,PC_10);


DigitalIn btn(USER_BUTTON);

Ticker display_timer;
void all_drive(float val){
        wheel1.write(val);
        wheel2.write(val);
        wheel3.write(val);
}
void display(){
  pc.printf("%d\t%d\t%d\n",wheel1_encoder.q_state,wheel2_encoder.q_state,wheel3_encoder.q_state);
}
int main() {
        pc.baud(115200);
        display_timer.attach(&display,0.1);


        while(btn.read()==1) ;
        while(1) {
                for(float i = 0.0; i<1.0; i+=0.01) {
                        all_drive(i);
                        wait(0.025);
                }
                wait(0.5);
                for(float i = 1.0; i>0; i-=0.01) {
                        all_drive(i);
                        wait(0.025);
                }
                for(float i = 0.0; i>-1.0; i-=0.01) {
                        all_drive(i);
                        wait(0.025);
                }
                wait(0.5);
                for(float i = -1.0; i<0; i+=0.01) {
                        all_drive(i);
                        wait(0.025);
                }
        }

}
