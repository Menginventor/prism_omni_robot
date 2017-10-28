#include <mbed.h>

#include <conf.h>

/*
   Prism group
   software for omni robot project
   start dev at 29/10/2017 (DD/MM/YYYY)
   If you develop this file , sign your name here.
   Dhamdhawach Horsuwan
 */


Ticker display_timer;
float display_period = (1.0/10.0);
void all_drive(float val){
        wheel1.write(val);
        wheel2.write(val);
        wheel3.write(val);
}
void display(){
        //pc.printf("%d\t%d\t%d\n",wheel1_encoder.q_state,wheel2_encoder.q_state,wheel3_encoder.q_state);
        //pc.printf("%d\t%d\t%d\n",wheel1_encoder.pos,wheel2_encoder.pos,wheel3_encoder.pos);
        pc.printf("%f\t%f\n",(float(wheel1_encoder.pos)/display_period )/1000.0,wheel1_encoder.speed/1000.0);
        wheel1_encoder.pos = 0;
        wheel2_encoder.pos= 0;
        wheel3_encoder.pos= 0;

}
int main() {
        pc.baud(115200);
        display_timer.attach(&display,display_period );


        while(btn.read()==1) ;
        all_drive(0.1);
        while(1) {
/*
                for(float i = 0.0; i<1.0; i+=0.01) {
                        all_drive(i);
                        wait(0.01);
                }
                wait(0.5);
                for(float i = 1.0; i>0; i-=0.01) {
                        all_drive(i);
                        wait(0.01);
                }
                for(float i = 0.0; i>-1.0; i-=0.01) {
                        all_drive(i);
                        wait(0.01);
                }
                wait(0.5);
                for(float i = -1.0; i<0; i+=0.01) {
                        all_drive(i);
                        wait(0.01);
                }
*/
        }

}
