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
float display_period = 1.0/100.0;
void all_drive(float val){
        wheel1.write(val);
        wheel2.write(val);
        wheel3.write(val);
}
void display(){
        //pc.printf("%d\t%d\t%d\n",wheel1_encoder.q_state,wheel2_encoder.q_state,wheel3_encoder.q_state);
        pc.printf("%f\t%f\t%f\n",float(wheel1_encoder.pos)/display_period,float(wheel2_encoder.pos)/display_period,float(wheel3_encoder.pos)/display_period);
/*
        wheel1_encoder.pos = 0;
        wheel2_encoder.pos= 0;
        wheel3_encoder.pos= 0;
*/
}
void motor_test_trianglewave(){
        for(float i = 0.0; i<1.0; i+=0.01) {
                all_drive(i);
                wait(0.01);
        }
        //wait(0.5);
        for(float i = 1.0; i>0; i-=0.01) {
                all_drive(i);
                wait(0.01);
        }
        for(float i = 0.0; i>-1.0; i-=0.01) {
                all_drive(i);
                wait(0.01);
        }
        //wait(0.5);
        for(float i = -1.0; i<0; i+=0.01) {
                all_drive(i);
                wait(0.01);
        }
}
void mat_init(){
        mat_debug_port = &pc;
}
int main() {
        pc.baud(115200);
        //display_timer.attach(&display,display_period);
        mat_init();
        mat a(3,3);
        mat b(3,3);
        a.set_to_I();
        b.set_to_I();

        printf("A = \n");
        a.print();
        printf("B = \n");
        b.print();

        while(btn.read()==1) ;
        while(1) {
                motor_test_trianglewave();
        }

}
