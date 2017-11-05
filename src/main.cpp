#include <mbed.h>

#include <conf.h>

/*
   Prism group
   software for omni robot project
   start dev at 29/10/2017 (DD/MM/YYYY)
   If you develop this file , sign your name here.
   Dhamdhawach Horsuwan
 */
mat omni_mat_init(){
         mat omni (3,3);

         omni.mat_data[0][0] = sin(M_PI/3.0);
         omni.mat_data[0][1] = -cos(M_PI/3.0);
         omni.mat_data[0][2] = -robot_l;

         omni.mat_data[1][0] = 0.0;
         omni.mat_data[1][1] = -cos(M_PI);
         omni.mat_data[1][2] = -robot_l;

         omni.mat_data[2][0] = sin(-M_PI/3.0);
         omni.mat_data[2][1] = -cos(-M_PI/3.0);
         omni.mat_data[2][2] = -robot_l;
         return omni;
 }
float constrain(float val,float upper_lim,float under_lim){
         if (val > upper_lim) return upper_lim;
         else if (val < under_lim) return under_lim;
         else return val;

 }
void reg_changing();
prism_connect prism_comport (&pc,115200,&reg_changing);
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

        wheel1_encoder.pos = 0;
        wheel2_encoder.pos= 0;
        wheel3_encoder.pos= 0;

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
        while(btn.read()==1) ;
        while(1) {
                //motor_test_trianglewave();
        }

}
void reg_changing(){
  static mat omni = omni_mat_init();
  if(prism_comport.read_byte_reg(0)==0) {
          prism_comport.write_float_reg(25,0); //p1
          prism_comport.write_float_reg(29,0); //p2
          prism_comport.write_float_reg(33,0); //p3
          return;
  }
  float crr_x = prism_comport.read_float_reg(1);
  float crr_y = prism_comport.read_float_reg(5);
  float crr_h = prism_comport.read_float_reg(9);

  float goal_x = prism_comport.read_float_reg(13);
  float goal_y = prism_comport.read_float_reg(17);
  float goal_h = prism_comport.read_float_reg(21);

  mat state_error(3,1);
  mat wheel_speed (3,1);
  mat R (3,3);

  state_error.mat_data[0][0] = constrain(0.1*(goal_x - crr_x),2,-2);
  state_error.mat_data[1][0] = constrain(0.1*(goal_y - crr_y),2,-2);
  state_error.mat_data[2][0] = constrain(0.05*(goal_h - crr_h),0.1,-0.1);

  R.set_to_Rz(-crr_h);

  wheel_speed = (omni*R)*state_error;

  prism_comport.write_float_reg(25,wheel_speed.mat_data[0][0]);   //p1
  prism_comport.write_float_reg(29,wheel_speed.mat_data[1][0]);   //p2
  prism_comport.write_float_reg(33,wheel_speed.mat_data[2][0]);   //p3

}
