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

 mat omni = omni_mat_init();

 float h = 0.001;
 mat mat_R2(float theta);
 mat omni_kinematic(mat &_wheel_speed);
 mat _g_state_dot(mat &_g_state, mat &_relative_state);
 mat runge(mat &_g_state, mat &_relative_state);
 mat g_state(3,1);

 mat mat_R2(float theta){
   float _cos = cos(theta);
  float _sin = sin(theta);
  float R2_arr[2][2] = {{_cos, -_sin}, {_sin, _cos}};
  mat R2(2,2);
  R2.array_copy(&R2_arr[0][0]);
  return R2;
}

mat omni_kinematic(mat &_wheel_speed){
  mat result = omni.inverse() * (_wheel_speed * robot_wheel_r);
  return result;
}

mat _g_state_dot(mat &_g_state, mat &_relative_state){

  mat r_pos_dot(2,1);
  r_pos_dot.mat_data[0][0] = _relative_state.mat_data[0][0];
  r_pos_dot.mat_data[1][0] = _relative_state.mat_data[1][0];
  mat g_pos_dot =   mat_R2(_g_state.mat_data[2][0]) * r_pos_dot;

  mat result(3,1);

  result.mat_data[0][0] = g_pos_dot.mat_data[0][0];
  result.mat_data[1][0] = g_pos_dot.mat_data[1][0];
  result.mat_data[2][0] = _relative_state.mat_data[2][0];

  return result;
}

mat runge(mat &_g_state, mat &_relative_state){
  mat K1 = _g_state_dot(_g_state,_relative_state);
  mat K1_data = _g_state + (K1 * (h/2.0));
  mat K2 = _g_state_dot(K1_data,_relative_state);
  mat K2_data = _g_state + (K2 * (h/2.0));
  mat K3 = _g_state_dot(K2_data,_relative_state);
  mat K3_data = _g_state + (K3 * h);
  mat K4 = _g_state_dot(K3_data,_relative_state);
  mat result = _g_state + ((K1 + (K2 * 2.0) + (K3 * 2.0) + K4) * (h/6.0));
  return result;
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
        prism_comport.led_status = &LED;
        pc.baud(115200);
        g_state.mat_data[0][0] = 0;
        g_state.mat_data[1][0] = 0;
        g_state.mat_data[2][0] = -M_PI/2;
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
  float head_error = goal_h - crr_h;
  if(head_error >M_PI)head_error -= 2*M_PI;
  else if(head_error < -M_PI)head_error += 2*M_PI;
  state_error.mat_data[0][0] = constrain(0.1*(goal_x - crr_x),2,-2);
  state_error.mat_data[1][0] = constrain(0.1*(goal_y - crr_y),2,-2);
  state_error.mat_data[2][0] = constrain(0.05*(head_error),0.1,-0.1);

  R.set_to_Rz(-crr_h);

  wheel_speed = (omni*R)*state_error;

  prism_comport.write_float_reg(25,wheel_speed.mat_data[0][0]);   //p1
  prism_comport.write_float_reg(29,wheel_speed.mat_data[1][0]);   //p2
  prism_comport.write_float_reg(33,wheel_speed.mat_data[2][0]);   //p3

  mat relative_state_dot = omni_kinematic(wheel_speed);
  g_state = g_state + _g_state_dot(g_state,relative_state_dot);
  //g_state = runge(g_state,relative_state_dot);
  prism_comport.write_float_reg(1,g_state.mat_data[0][0]);   //x
  prism_comport.write_float_reg(5,g_state.mat_data[1][0]);   //y
  prism_comport.write_float_reg(9,g_state.mat_data[2][0]);   //theta

}
