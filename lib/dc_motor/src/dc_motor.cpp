#include <dc_motor.h>
dc_motor::dc_motor(PinName _pin_A,PinName _pin_B,PinName _pin_E){
        pin_E = new  PwmOut(_pin_E);
        pin_A = new DigitalOut(_pin_A);
        pin_B = new DigitalOut( _pin_B);
        pin_E->period(PWM_frequency);
        pin_A->write(0);
        pin_B->write(0);
}
void dc_motor::write(float _val){
        static float val = 0;
        if(val *_val < 0) {
              pin_E->write(0);
        }
        if(_val >0) {
              pin_A->write(1);
              pin_B->write(0);
        }
        else if(_val <0) {
              pin_A->write(0);
              pin_B->write(1);
        }
        else{
              pin_A->write(0);
              pin_B->write(0);
        }
        val = _val;
        if(val>=0) pin_E->write(val);
        else pin_E->write(-val);

}
