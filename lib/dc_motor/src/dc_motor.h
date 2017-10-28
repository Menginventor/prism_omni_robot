#include <mbed.h>
#define PWM_frequency 1.0/20000.0
class dc_motor{
public:
  PwmOut* pin_E;
  DigitalOut* pin_A;
  DigitalOut* pin_B;
  dc_motor(PinName _pin_A,PinName _pin_B,PinName _pin_E);
  void write(float val);//val -1 to 1
};
