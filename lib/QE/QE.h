#include <mbed.h>
class QE {
private:
  int Q_state_compare(int pre_state,int crr_state);


public:
InterruptIn *enA;
InterruptIn *enB;
DigitalIn   *d_enA;
DigitalIn   *d_enB;
int q_state = 0;
long pos = 0;
QE(PinName pin_A,PinName pin_B);
void get_q_state();

};
