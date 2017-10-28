#include <mbed.h>
class QE {
public:
InterruptIn *enA;
InterruptIn *enB;
DigitalIn   *d_enA;
DigitalIn   *d_enB;
int q_state = 0;
QE(PinName pin_A,PinName pin_B);
void get_q_state();

void a_rise();
void a_fall();

void b_rise();
void b_fall();
};
