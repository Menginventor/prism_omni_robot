#include <QE.h>
void QE::get_q_state(){

        if(d_enA->read() == 0) {
                if( d_enB->read() == 0) {
                        q_state = 0;
                }
                else{//B == 1
                        q_state = 3;
                }
        }
        else{//A == 1
                if(d_enB->read() == 0) {
                        q_state = 1;
                }
                else{
                        q_state = 2;
                }
        }

}
QE::QE(PinName pin_A,PinName pin_B){
        /*assign pin*/
        enA = new InterruptIn(pin_A);
        enB = new InterruptIn(pin_B);
        d_enA = new DigitalIn(pin_A);
        d_enB = new DigitalIn(pin_B);
        get_q_state();
        enA->rise(callback(this,&QE::a_rise));
        enB->rise(callback(this,&QE::b_rise));
        enA->fall(callback(this,&QE::a_fall));
        enB->fall(callback(this,&QE::b_fall));


}
void QE::a_rise(){
        get_q_state();
}
void QE::a_fall(){
        get_q_state();
}
void QE::b_rise(){
        get_q_state();
}
void QE::b_fall(){
        get_q_state();
}
