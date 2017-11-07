#include <mbed.h>
/*Protocol constant*/
#define read_instruction_code 0x02
#define write_instruction_code 0x03
#define power_addr  0
#define crr_pos_addr  1
#define crr_x_addr 1
#define crr_y_addr  5
#define crr_h_addr  9

#define goal_pos_addr  13
#define goal_x_addr  13
#define goal_y_addr  17
#define goal_h_addr  21

#define p_addr  25
#define p1_addr  25
#define p2_addr  29
#define p3_addr  33


#define data_len_byte 2
#define instruction_byte 3

#define reg_size  64
#define serial_buf_size 128
class prism_connect {


public:

//**:Variable:**//

unsigned char data_reg [reg_size];  //data register to store ,read and send.




/*Serial comunication*/
DigitalOut* led_status = NULL;
void led_status_off();
Timeout led_status_timeout;
Serial*  serial_port;
int serial_buad;
unsigned char serial_buf[128];  //serial buffer refer to max packet length
unsigned char serial_data_sum;   //for checksum
unsigned char serial_buf_index;   //index of serial incoming data for serial buffer.

void (*reg_update)();

/*Serial timeput*/
Timeout serial_timeout_interrupt;   //timer to detect timeout.
float time_char;   //time period for 1-byte (setup in contructor)
float serial_timeout;   //serial timeout for saperate package and detect lost connection (setup  in contructor)
bool receiving;   //use with timeput

/*Data conversion*/
union float_char {  //union for byte - float conversion
        float f;
        char fBuff[sizeof(float)];
};
float b2f (unsigned char B0,unsigned char B1,unsigned char B2,unsigned char B3){  //byte to float conversion
        float_char fc_con;
        fc_con.fBuff[0] = B0;
        fc_con.fBuff[1] = B1;
        fc_con.fBuff[2] = B2;
        fc_con.fBuff[3] = B3;
        return fc_con.f;
}

//**:Method:**//
prism_connect(Serial* port,int _buad,void (*f)());
~prism_connect();

void Rx_interrupt();  //serial receive interupt
void packet_terminate();
bool processing_packet();
void  print_float_reg(unsigned char addr);
float read_float_reg(unsigned char addr);
unsigned char read_byte_reg(unsigned char addr);
void write_float_reg(unsigned char addr,float F);
void send_return_packet(unsigned char status,unsigned char addr,unsigned char len);

};
