#include <prism_connect.h>

prism_connect::prism_connect(Serial* port,int _buad,void (*f)()){
/*variable setup*/
        serial_buad = 0;

        serial_data_sum = 0;//for checksum
        serial_buf_index = 0;//index of serial incoming data for serial buffer.
        time_char = 0;//time period for 1-byte (setup in contructor)
        serial_timeout = 0;//serial timeout for saperate package and detect lost connection (setup  in contructor)
        receiving = false;//use with timeput
/**/
        serial_port = port;


        serial_buad =  _buad;
        time_char = 10.0/float(serial_buad);//10 from 8 databits 1 start bit,1 stopbit.
        //serial_timeout = 3.0*time_char;
        serial_timeout = 0.010;
        serial_port->attach(callback(this,&prism_connect::Rx_interrupt), Serial::RxIrq);
        reg_update = f;
}
prism_connect::~prism_connect(){
        delete serial_port;
}
void prism_connect::Rx_interrupt(){

        static unsigned char packet_len = 0;
        unsigned char data_in = serial_port->getc();
        //debug -> printf("%02x " ,data_in);
        if(serial_buf_index>serial_buf_size&&debug != NULL)  serial_buf_index = 0;
        if(!receiving) {  //first receiving
                //  if(debug != NULL)debug -> printf(".");
                serial_buf_index = 0;
                serial_data_sum = 0;


                if(data_in == 0xFF) {
                        receiving = true;
                }else{
                        return;
                }
        }


        serial_buf[serial_buf_index] = data_in;


        if(serial_buf_index >= 1 && serial_buf[0] != 0xFF && serial_buf[1] != 0xFF) {
                receiving = false;
                return;
        }

        if(serial_buf_index == 2 ) {
                packet_len = serial_buf[2];
                if(serial_buf[2] == 0xFF){
                    receiving = false;
                  return;
                }
        }

        if(serial_buf_index >= 2 &&  serial_buf_index < packet_len + 2) {
                serial_data_sum += serial_buf[serial_buf_index];
        }
        else if (serial_buf_index >= 2 &&  serial_buf_index >= packet_len + 2) {
                //serial_port->putc(serial_buf[serial_buf_index]);

                serial_buf_index++;
                serial_timeout_interrupt.attach(callback(this,&prism_connect::packet_terminate),0);

                return;

        }

        serial_buf_index++;
        serial_timeout_interrupt.attach(callback(this,&prism_connect::packet_terminate),serial_timeout);
}
void prism_connect::packet_terminate(){
        receiving = false;
        if(!processing_packet()) {
                if(debug != NULL)debug -> printf("processing_packet error!\n");
                send_return_packet(1,0,0);
        }
}

bool prism_connect::processing_packet(){
        unsigned char datalength = serial_buf_index;
        /*
               0     1      2         3           4           3+N       4+N
            [0xFF][0xFF][Length][Instruction]	[PARAM 1]... [PARAM N][Checksum]
         */
        if(!(serial_buf[0] == 0xFF && serial_buf[1] == 0xFF)) {  //check header
            if(debug != NULL)debug -> printf("processing_packet header error!\n");
                return false;
        }
        serial_data_sum = ~serial_data_sum;
        if(serial_buf[datalength-1] != serial_data_sum) {  //check sum
            if(debug != NULL)debug -> printf("processing_packet checksum error!\n");
                //serial_port->printf("checksum error\n");
                return false;
        }

        int length = serial_buf[data_len_byte];
        //serial_port->printf("length = %d , ",length);

        if(serial_buf[instruction_byte] == read_instruction_code) {

                /*serial_port->printf("found read command , ");
                   serial_port->printf("start read @ addr %d  , ",serial_buf[4]);
                   serial_port->printf("read %d reg  , ",serial_buf[5]);*/
                /*for(int i = 0; i<serial_buf[5]; i+=4) {
                      serial_port->printf("read  reg [%d] = %f , ",i+serial_buf[4], read_float_reg(i+serial_buf[4]));
                   }*/
                if(reg_update != NULL){
                  if(debug != NULL)debug -> printf("got packet!\n");
                  reg_update();

                  if(led_status != NULL){
                    led_status->write(1);
                    led_status_timeout. attach(callback(this,&prism_connect::led_status_off),0.1);


                  }

                }
                send_return_packet(0,serial_buf[4],serial_buf[5]);

        }
        else if(serial_buf[instruction_byte] == write_instruction_code) {
                //serial_port->printf("found write command , ");
                //serial_port->printf("start write @ addr %d  , ",serial_buf[4]);
                //serial_port->printf("write %d reg  , ",length-3);
                for(int i = 0; i<length-3; i++) {
                        data_reg[i+serial_buf[4]] = serial_buf[5+i];
                        //  serial_port->printf("write  reg %d , ",i+serial_buf[4]);
                }

                //print_float_reg(serial_buf[4]);
                //print_float_reg(serial_buf[4]+4);
                send_return_packet(0,0,0);
        }
        return true;
}
float prism_connect::read_float_reg(unsigned char addr){

        return b2f(data_reg[addr],data_reg[addr+1],data_reg[addr+2],data_reg[addr+3]);

}
void prism_connect::print_float_reg(unsigned char addr){

        serial_port->printf("reg [%d] = %f\n",addr, read_float_reg(addr));

}
void prism_connect::write_float_reg(unsigned char addr,float F){
        float_char fc_con;
        fc_con.f = F;
        for(int i = 0; i < 4; i++) {
                data_reg[i+addr] = fc_con.fBuff[i];
        }

}
void prism_connect::send_return_packet(unsigned char status,unsigned char addr,unsigned char len){
        unsigned char p_len = len+2;
        unsigned char checksum = 0;
        checksum += p_len;
        //serial_port->printf("\nchecksum += %02x , checksum = %02x\n",p_len,checksum );
        checksum += status;
        //serial_port->printf("checksum += %02x , checksum = %02x\n",status,checksum );
        serial_port->putc(0xFF);
        serial_port->putc(0xFF);
        serial_port->putc( p_len);
        serial_port->putc( status);
        for(int i = 0; i<len; i++) {
                checksum += data_reg[i+addr];
                //serial_port->printf("checksum += %02x , checksum = %02x\n",data_reg[i+addr],checksum );
                serial_port->putc(data_reg[i+addr]);

        }
        checksum = ~checksum;
        //serial_port->printf("~checksum = %02x\n",checksum );
        serial_port->putc(checksum);

}

unsigned char prism_connect::read_byte_reg(unsigned char addr){
      return data_reg[addr];
}
void prism_connect::led_status_off(){
  if(led_status != NULL){
      led_status->write(0);

  }
}
