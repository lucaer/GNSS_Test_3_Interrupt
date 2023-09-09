// FUNZIONANTE
#include "mbed.h"
#include "XNucleoIKS01A2.h"

#include <stdio.h>
#include <string.h>

#define MSGSIZE 150

void Rx_interrupt();

DigitalOut led(LED1);
DigitalIn btn(USER_BUTTON);
DigitalOut rst(D7); // reset pin for GNSS
DigitalOut wkp(D13); // wake up pin for GNSS
DigitalIn pps(D6); // To start a new timer every time the GNSS provide a signal of new second connected whit RED LED of GNSS

Serial gnss(D8,D2,230400);
Serial pc(USBTX,USBRX,921600);

// To have a correct time step period
Timer tim; // Start each time a new measurement is received
Timer tim2; // Start without resetting



// Instantiate the expansion board
// Define static variable (non cambia during program execution) that point to a our pointer
// D4 & D5 additional Pins, D14 & D15 for I2C
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);


// Retrieve the composing elements of the expansion board (iNEMO) we want to use
//static LSM303AGRMagSensor *magnetometer = mems_expansion_board->magnetometer;
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;
//static LPS22HBSensor *press_temp = mems_expansion_board->pt_sensor;

int16_t xRaw[3], gRaw[3];
//float sX, sG;

char carat;
char buffer[MSGSIZE]; // To collect all the msg
char bufferRX[MSGSIZE]; // To save only the usefull msg
char msg[]="$PSTMPV"; // Header of the message i need

uint8_t i,lenght;
int8_t res;

bool firstTime = true; // First time that i fill the buffer to erase the previouse buffer
bool rcvdMsg = false; // To send the msg at the end of the ISR
uint8_t cnt = 0, cntRx;

// Define the union
union MYFLOAT {
    float time;
    uint8_t byte[4];
} stamp;


union MYINT16 {
    uint16_t data;
    uint8_t byte[2];
} app;



int main(){
    pc.printf("Programma del GNSS \r\n");
    acc_gyro -> enable_x();
    acc_gyro -> enable_g();
    rst.write(1);
    //acc_gyro -> get_x_sensitivity(&sX);
    //acc_gyro -> get_g_sensitivity(&sG);
    // To sincronize with matlab
    while(pc.getc()!='k');
    tim.start(); // Start timer for sample rate
    tim2.start(); // Start timer for acquisition time
    // To know the rising edge of the transmission acquisition time thanks to the GNSS LED
    while(!pps);
    while(pps);
    while(!pps);
    tim2.reset();
    // This is the interrupt routine call
    // Is the link between the function interrupt and the main when RxIrq is received
    // This happen when a new character is received on the serial port
    gnss.attach(&Rx_interrupt, Serial::RxIrq);
    //wkp.write(1);
    while(1){
        tim.reset();
        acc_gyro->get_x_axes_raw(xRaw);
        acc_gyro->get_g_axes_raw(gRaw);
        stamp.time = tim2.read(); // To get the timestamp
        // Each acquisition i get 3 information

        // Sampling Instant
        pc.putc('T');
        for(int i=0;i<4;i++){
            pc.putc(stamp.byte[i]);
        }

        // Three value for acceleration x,y,z
        pc.putc('A');
        for(int i=0;i<3;i++){
            app.data = xRaw[i];
            pc.putc(app.byte[0]);
            pc.putc(app.byte[1]);
        }

        // Three value for angular rate 
        pc.putc('W');
        for(int i=0;i<3;i++){
            app.data = gRaw[i];
            pc.putc(app.byte[0]);
            pc.putc(app.byte[1]);
        }

        // Chech if there is a GNSS msg
        if(rcvdMsg){
            for(int i=0; i<cntRx; i++){
                pc.putc(bufferRX[i]);
            }
            rcvdMsg = false;
        }

        // Because of the delay for a new sampling
        double time = tim.read();
        if(time<0.01){
            wait(0.01 - time);
        }
    }
return 0;
}

void Rx_interrupt(){
    led = 1;
    // If first time i execute the ISR i have to erase the buffer
    if (firstTime){
        memset(buffer,0,MSGSIZE); // Set the value of 0 to all the byte
        firstTime = false;
    }
    carat = gnss.getc(); // Take a character from GNSS
    // If is different from Line Feed, put it in the buffer and this char is '$'
    if(carat != 10){
        buffer[cnt++] = carat;
        // Else put it in buffer as last value and compare the content of buffer
        // With the header i'm interested in
        // If the result is correct
    }else{
        buffer[cnt] = carat;
        res = memcmp(buffer,msg,sizeof(msg));
        if(res == 1){
            rcvdMsg = true;
            // Copy the received buffer into bufferRX which is the one to go to the pc
            memcpy(bufferRX,buffer,cnt+1);
            cntRx = cnt+1; // Save the value of counter
        }
        firstTime = true;
        cnt = 0;
    }
    led = 0;
    return;
};