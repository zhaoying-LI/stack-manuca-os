/**
@file main.cpp
 
@brief Pedometer implementation using the sensor Triple Axis Accelerometer Breakout - MMA8452Q
 
*/
/**
@file main.h
@brief Header file containing functions prototypes and global variables.
@brief Implementation of a Pedometer using the accelerometer MMA8452Q, Nokia 5110 display and the mbed LPC1768.
@brief Revision 1.5.
@author Edson Manoel da Silva
@date   May 2015
*/

#include "mbed.h"
#include <queue> 
Serial sd(USBTX, USBRX, 115200);
#define SQRT_MAGIC_F 0x5f3759df 
#define  iterationNumber 16
//#define iterationNumber 1 // for spi printf test

typedef struct Acceleration Acceleration;
struct Acceleration {
    int x;
    int y;
    int z;
};

const PinName spi_MOSI = PinName::PF_9;
const PinName spi_MISO = PinName::PF_8;
const PinName spi_CLK = PinName::PF_7;
const PinName spi_CS = PinName::PB_11;

SPI spi(spi_MOSI, spi_MISO, spi_CLK); 
DigitalOut cs(spi_CS);


Acceleration acceleration;  

/**  
@namespace mma8452
@brief Accleration average structure declared in MMA8452 class
*/
Acceleration acc_avg;

unsigned char second = 0; /*!< second flag set in TimerExpired3 */
unsigned char minute = 0; /*!< minute flag set in TimerExpired3 */
unsigned char hour = 0;/*!< hour flag set in TimerExpired3 */
unsigned char state = 0;/*!< state variable for the FSM */
unsigned char I1_flag = 0;/*!< Interrupt flag set in Transient Detection Interrupt */
unsigned char I2_flag = 0;/*!< Interrupt flag set in Pulse(Tap) Detection Interrupt */
unsigned char timerFlag1 = 0;/*!< Interrupt flag set in Timer1 */
unsigned char timerFlag2 = 0;/*!< Interrupt flag set in Timer2 */
unsigned char aux=0;/*!< Auxiliar for checking if the user stopped using the device */

char Int_SourceSystem =0;/*!< Variable used to read the MMA8452Q Interrupt Source Register */
char Int_SourceTrans=0;/*!< Variable used to clear the MMA8452Q Interrupt Registers */

unsigned char length;/*!< Variable used to check the string length to be printed in the LCD */
char buffer[14];/*!< Buffer used for printing strings on the display */

 int step[iterationNumber]={};/*!< Variable used to ccalculate the steps */
//int step = 0;
// int km = 0;/*!< Variable used to ccalculate the kilometers */
int km[iterationNumber]={};
//int km = 0;
//int km_day[30] = {0,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6,7,8,1,2,3,4,5,6};
//int km_day[30] = {};/*!< Variable used to ccalculate the kilometers per day */

int acc_vector[iterationNumber]={};/*!< Variable for check if a step was performed */
//int acc_vector = 0;
//int sub_x = 0;/*!< Variable used for calibration */
//int sub_y = 0;!< Variable used for calibration 
//int sub_z = 0;/*!< Variable used for calibration */
int i;
int sub_x;
int sub_y;
int sub_z;
int i_4I;
int i_4Iplus2;

//short bank_shift = 2048;
short bank_shift = 4096 + 16384; // 4096 is the global offset of Hycube data memory; 16384 = 2^14, indicating first 01 for spi 


short acceleration_x_base_address = 256+bank_shift;
short acceleration_y_base_address = 128+bank_shift;
short acceleration_z_base_address = 64+bank_shift;
short acc_avg_x_base_address = 192+bank_shift;
short acc_avg_y_base_address = 2240+bank_shift;
short acc_avg_z_base_address = 0+bank_shift;


short spi_addr;
short spi_data;

int acceleration_x[iterationNumber]={}; // 4-byte data
int acceleration_y[iterationNumber];
int acceleration_z[iterationNumber];
int acc_avg_x[iterationNumber]={};
int acc_avg_y[iterationNumber];
int acc_avg_z[iterationNumber];

#define LIS3DH_CTRL_REG1       0x20
#define LIS3DH_OUT_X_L         0x28
#define LIS3DH_OUT_X_H         0x29
#define LIS3DH_OUT_Y_L         0x2a
#define LIS3DH_OUT_Y_H         0x2b
#define LIS3DH_OUT_Z_L         0x2c
#define LIS3DH_OUT_Z_H         0x2d
#define LIS3DH_SENSITIVITY_2G  (0.001F)
#define GRAVITY (9.80665F)
I2C i2c(PB_9,PB_6);



/**
Set a flag to alert that a Transient Detection Interrupt has ocurred
*/
void Interrupt();

/**
Set a flag to alert that a Pulse Detection Interrupt has ocurred
*/
void Interrupt2();

/**
Blind the LEDS for state machine error alert
*/
void error();

/**
Set a flag to alert that a Timer1 Interrupt has ocurred
*/
void TimerExpired1();

/**
Set a flag to alert that a Timer2 Interrupt has ocurred
*/
void TimerExpired2();

/**
Performs the calculation for the chronometer time
*/
void TimerExpired3();

/**
Saves the data collected in the stepping count to the flash disk
@param date - the date of the data
@param data1 - steps
@param data2 - Kilometer
*/
void writeDataToFile(char *date,int data1,int data2);
int xx=0;
                    int xhalf ;




#define SIZE  10
int n = SIZE;
int A[SIZE][SIZE], B[SIZE][SIZE], C[SIZE][SIZE];

void pedometer()
{

    state = 0;
    I1_flag = 0;
    I2_flag = 0;
                    // leds = 0x04; 
                    aux = 0; 
                    timerFlag2 = 0;

                    //TODO_WANGBO:Add delay 


                         Timer t;
                        t.reset();
                        t.start();
                        
                        printf("Select HyCUBE and Update Sensing Data\r\n");
                        cs = 0;
                        
                        
                        spi.write(0xFD);  //first byte for idle state
                        spi.write(0x0C);  //second byte for idle state
                        spi.write(0x00);  //first byte for WR cmd
                        spi.write(0x40);  //second byte for WR cmd

                        // cs = 1;

                     for(int i=0;i<iterationNumber;i++)
                     {    
                         i_4I = 4*i;
                         i_4Iplus2 = 4*i+2;
                         
         //  #ifdef CGRA_COMPILER
         //  please_map_me();
         //  #endif
                        /*
                        acc_vector[i] = (acceleration_x[i]- acc_avg_x[i]) * (acceleration_x[i]- acc_avg_x[i])+  (acceleration_y[i]- acc_avg_y[i]) * (acceleration_y[i]- acc_avg_y[i])+ (acceleration_z[i]-acc_avg_z[i]) * (acceleration_z[i]-acc_avg_z[i]) ;
                        
                    //     // // If the acceleration vector is greater than 0.15, add the steps
                        if(acc_vector[i]  > 15)
                        {
                             step[i] = step[i] + 2;
                             // Runing
                             if (acc_vector[i]  > 100){
                                km[i] = km[i]+ 2;
                                // sd.printf("running \r\n");
                                }
                             // Walking
                             else{
                                km[i] = km[i] + 1;   
                                // sd.printf("walking \r\n");
                                }  
                        }else{
                            // sd.printf("nothing \r\n");
                        }*/

                        // cs = 0;
                        // printf("Select HyCUBE and Update Sensing Data\r\n");
                        
                        // spi.write(0xFD);  //first byte for idle state
                        // spi.write(0x0C);  //second byte for idle state
                        // spi.write(0x00);  //first byte for WR cmd
                        // spi.write(0x40);  //second byte for WR cmd

                        spi_addr = acceleration_x_base_address + i_4I;
                        spi_data = acceleration_x[i] & 0xFFFF;  // extract LSB [15:0];
                        //TODO_WANGBO:Place spi call here
                        //printf ("acceleration x is %d \r\n", acceleration_x[i]);
                        // printf ("spi address is %hi \r\n", spi_addr); printf ("spi data is %hi \r\n", spi_data); 
                        // printf ("spi address[7:0] is %hi \r\n", (spi_addr & 0x00FF)); printf ("spi address[15:8] is %hi \r\n", (spi_addr & 0xFF00) >>8);
                        // printf ("spi data[7:0] is %hi \r\n", (spi_data & 0x00FF)); printf ("spi data[15:8] is %hi \r\n", (spi_data & 0xFF00) >>8);
                        spi.write(spi_addr & 0x00FF); spi.write((spi_addr & 0xFF00) >>8); spi.write(0x02); spi.write(0x00); spi.write(spi_data & 0x00FF); spi.write((spi_data & 0xFF00) >>8);  

                        spi_addr = acceleration_x_base_address + i_4Iplus2;
                        spi_data = (acceleration_x[i] & 0xFFFF0000) >> 16; // extract MSB [31:16];
                        // printf ("spi address is %hi \r\n", spi_addr); printf ("spi data is %hi \r\n", spi_data);
                        //TODO_WANGBO:Place spi call here
                        spi.write(spi_addr & 0x00FF); spi.write((spi_addr & 0xFF00) >>8); spi.write(0x02); spi.write(0x00); spi.write(spi_data & 0x00FF); spi.write((spi_data & 0xFF00) >>8); 

                        spi_addr = acceleration_y_base_address + i_4I;
                        spi_data = acceleration_y[i] & 0xFFFF;
                        // printf ("spi address is %hi \r\n", spi_addr); printf ("spi data is %hi \r\n", spi_data);
                        //TODO_WANGBO:Place spi call here
                        spi.write(spi_addr & 0x00FF); spi.write((spi_addr & 0xFF00) >>8); spi.write(0x02); spi.write(0x00); spi.write(spi_data & 0x00FF); spi.write((spi_data & 0xFF00) >>8); 

                        spi_addr = acceleration_y_base_address + i_4Iplus2;
                        spi_data = (acceleration_y[i] & 0xFFFF0000) >> 16;
                        // printf ("spi address is %hi \r\n", spi_addr); printf ("spi data is %hi \r\n", spi_data);
                        //TODO_WANGBO:Place spi call here
                        spi.write(spi_addr & 0x00FF); spi.write((spi_addr & 0xFF00) >>8); spi.write(0x02); spi.write(0x00); spi.write(spi_data & 0x00FF); spi.write((spi_data & 0xFF00) >>8); 

                        spi_addr = acceleration_z_base_address + i_4I;
                        spi_data = acceleration_z[i] & 0xFFFF;
                        // printf ("spi address is %hi \r\n", spi_addr); printf ("spi data is %hi \r\n", spi_data);
                        //TODO_WANGBO:Place spi call here
                        spi.write(spi_addr & 0x00FF); spi.write((spi_addr & 0xFF00) >>8); spi.write(0x02); spi.write(0x00); spi.write(spi_data & 0x00FF); spi.write((spi_data & 0xFF00) >>8); 

                        spi_addr = acceleration_z_base_address + i_4Iplus2;
                        spi_data = (acceleration_z[i] & 0xFFFF0000) >> 16;
                        // printf ("spi address is %hi \r\n", spi_addr); printf ("spi data is %hi \r\n", spi_data);
                        //TODO_WANGBO:Place spi call here
                        spi.write(spi_addr & 0x00FF); spi.write((spi_addr & 0xFF00) >>8); spi.write(0x02); spi.write(0x00); spi.write(spi_data & 0x00FF); spi.write((spi_data & 0xFF00) >>8); 

                        spi_addr = acc_avg_x_base_address + i_4I;
                        spi_data = acc_avg_x[i] & 0xFFFF;
                        // printf ("spi address is %hi \r\n", spi_addr); printf ("spi data is %hi \r\n", spi_data);
                        //TODO_WANGBO:Place spi call here
                        spi.write(spi_addr & 0x00FF); spi.write((spi_addr & 0xFF00) >>8); spi.write(0x02); spi.write(0x00); spi.write(spi_data & 0x00FF); spi.write((spi_data & 0xFF00) >>8); 
                        
                        spi_addr = acc_avg_x_base_address + i_4Iplus2;
                        spi_data = (acc_avg_x[i] & 0xFFFF0000) >> 16;
                        // printf ("spi address is %hi \r\n", spi_addr); printf ("spi data is %hi \r\n", spi_data);
                        //TODO_WANGBO:Place spi call here
                        spi.write(spi_addr & 0x00FF); spi.write((spi_addr & 0xFF00) >>8); spi.write(0x02); spi.write(0x00); spi.write(spi_data & 0x00FF); spi.write((spi_data & 0xFF00) >>8); 

                        spi_addr = acc_avg_y_base_address + i_4I;
                        spi_data = acc_avg_y[i] & 0xFFFF;
                        // printf ("spi address is %hi \r\n", spi_addr); printf ("spi data is %hi \r\n", spi_data);
                        //TODO_WANGBO:Place spi call here
                        spi.write(spi_addr & 0x00FF); spi.write((spi_addr & 0xFF00) >>8); spi.write(0x02); spi.write(0x00); spi.write(spi_data & 0x00FF); spi.write((spi_data & 0xFF00) >>8); 
                        
                        spi_addr = acc_avg_y_base_address + i_4Iplus2;
                        spi_data = (acc_avg_y[i] & 0xFFFF0000) >> 16;
                        // printf ("spi address is %hi \r\n", spi_addr); printf ("spi data is %hi \r\n", spi_data);
                        //TODO_WANGBO:Place spi call here
                        spi.write(spi_addr & 0x00FF); spi.write((spi_addr & 0xFF00) >>8); spi.write(0x02); spi.write(0x00); spi.write(spi_data & 0x00FF); spi.write((spi_data & 0xFF00) >>8); 

                        spi_addr = acc_avg_z_base_address +i_4I;
                        spi_data = acc_avg_z[i] & 0xFFFF;
                        // printf ("spi address is %hi \r\n", spi_addr); printf ("spi data is %hi \r\n", spi_data);
                        //TODO_WANGBO:Place spi call here
                        spi.write(spi_addr & 0x00FF); spi.write((spi_addr & 0xFF00) >>8); spi.write(0x02); spi.write(0x00); spi.write(spi_data & 0x00FF); spi.write((spi_data & 0xFF00) >>8); 

                        spi_addr = acc_avg_z_base_address + i_4Iplus2;
                        spi_data = (acc_avg_z[i] & 0xFFFF0000) >> 16;
                        // printf ("spi address is %hi \r\n", spi_addr); printf ("spi data is %hi \r\n", spi_data);
                        //TODO_WANGBO:lace spi call here
                        spi.write(spi_addr & 0x00FF); spi.write((spi_addr & 0xFF00) >>8); spi.write(0x02); spi.write(0x00); spi.write(spi_data & 0x00FF); spi.write((spi_data & 0xFF00) >>8); 

                        //LUT
                        // spi.write(0x00); spi.write(0x60); spi.write(0x02); spi.write(0x00); spi.write(0x0b); spi.write(0x00);
                        // spi.write(0x01); spi.write(0x60); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00);
                        // spi.write(0x02); spi.write(0x60); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00);
                        // spi.write(0x03); spi.write(0x60); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00);

                        // spi.write(0x00); //first byte for EXEC cmd
                        // spi.write(0x80); //second byte for EXEC cmd
                        // spi.write(0x00); //pull for one more cycle
                        // spi.write(0x80);

                        // cs = 1;
                        // printf("SPI Transfer Ends here \r\n"); 

                        // wait(0.1);

                     }
                     
                        //LUT
                        // cs = 0;
                        spi.write(0x00); spi.write(0x60); spi.write(0x02); spi.write(0x00); spi.write(0x0b); spi.write(0x00);
                        spi.write(0x01); spi.write(0x60); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00);
                        spi.write(0x02); spi.write(0x60); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00);
                        spi.write(0x03); spi.write(0x60); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00);

                        spi.write(0x00); //first byte for EXEC cmd
                        spi.write(0x80); //second byte for EXEC cmd
                        spi.write(0x00); //pull for one more cycle
                        spi.write(0x80);
                        
                        cs = 1;
                        t.stop();
                        printf("SPI Transfer Ends here \r\n");
                        // sd.printf("The time taken was %llu microseconds\r\n", t.read_high_resolution_us());
                        wait(0.1);

                    
}



void Interrupt()
{
    /// Controls the Transient Detection Interrupt flag
    I1_flag = 1;
}

void Interrupt2()
{
    /// Controls the Pulse(Tap)Detection Interrupt flag
    I2_flag = 1;
}

void error() 
{
 }

void TimerExpired1()
{
    /// Timer 1 flag
    timerFlag1 = 1;
}

void TimerExpired2()
{
    /// Timer 2 Flag
    timerFlag2 = 1;
}

void TimerExpired3()
{
    /// Calculates the chronometer time
    second = second + 1;
    if (second > 60)
    {
        second = 0;
        minute = minute + 1;
        if (minute > 60)
        {
            hour = hour + 1;
            minute = 0;
        }
   
    }    
}

void writeDataToFile(char *date,int data1,int data2)
{
}



int main(){
 

int allData=0;
std::queue<int> xqueue;
std::queue<int> yqueue;
std::queue<int> zqueue;
int x_all,y_all,z_all;

i2c.frequency(100000);


//SPI config.
cs = 1;
spi.format(8,3);
// spi.frequency(1000000); //1MHz
spi.frequency(100000000); //100MHz
printf("SPI Master Initialized \r\n");

//write frequency to acc
char ft[2];
ft[0] = 0x20;
ft[1] = 0x97;
i2c.write(0x3c, ft, 2, false);

i2c.write(0x3c, ft, 1, true);
i2c.read(0x3c, ft, 1, false);
sd.printf("reg value %02x  \r\n",(uint8_t)ft[0]);
//Add SPI call to configure the instructions
        cs = 0;
        spi.write(0xFD);  //first byte for idle state
        spi.write(0x0C);  //second byte for idle state
        spi.write(0x00);  //first byte for WR cmd
        spi.write(0x40);  //second byte for WR cmd
// Hycube instructions generated by pedometer_spi_automate scripts
spi.write(0x06); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x1f); spi.write(0x80); 
spi.write(0x00); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x0e); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0c); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0a); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xdc); spi.write(0x01); 
spi.write(0x08); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x7f); 
spi.write(0x16); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x14); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x06); spi.write(0x00); 
spi.write(0x12); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xc7); spi.write(0x00); 
spi.write(0x10); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x8f); 
spi.write(0x1e); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x1c); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x3f); 
spi.write(0x1a); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xdc); spi.write(0x41); 
spi.write(0x18); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xfc); spi.write(0x7f); 
spi.write(0x26); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x24); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x06); spi.write(0x00); 
spi.write(0x22); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x20); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xef); spi.write(0xff); 
spi.write(0x2e); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2c); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2a); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x00); 
spi.write(0x28); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xf9); spi.write(0xcf); 
spi.write(0x36); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x34); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x10); spi.write(0x00); 
spi.write(0x32); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x41); 
spi.write(0x30); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xcf); spi.write(0xff); 
spi.write(0x3e); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3c); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3a); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x0b); 
spi.write(0x38); spi.write(0x40); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x06); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x1f); spi.write(0x80); 
spi.write(0x00); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0xfc); spi.write(0x9f); 
spi.write(0x0e); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x0c); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0a); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0xdf); spi.write(0x41); 
spi.write(0x08); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x07); spi.write(0xff); 
spi.write(0x16); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x14); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x12); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x00); 
spi.write(0x10); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x7f); spi.write(0xfe); 
spi.write(0x1e); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1c); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1a); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0xbf); spi.write(0x08); 
spi.write(0x18); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x7a); spi.write(0xfe); 
spi.write(0x26); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x24); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x22); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x20); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x2e); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2c); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2a); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x9c); spi.write(0x01); 
spi.write(0x28); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0xcf); spi.write(0x2f); 
spi.write(0x36); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x32); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0xfe); spi.write(0x80); 
spi.write(0x30); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0xf9); spi.write(0x4f); 
spi.write(0x3e); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3c); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3a); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xc0); 
spi.write(0x38); spi.write(0x41); spi.write(0x02); spi.write(0x00); spi.write(0xfc); spi.write(0x9f); 
spi.write(0x06); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x1f); spi.write(0x80); 
spi.write(0x00); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xd4); spi.write(0x9f); 
spi.write(0x0e); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x0c); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x04); 
spi.write(0x0a); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x40); 
spi.write(0x08); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x3f); spi.write(0x9f); 
spi.write(0x16); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x14); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x46); 
spi.write(0x12); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x41); 
spi.write(0x10); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xe7); spi.write(0xff); 
spi.write(0x1e); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1c); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1a); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x11); 
spi.write(0x18); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x26); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x24); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x22); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x20); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x2e); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2c); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2a); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x28); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xbf); spi.write(0xfe); 
spi.write(0x36); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x32); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xbd); spi.write(0x01); 
spi.write(0x30); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x2f); 
spi.write(0x3e); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3c); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3a); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xbf); spi.write(0xc0); 
spi.write(0x38); spi.write(0x42); spi.write(0x02); spi.write(0x00); spi.write(0xd4); spi.write(0x9f); 
spi.write(0x06); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x1c); spi.write(0x80); 
spi.write(0x00); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xaf); 
spi.write(0x0e); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0c); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0a); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x41); 
spi.write(0x08); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xe7); spi.write(0xff); 
spi.write(0x16); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x14); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x12); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x10); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x1e); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1c); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1a); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x18); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x26); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x24); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x22); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x20); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x2e); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2c); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2a); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x28); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x36); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x32); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x30); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x3e); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3c); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3a); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xbc); spi.write(0x00); 
spi.write(0x38); spi.write(0x43); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xaf); 
spi.write(0x06); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x1f); spi.write(0x80); 
spi.write(0x00); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xdf); spi.write(0xf1); 
spi.write(0x0e); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0c); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0a); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xdc); spi.write(0x09); 
spi.write(0x08); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x7f); 
spi.write(0x16); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x14); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x06); spi.write(0x00); 
spi.write(0x12); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x04); 
spi.write(0x10); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xf3); 
spi.write(0x1e); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1c); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1a); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xfc); spi.write(0x03); 
spi.write(0x18); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xc7); spi.write(0xff); 
spi.write(0x26); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x24); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x06); spi.write(0x00); 
spi.write(0x22); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xdc); spi.write(0x20); 
spi.write(0x20); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x1f); 
spi.write(0x2e); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2c); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x06); spi.write(0x00); 
spi.write(0x2a); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xc4); spi.write(0xc0); 
spi.write(0x28); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xfd); spi.write(0x39); 
spi.write(0x36); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x06); spi.write(0x00); 
spi.write(0x32); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xdf); spi.write(0xe5); 
spi.write(0x30); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xc7); spi.write(0xff); 
spi.write(0x3e); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3c); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3a); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x38); spi.write(0x44); spi.write(0x02); spi.write(0x00); spi.write(0xdf); spi.write(0xf1); 
spi.write(0x06); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x1f); spi.write(0x80); 
spi.write(0x00); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xf9); 
spi.write(0x0e); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x0c); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x00); 
spi.write(0x0a); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x5f); spi.write(0xc3); 
spi.write(0x08); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x8e); 
spi.write(0x16); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x14); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x12); spi.write(0x00); 
spi.write(0x12); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0xdf); spi.write(0x03); 
spi.write(0x10); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x3f); spi.write(0xfa); 
spi.write(0x1e); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x1c); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x07); spi.write(0x00); 
spi.write(0x1a); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xc2); 
spi.write(0x18); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xd3); 
spi.write(0x26); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x24); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x22); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x43); 
spi.write(0x20); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x2f); spi.write(0xfe); 
spi.write(0x2e); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2c); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2a); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0xbc); spi.write(0x24); 
spi.write(0x28); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xae); 
spi.write(0x36); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x32); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x80); 
spi.write(0x30); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x79); spi.write(0x99); 
spi.write(0x3e); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x3c); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0x12); spi.write(0x00); 
spi.write(0x3a); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x38); spi.write(0x45); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xf9); 
spi.write(0x06); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x1f); spi.write(0x80); 
spi.write(0x00); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0xd3); spi.write(0xa9); 
spi.write(0x0e); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x0c); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x12); spi.write(0x00); 
spi.write(0x0a); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x08); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x97); spi.write(0xa8); 
spi.write(0x16); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x14); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x12); spi.write(0x00); 
spi.write(0x12); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x7f); spi.write(0x03); 
spi.write(0x10); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xae); 
spi.write(0x1e); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x1c); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x12); spi.write(0x00); 
spi.write(0x1a); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x03); 
spi.write(0x18); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xdf); 
spi.write(0x26); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x24); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x06); 
spi.write(0x22); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x43); 
spi.write(0x20); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xdf); 
spi.write(0x2e); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x2c); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x44); 
spi.write(0x2a); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x41); 
spi.write(0x28); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x2f); spi.write(0xff); 
spi.write(0x36); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x32); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0xbf); spi.write(0x11); 
spi.write(0x30); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xaf); 
spi.write(0x3e); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x3c); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x12); spi.write(0x00); 
spi.write(0x3a); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0x7f); spi.write(0x01); 
spi.write(0x38); spi.write(0x46); spi.write(0x02); spi.write(0x00); spi.write(0xd3); spi.write(0xa9); 
spi.write(0x06); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x1f); spi.write(0x80); 
spi.write(0x00); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xf5); 
spi.write(0x0e); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0c); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0a); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x21); 
spi.write(0x08); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x16); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x14); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x12); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x10); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xdf); spi.write(0xff); 
spi.write(0x1e); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1c); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1a); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x18); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x26); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x24); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x22); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x20); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x2e); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2c); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2a); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x28); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x36); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x32); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x30); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x3e); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3c); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3a); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xbf); spi.write(0x01); 
spi.write(0x38); spi.write(0x47); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xf5); 
spi.write(0x06); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x05); spi.write(0x80); 
spi.write(0x00); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xfc); spi.write(0x8f); 
spi.write(0x0e); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0c); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x06); spi.write(0x00); 
spi.write(0x0a); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xdf); spi.write(0xc1); 
spi.write(0x08); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xe7); spi.write(0xf1); 
spi.write(0x16); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x14); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x46); spi.write(0x3f); 
spi.write(0x12); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xdf); spi.write(0x01); 
spi.write(0x10); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xf1); 
spi.write(0x1e); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x1c); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xf6); spi.write(0x7f); 
spi.write(0x1a); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x7d); spi.write(0x81); 
spi.write(0x18); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xfc); spi.write(0xff); 
spi.write(0x26); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x24); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x06); spi.write(0x00); 
spi.write(0x22); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xd3); spi.write(0x00); 
spi.write(0x20); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xfc); spi.write(0x83); 
spi.write(0x2e); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x2c); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xf7); spi.write(0x7f); 
spi.write(0x2a); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xdc); spi.write(0x48); 
spi.write(0x28); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xfc); spi.write(0x73); 
spi.write(0x36); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x06); spi.write(0x00); 
spi.write(0x32); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x7d); spi.write(0x01); 
spi.write(0x30); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x3e); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3c); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x06); spi.write(0x00); 
spi.write(0x3a); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0x45); spi.write(0x01); 
spi.write(0x38); spi.write(0x48); spi.write(0x02); spi.write(0x00); spi.write(0xfc); spi.write(0x8f); 
spi.write(0x06); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x1f); spi.write(0x80); 
spi.write(0x00); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x17); spi.write(0xce); 
spi.write(0x0e); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x0c); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x84); spi.write(0x00); 
spi.write(0x0a); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0xd3); spi.write(0x83); 
spi.write(0x08); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x3f); spi.write(0xde); 
spi.write(0x16); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x80); 
spi.write(0x14); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x04); spi.write(0x00); 
spi.write(0x12); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0xdf); spi.write(0x43); 
spi.write(0x10); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x7f); spi.write(0x8f); 
spi.write(0x1e); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x1c); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x42); 
spi.write(0x1a); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0xbf); spi.write(0x41); 
spi.write(0x18); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0xfa); spi.write(0xf9); 
spi.write(0x26); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x24); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x22); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x0b); spi.write(0x01); 
spi.write(0x20); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x8e); 
spi.write(0x2e); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x2c); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x04); spi.write(0x00); 
spi.write(0x2a); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x9e); spi.write(0x41); 
spi.write(0x28); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x3f); spi.write(0xc4); 
spi.write(0x36); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x04); spi.write(0x00); 
spi.write(0x32); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x30); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0xfc); spi.write(0xc9); 
spi.write(0x3e); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x3c); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x08); spi.write(0x00); 
spi.write(0x3a); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x9f); spi.write(0x43); 
spi.write(0x38); spi.write(0x49); spi.write(0x02); spi.write(0x00); spi.write(0x17); spi.write(0xce); 
spi.write(0x06); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x1f); spi.write(0x80); 
spi.write(0x00); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x7c); spi.write(0xbe); 
spi.write(0x0e); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x0c); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x12); spi.write(0x00); 
spi.write(0x0a); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x5f); spi.write(0x03); 
spi.write(0x08); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x3f); spi.write(0xbe); 
spi.write(0x16); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x14); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x12); spi.write(0x00); 
spi.write(0x12); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0xdf); spi.write(0x01); 
spi.write(0x10); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x07); spi.write(0xdf); 
spi.write(0x1e); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x1c); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x08); 
spi.write(0x1a); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x51); 
spi.write(0x18); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0xe7); spi.write(0xff); 
spi.write(0x26); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x24); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x22); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x20); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0xbf); spi.write(0xfe); 
spi.write(0x2e); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2c); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2a); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x7f); spi.write(0x01); 
spi.write(0x28); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xfe); 
spi.write(0x36); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x32); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0xbf); spi.write(0x09); 
spi.write(0x30); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xaf); 
spi.write(0x3e); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x3c); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x12); spi.write(0x00); 
spi.write(0x3a); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x7f); spi.write(0x01); 
spi.write(0x38); spi.write(0x4a); spi.write(0x02); spi.write(0x00); spi.write(0x7c); spi.write(0xbe); 
spi.write(0x06); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x1f); spi.write(0x80); 
spi.write(0x00); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xaf); 
spi.write(0x0e); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x0c); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x02); 
spi.write(0x0a); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x41); 
spi.write(0x08); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x3f); spi.write(0xff); 
spi.write(0x16); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x14); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x12); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x7f); spi.write(0x01); 
spi.write(0x10); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xfe); 
spi.write(0x1e); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1c); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1a); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x18); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x26); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x24); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x22); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x20); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x2e); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2c); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2a); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x28); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x36); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x32); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x30); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x3e); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3c); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3a); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0xbf); spi.write(0x01); 
spi.write(0x38); spi.write(0x4b); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xaf); 
spi.write(0x06); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x1f); spi.write(0x80); 
spi.write(0x00); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x0e); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0c); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0a); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x7f); spi.write(0x01); 
spi.write(0x08); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xfb); spi.write(0xff); 
spi.write(0x16); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x14); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x12); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x10); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x1e); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1c); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1a); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xdc); spi.write(0x01); 
spi.write(0x18); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x7f); 
spi.write(0x26); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x24); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x06); spi.write(0x00); 
spi.write(0x22); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xdf); spi.write(0x01); 
spi.write(0x20); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xf1); 
spi.write(0x2e); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2c); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2a); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xdf); spi.write(0x01); 
spi.write(0x28); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xfc); spi.write(0xf1); 
spi.write(0x36); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x32); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x30); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x3e); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3c); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3a); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x38); spi.write(0x4c); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x06); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x1d); spi.write(0x80); 
spi.write(0x00); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xaf); 
spi.write(0x0e); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0c); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0a); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0xbe); spi.write(0x81); 
spi.write(0x08); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0xfa); spi.write(0x4f); 
spi.write(0x16); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x14); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x12); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0xde); spi.write(0xc1); 
spi.write(0x10); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x0f); 
spi.write(0x1e); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1c); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1a); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0xdf); spi.write(0x43); 
spi.write(0x18); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x3f); spi.write(0xce); 
spi.write(0x26); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x24); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x25); spi.write(0x03); 
spi.write(0x22); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x41); 
spi.write(0x20); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x7c); spi.write(0xdf); 
spi.write(0x2e); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x2c); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x7d); spi.write(0x00); 
spi.write(0x2a); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x51); 
spi.write(0x28); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x3f); spi.write(0xff); 
spi.write(0x36); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x32); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x30); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x3e); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3c); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3a); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0x7d); spi.write(0x01); 
spi.write(0x38); spi.write(0x4d); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xaf); 
spi.write(0x06); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x1f); spi.write(0x80); 
spi.write(0x00); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x0e); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0c); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0a); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x11); 
spi.write(0x08); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x16); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x14); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x12); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x7f); spi.write(0x01); 
spi.write(0x10); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xfe); 
spi.write(0x1e); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1c); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1a); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x7f); spi.write(0x01); 
spi.write(0x18); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xfe); 
spi.write(0x26); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x24); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x22); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0xab); spi.write(0x01); 
spi.write(0x20); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x2e); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x2c); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x0c); spi.write(0x00); 
spi.write(0x2a); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0xfe); spi.write(0x01); 
spi.write(0x28); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x2f); 
spi.write(0x36); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x32); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x41); 
spi.write(0x30); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xf9); 
spi.write(0x3e); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3c); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3a); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x38); spi.write(0x4e); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x06); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x40); 
spi.write(0x04); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x0f); spi.write(0x27); 
spi.write(0x02); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x1f); spi.write(0x80); 
spi.write(0x00); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x0e); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0c); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x0a); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x08); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x16); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x14); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x12); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x10); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x1e); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1c); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x1a); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x18); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x26); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x24); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x22); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x20); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x2e); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2c); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x2a); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x28); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x36); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x34); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x32); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x30); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 
spi.write(0x3e); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3c); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00); 
spi.write(0x3a); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0x01); 
spi.write(0x38); spi.write(0x4f); spi.write(0x02); spi.write(0x00); spi.write(0xff); spi.write(0xff); 

//LUT
// spi.write(0x00); spi.write(0x60); spi.write(0x02); spi.write(0x00); spi.write(0x0b); spi.write(0x00);
// spi.write(0x01); spi.write(0x60); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00);
// spi.write(0x02); spi.write(0x60); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00);
// spi.write(0x03); spi.write(0x60); spi.write(0x02); spi.write(0x00); spi.write(0x00); spi.write(0x00);

         spi.write(0xFD);  //first byte for idle state
         spi.write(0x0C);  //second byte for idle state
         cs = 1;
         printf("SPI for instructions done \r\n");

wait(1);
 while(true){
        for(int i=0;i<iterationNumber;i++){    
        sub_x = 0;
        sub_y = 0;
        sub_z = 0;
        
        char st[2],data[6];
        float dt[3];
        st[0] = LIS3DH_CTRL_REG1;
        st[1] = 0x27; //ORIGINAL 0x7f
        i2c.write(0x3c , st , 2);

        st[0] = LIS3DH_OUT_X_H | 0x80;
        //dbf[1] = 0xff; 
        i2c.write(0x3c, st, 1);
        
        st[0] = LIS3DH_OUT_Y_H | 0x80;
        i2c.write(0x3c, st, 1);
        
        st[0] = LIS3DH_OUT_Z_H | 0x80;
        i2c.write(0x3c, st, 1);
        
        i2c.read(0x3c | 1, data, 6);
        dt[0] = float(short((data[1]<<8) | data[0])) *LIS3DH_SENSITIVITY_2G  / 15 * GRAVITY;
        dt[1] = float(short((data[3]<<8) | data[2])) *LIS3DH_SENSITIVITY_2G  / 15 * GRAVITY;
        dt[2] = float(short((data[5]<<8) | data[4])) *LIS3DH_SENSITIVITY_2G  / 15 * GRAVITY;
        
        // sd.printf("x -> %.2f y -> %.2f z -> %.2f\r\n",dt[0],dt[1],dt[2]);




        acceleration_x[i] = (int) dt[0] * 10000;
        acceleration_y[i] = (int) dt[1] * 10000;
        acceleration_z[i] = (int) dt[2] * 10000;

        if(xqueue.size()<50){
            //no pop
            x_all += acceleration_x[i];
            xqueue.push(acceleration_x[i]);
            acc_avg_x[i] = x_all /(xqueue.size());

            y_all += acceleration_y[i];
            yqueue.push(acceleration_y[i]);
            acc_avg_y[i] = y_all /(yqueue.size());


            z_all += acceleration_z[i];
            zqueue.push(acceleration_z[i]);
            acc_avg_z[i] = z_all /(zqueue.size());

        }else{
            int temp;

            x_all += acceleration_x[i];
            temp = xqueue.front();
             x_all = x_all - temp;
             xqueue.pop();
            xqueue.push(acceleration_x[i]);
            acc_avg_x[i] = x_all /50;

            y_all += acceleration_y[i];
            temp = yqueue.front();
             y_all = y_all - temp;
             yqueue.pop();
            yqueue.push(acceleration_y[i]);
            acc_avg_y[i] = y_all /50;

            z_all += acceleration_z[i];
            temp = zqueue.front();
             z_all = z_all - temp;
             zqueue.pop();
            zqueue.push(acceleration_z[i]);
            acc_avg_z[i] = z_all /50;
        }
        
        
    }


    pedometer();

     }
//gemm();
return 0;
}
