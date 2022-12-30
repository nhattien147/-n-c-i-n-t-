//#include "SimpleKalmanFilter.h"
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdint.h> //int_16t
#include <math.h>
#define sample_rate 25
#define config 26
#define gyro_config 27
#define acc_config 28
#define acc_config2 29
#define interrupt 56
#define PWR_Managment 107
#define Acc_X 59
#define Acc_Y 61
#define Acc_Z 63
#define Gyro_X 67
#define Gyro_Y 69
#define Gyro_Z 71
#define WHOIAM 0x00 // thanh ghi 0 AK8963 ; dia chi tra ve : 0x48H
#define Mag_X 4     // 0X4H: high data  0X3H: low data   // magnetic flux density = 4912 // dữ liệu sắp xếp theo kiểu little edian (bit thấp được sắp đầu tiên sau đó tới bit cao)
#define Mag_Y 6     // 0X6H: high data  0X5H: low data
#define Mag_Z 8     // 0X8H: high data  0X7H: low data
#define User_control 0X6A
#define bypass_EN 0x37
#define CNTL1 10
#define ASAX 16
#define ASAY 17
#define ASAZ 18
#define ST2 9
#define ST1 2
#define g 9.81
	
int  MPU6050;

void MPU6050_init() // cấu hình các chế độ cho MPU6050
{
    MPU6050 = wiringPiI2CSetup(0x68);
    wiringPiI2CWriteReg8(MPU6050, sample_rate, 0);
    wiringPiI2CWriteReg8(MPU6050, config, 0x01);
	wiringPiI2CWriteReg8(MPU6050, gyro_config, 0x08);
    wiringPiI2CWriteReg8(MPU6050, acc_config, 0x10);  // +_8g
    wiringPiI2CWriteReg8(MPU6050, acc_config2, 0x09); // bat LPF cho cam bien gia tốc
    wiringPiI2CWriteReg8(MPU6050, interrupt, 1);
    wiringPiI2CWriteReg8(MPU6050, PWR_Managment, 0x01);
    // set cấu hình cho MPU6050
    // sample rate = 500 Hz => 1
    // bật DLPF chọn mode 1 bandwith= 184 Hz;delay= 2.9 ms; Fs=1KHz
    // Cấu hình con quay hồi chuyển : +500 độ/s =01    => thanh ghi: 00001011=> 0x0B
    // Cấu hình cảm biến gia tốc : +-8g = 10 => 00010000 => 0x10
    wiringPiI2CWriteReg8(MPU6050, User_control, 0x00); // Disable the I2C master interface : 0x00
    wiringPiI2CWriteReg8(MPU6050, bypass_EN, 0x02);    // Turn on bypass multiplexer : 0x02
}


int16_t read_value(unsigned char address)
{
    int16_t data;
    data = wiringPiI2CReadReg8(MPU6050,address)<<8 | wiringPiI2CReadReg8(MPU6050,address+1);
    return data;
}

float get_Ax_value(void)
{
    //uint16_t coef = 2046.0;
    uint16_t coef = 4096.0;
    float Ax = (float)read_value(Acc_X)/coef;
    // float Ay = (float)read_value(Acc_Y)/coef;
    // float Az = (float)read_value(Acc_Z)/coef;
    //return sqrt(pow(Ax,2)+pow(Ay,2)+pow(Az,2));
	return Ax;
}

float get_Ay_value(void)
{
    //uint16_t coef = 2046.0;
    uint16_t coef = 4096.0;
    // float Ax = (float)read_value(Acc_X)/coef;
    float Ay = (float)read_value(Acc_Y)/coef;
    // float Az = (float)read_value(Acc_Z)/coef;
    //return sqrt(pow(Ax,2)+pow(Ay,2)+pow(Az,2));
	return Ay;
}

float get_Az_value(void)
{
    //uint16_t coef = 2046.0;
    uint16_t coef = 4096.0;
    // float Ax = (float)read_value(Acc_X)/coef;
    // float Ay = (float)read_value(Acc_Y)/coef;
    float Az = (float)read_value(Acc_Z)/coef;
    //return sqrt(pow(Ax,2)+pow(Ay,2)+pow(Az,2));
    return Az;
}
