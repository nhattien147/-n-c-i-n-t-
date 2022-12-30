#include <stdio.h>
#include <stdint.h>
#include <math.h> // -lm
#include "unistd.h"
#include "string.h"
#include <stdlib.h>
#include "MPU6050.h"
#include "gps_imu.h"
#include "MQTTClient.h"
#include "mqtt_pub_sub.h"
#include "./includes/thuc.h"
#include <unistd.h>
#include <time.h>

float Ax, Ay, Az;
float Vx=0, Vy=0, Vz=0, V=0;
float delT1 = 0.5;
float lat, lon;
int on_message(void *context, char *topicName, int topicLen, MQTTClient_message *message){
    char* payload = (char*)message->payload;
    printf("Received message: %s\n", payload);
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}
//*******************************************************************************
//                               Khai bao Heart Rate
//*******************************************************************************
int I2C_ADS1015;
adsGain_t dADS1015_GAIN;
uint8_t dADS1015_CON_DELAY;
uint8_t   dADS1015_BIT_SHIFT;
int giatri=0, dem=0;
//uint32_t i=0;
clock_t start_t, end_t;
float T, f;
int temp = 0, data1=0, data2=0;
int threshold = 60, flag=1;

void dINIT_ADS1015(){
    I2C_ADS1015 = wiringPiI2CSetup(ADS1015_ADDRESS);
}

uint16_t dADS1015_READ_ADC(uint8_t channel){
    if(channel > 3){return 0;}
    uint16_t config1 =   ADS1015_REG_CONFIG_CQUE_NONE    |
                         ADS1015_REG_CONFIG_CLAT_NONLAT  |
                         ADS1015_REG_CONFIG_CPOL_ACTVLOW |
                         ADS1015_REG_CONFIG_CMODE_TRAD   |
                         ADS1015_REG_CONFIG_DR_1600SPS   |
                         ADS1015_REG_CONFIG_MODE_SINGLE;
    
    // Set PGA/voltage range
    config1 |= dADS1015_GAIN;

    switch (channel)
    {
        case (0):
        config1 |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
        break;
        case (1):
        config1 |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
        break;
        case (2):
        config1 |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
        break;
        case (3):
        config1 |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
        break;
    }
    config1 |= ADS1015_REG_CONFIG_OS_SINGLE;
    wiringPiI2CWriteReg16(I2C_ADS1015, ADS1015_REG_POINTER_CONFIG, (config1>>8) | (config1<<8));
    usleep(1000*dADS1015_CON_DELAY);

    return dADS1015_READ_REGISTER(ADS1015_REG_POINTER_CONVERT) >> dADS1015_BIT_SHIFT;  
}

uint16_t dADS1015_READ_REGISTER(uint8_t reg){
    wiringPiI2CWrite(I2C_ADS1015, ADS1015_REG_POINTER_CONVERT);
    uint16_t reading = wiringPiI2CReadReg16(I2C_ADS1015, reg);
    reading = (reading>>8) | (reading<<8);
    return reading;
}

/*
*   CÁC HÀM KHỞI TẠO CHƯƠNG TRÌNH
*/
void InitGPIO(){
    wiringPiSetup();
}

void InitSensors(){
    // Khởi tạo, cấu hình ADC.
    dINIT_ADS1015();    
}

void ReadAllSensors(){

    s_Sensors.soilMoisture = map(dADS1015_READ_ADC(0), 0, 16480, 0, 100);

}

uint8_t map(int x, int in_min, int in_max, uint8_t out_min, uint8_t out_max){
    return   ((x -in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

int a[10000];

float nhip(){
    ReadAllSensors();
    int data =  s_Sensors.soilMoisture;
    temp = data;
    data=data1;
    data1=temp;
    if(data!=data1) {
        printf("%d\n", data1);
        if (data1 > threshold){
            // nhiptrengiay++;
            // printf("nhiptrengiay duoc: %d\n", nhiptrengiay);
            end_t = clock();
            T = (end_t-start_t)/100000;
            f = 1/T;
            end_t = start_t;

        }
    }
    return f;
}
//*******************************************************************************
//                               MAIN CODE
//*******************************************************************************
int main(int argc, char* argv[])
{
    //*************************************
    //      Khai bao bien Heart rate
    //*************************************
    InitGPIO();
    InitSensors();
    start_t = clock();
    //***********GPS declare variable************
    int fd;
    fd = serialOpen("/dev/ttyAMA0", 9600);
    wiringPiSetup();
    char c;
    Ublox M8_Gps;
    float gpsArray[10];
	MPU6050_init();
    MQTTClient client;
    MQTTClient_create(&client, ADDRESS, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
	MQTTClient_setCallbacks(client, NULL, NULL, on_message, NULL);
    
    int rc;
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect, return code %d\n", rc);
        exit(-1);
    }
    //listen for operation
    MQTTClient_subscribe(client, SUB_TOPIC, 0);
    while(1) {
		//--------------------------------
		while(serialDataAvail(fd)){
            c = serialGetchar(fd);
            if(M8_Gps.encode(c)){
                gpsArray[1] = M8_Gps.latitude;
                gpsArray[2] = M8_Gps.longitude;
            }
        }
		lat = gpsArray[1];
		lon = gpsArray[2];
		// printf("\nlat = %0.5f\n", lat);
		// printf("\nlon = %0.5f\n", lon);
		//--------------------------------
        char msg [50];
        Az = get_Az_value();
		Ay = get_Ay_value();
        Az = get_Az_value();
        int getAx = 0, getAy=0, getAz=0;
        if(getAz < 10 && getAy <10){
            getAz++;
            getAy++;
            Az = get_Az_value();
            Ay = get_Ay_value();
            Az += Az;
            Ay += Ay;
        }
        else{
            getAz = 0;
            getAy = 0;
            Az = Az/10;
            Ay = Ay/10;
        }
        // if(getAy < 50){
        //     getAy++;
        //     Ay = get_Ay_value();
        //     Ay += Ay;
        // }
        // else{
        //     getAy = 0;
        //     Ay = Ay/50;
        // }
        printf("Ax = %0.2f\n", Ax);
        printf("Ay = %0.2f\n", Ay);
        printf("Az = %0.2f\n", Az);
        // Vx = Vx + Ax*delT1;
        // Vy = Vy + Ay*delT1;
        // Vz = Vz + Az*delT1;
        // V = sqrt(Vx*Vx + Vy*Vy);
        // printf("\nVx = %0.2f\n", Vx);
        // printf("\nVy = %0.2f\n", Vy);
        // printf("\nVz = %0.2f\n", Vz);
        // printf("\nV = %0.2f\n", V);
		KalmanFilter(Ax, Ay, lat, lon);
		// float lat1 = gpsArray[1];
        // float lon1 = gpsArray[2];
        //**************************************
        //              Heart rate
        //**************************************
        if(giatri<3){
            giatri++;
            dem += nhip();
        }
        else if(giatri>=3){
            giatri=0;
            dem=dem/3;
        }
        //***************************SEND DATA TO BROKER*********************************
        sprintf(msg,"{\"heart_rate\":%d, \"heart_draw\":%d, \"v\":%0.2f, \"lat\":%0.5f, \"lon\":%0.5f}",dem,data1,V_corr,lat_corr,lon_corr);
        publish(client, PUB_TOPIC, msg);
        delay(500);
    }
    MQTTClient_disconnect(client, 1000);
    MQTTClient_destroy(&client);
    return rc;
    
}
