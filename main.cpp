#include "mbed.h"
#include "PS3.h"
#include "mbed_wait_api.h"
#include "QEI.h"
#include "PIDcontroller.h"

#define ADDRESS_ROLLER 0x60

I2C i2c(D14,D15);
PS3 ps3 (A0,A1);
DigitalOut sig(D13);

PID roller(1000.0 , 0.0, 0.0 , 0.050);

QEI encoder_roller( D2, D3, NC, 2048, QEI::X2_ENCODING);
Ticker flip;

void send(char add, char data);
void get_rpm();
void get_angle();
int roller_PID();

int pulse;
double true_rpm;
double angle;

char roller_data;
char true_roller_data;

int main(){

    sig = 0;

    encoder_roller.reset();
    flip.attach(&get_rpm,50ms);

    while (true) {
        
        //緊急停止
        if(ps3.getSELECTState()){
            sig = 1;
        }

        if(ps3.getSTARTState()){
            sig = 0;
        }

        //printf("m:%d L:%d R:%d Lx%d Ly%d\n",button_maru,L1,R1,Lx,Ly);
        printf("pulse:%6.0d motordata: %3.0d %5.2lf[rpm]  \n", pulse, true_roller_data, true_rpm);

        if(ps3.getButtonState(PS3::maru)){
            send(ADDRESS_ROLLER, true_roller_data);
        }else{
            send(ADDRESS_ROLLER, true_roller_data);
        }

    }
}

int roller_PID(float target_rpm){

    //入力値（rpm）の範囲定義
    roller.setInputLimits(-5000.0,5000.0);

    if(true_rpm <= target_rpm){
        roller.setOutputLimits(0x84, 0xff);
    }else if(true_rpm > target_rpm){
        roller.setOutputLimits(0x00, 0x7B);
    }

    roller.setSetPoint(target_rpm);//目標値設定

    roller.setProcessValue(true_rpm);//現在の値（rpm）

    roller_data = roller.compute();

    if(true_rpm <= target_rpm){
        true_roller_data = roller_data;
    }else if(true_rpm > target_rpm){
        true_roller_data = 0x7b - roller_data;
    }

    return 0;
}

void send(char address, char data){
    wait_us(15000);
    i2c.start();
    i2c.write(address);
    i2c.write(data);
    i2c.stop();
}

void get_rpm(){
    pulse = encoder_roller.getPulses();
    encoder_roller.reset();
    true_rpm = (60*20*(double)pulse) / (2048*2) ;
}
