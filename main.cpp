/*
Fair and Balanced
June 2023

Authors: 
Brandon Zalinsky - brandonzalinsky@gmail.com
Daniel J. Gonzalez - dgonzrobotics@gmail.com

Designed for NucleoL432KC
*/
#include "main.h"
#include <cmath>

using namespace FastMath;

void setupServo() {
    float myRange = .000675;
    float myDegrees = 90;
    drive0.calibrate(myRange, myDegrees);
    drive1.calibrate(myRange, myDegrees);
    weapon.calibrate(myRange, myDegrees);
}

void setupIMU() {
    roll = 0;
    pitch = 0; 
    yaw = 0;
    rollDot = 0; 
    pitchDot = 0;
    yawDot = 0;
    gyro_comp = 0.98;

    if(mpu.getID()==0x68) {
         pcSerial.printf("MPU6050 OK\n");
         wait(1);
     } else {
         pcSerial.printf("MPU6050 error ID=0x%x\r\n",mpu.getID());
     }
     mpu.start();
     pcSerial.printf("Zeroing in 3...");
     wait(1);
     pcSerial.printf("2...");
     wait(1);
     pcSerial.printf("1...");
     wait(1);
     pcSerial.printf("Go!\n");
     gx_z = 0;
     gy_z = 0;
     gz_z = 0;
     for(int i = 0; i<100; i++){
         mpu.read(&gx,&gy,&gz,&ax,&ay,&az);
         wait(0.01);
         gx_z += gx;
         gy_z += gy;
         gz_z += gz;
     }
     gx_z /= 100.0f;
     gy_z /= 100.0f;
     gz_z /= 100.0f;
     pcSerial.printf("Zeroing Complete.\n");
}



void setup(){
    
    pcSerial.baud(115200);
    pcSerial.printf("----------- Start! -----------\n");
    //------------------------------    Servo Out
    setupServo();  
    drive0.write(0.5);
    drive1.write(0.5);
    weapon.write(0.0);
    pcSerial.printf("Servos Set Up\n");
    
    //Create instance of PPM class
    //Pass in interrupt pin, minimum output value, maximum output value, minimum pulse time from transmitter, maximum pulse time from transmitter, number of channels, throttle channel (used for failsafe)
    ppmInputs = new PPM(PPMinterruptPin, 0, 1, 1000, 1900, 8, 3);
    pcSerial.printf("PPM Set Up\n");
    setupIMU();
    pcSerial.printf("IMU Set Up\n");
    
    //set up timer
    timer.start();
    tPrev = timer.read_us()/1000000.0;
    t = timer.read_us()/1000000.0;
    dt = t - tPrev;
    pcSerial.printf("Timer Set Up\n"); 
    
    enabled = 1;
}

int main() {
    setup();
    pcSerial.printf("----------- Loop Start! -----------\n");
    while(isRunning){
        loop();
    }
}

void getRCInputs(){
    //------------------------------    Get RC Input
    //Get channel data (mapped to between 0 and 1 as I passed these in as my max and min in the PPM constructor)
    //The throttle channel will return -1 if the signal from the transmitter is lost (the throttle pulse time goes below the minimum pulse time passed into the PPM constructor)
    ppmInputs->GetChannelData(rcCommandInputsRaw);
    
    // Channels: LUD, RRL, RUD, LRL, Pot1, Pot2, LeftSwitch, Right trigger
    for(int i = 0; i < 8; i++){
        rcCommandInputsRaw[i]*=100;
        rcCommandInputs[i]=rcCommandInputsRaw[i];
        if(i==1){  //If RRL
            rcCommandInputs[i] = map(rcCommandInputs[i], RRL_MIN, RRL_MAX,-100,100);
        }else if(i==0){ //if LUD
            rcCommandInputs[i] = map(rcCommandInputs[i], LUD_MIN, LUD_MAX,-100,100);
        }else if(i==2){ //if LUD
            rcCommandInputs[i] = map(rcCommandInputs[i], LUD_MIN, LUD_MAX,-100,100);
        }else if(i==3){ //if LRL
            rcCommandInputs[i] = map(rcCommandInputs[i], LRL_MIN, LRL_MAX,-100,100);
        }else if(i==4 || i==5){ //if Pot1 or Pot 2
            rcCommandInputs[i] = map(rcCommandInputs[i], LRL_MIN, LRL_MAX,0.0,1.0); //Do scaling factor for potentiometers
        }else if(i == 6 || i == 7){
            rcCommandInputs[i] = rcCommandInputs[i] > (LRL_MAX/2); //Toggle Switches
        }
        
        //Implement deadband
        if(i>=0 && i<=3){
            if(rcCommandInputs[i]<DEADBAND && rcCommandInputs[i]>-DEADBAND){
                rcCommandInputs[i] = 0;
            }else if(rcCommandInputs[i]>=DEADBAND){
                rcCommandInputs[i] = map(rcCommandInputs[i], DEADBAND, 100, 0, 100);
            }else if(rcCommandInputs[i]<=-DEADBAND){
                rcCommandInputs[i] = map(rcCommandInputs[i], -100, -DEADBAND, -100, 00);
            }
        }
    }
}

void scaleSkidSteer(int arrayLen){
    bool flag = 0;
    double maxVal = 0;
    for(int i=0; i<arrayLen; i++){
        if(fabs(driveCmds[i])>100 && fabs(driveCmds[i])>maxVal){
            flag = 1;
            maxVal = fabs(driveCmds[i]);
        }
    }
    if(flag){
        for(int i=0; i<arrayLen; i++){
            driveCmds[i] = driveCmds[i]*100.0/maxVal;
        }
    }
}

void loop(){
    t = timer.read_us()/1000000.0;
    loopCounter++;
    
    if((t-tPrev)>=PERIOD){
        dt = t - tPrev;
        tPrev = t;
        badTime = (dt>1.10f*PERIOD); //If more than 10% off
        
        getRCInputs();
        
        if(IMUCounter == 0){
            if(mpu.read(&gx,&gy,&gz,&ax,&ay,&az)){ // Get IMU readings, zero them. Raw is [dps, G]
                //Zero-ing
                gx -= gx_z;
                gy -= gy_z;
                gz -= gz_z;
                gx *= PI/180.0f; // convert to radians/second
                gy *= PI/180.0f;
                gz *= PI/180.0f;
                ax *= GRAV; // meters/second/second
                ay *= GRAV;
                az *= GRAV;
            }else{
                pcSerial.printf("IMU Failure %X", mpu.getID());
                mpu.start();
                IMUCounter = 1;
            }
            roll = gyro_comp*(roll + gx*dt) + (1.0f - gyro_comp)*atan2(ay, az);
            pitch = gyro_comp*(pitch + gy*dt) + (1.0f - gyro_comp)*atan2(ax, az);
            yaw = (yaw + gz*dt);
            rollDot = gx;
        }else{
            IMUCounter++; // This is in case the IMU fails, it restarts it
            if(IMUCounter == 100){
                    IMUCounter = 0;
                }
        }
        
        
        if(CONTROL_MODE == 0){ //Do nothing (Debug Mode)
            // Channels: LUD, RRL, RUD, LRL, Pot1, Pot2, LeftSwitch, Right trigger
            //----    Servo Out
            drive0.write(0.5);
            drive1.write(0.5);
            weapon.write(0.0);
        }else if(CONTROL_MODE==1){ //Seg
            //             0    1    2    3     4     5           6              7
            // Channels: LUD, RRL, RUD, LRL, Pot1, Pot2, LeftSwitch, Right trigger
            
            //Pre Scaling with Potentiometer
            // for(int i=0; i<4; i++){
            //     rcCommandInputs[i]*=rcCommandInputs[5];
            // }
            if(rcCommandInputs[7] == 1){
                rollDesired = -170.0F*DEG2RAD;
                kP = 300.0;
                kD = 0.0;
            }else{
                rollDesired = -20.0F*DEG2RAD - map(rcCommandInputs[2], -100 ,100, -75*DEG2RAD, 75*DEG2RAD) + map(rcCommandInputs[5], 0.0 ,1.0, -15*DEG2RAD, 15*DEG2RAD);
                kP = 60.0;
                kD = 7.0f;
            }

            if(rcCommandInputs[6] == 1){ //Commence seg
                segCmd = kP*(rollDesired - roll) + kD*(0.0f - gx);

                //----    Drive Out
                driveCmds[0] = segCmd + rcCommandInputs[2]*0.25 - rcCommandInputs[1]*0.25;
                driveCmds[1] = -segCmd - rcCommandInputs[2]*0.25 - rcCommandInputs[1]*0.25;

            }else{
                //----    Drive Out
                driveCmds[0] = rcCommandInputs[2] - rcCommandInputs[1]*0.25;
                driveCmds[1] = -rcCommandInputs[2] - rcCommandInputs[1]*0.25;
            }
            
            
            //Skid Steer Scaling
            scaleSkidSteer(2);
            
           //Final Scaling with Potentiometer
           for(int i=0; i<2; i++){
               driveCmds[i]*=rcCommandInputs[4];
           }
            
            drive0.write(map(-driveCmds[0],-100,100,1,0));
            drive1.write(map(-driveCmds[1],-100,100,1,0));
            weapon.write(rcCommandInputs[6]);
        }
        loopCounter = 0;
    }
    if((t-tPrevSerial)>= SERIAL_PERIOD){

        //Telemetry
        tPrevSerial = t;
        if(USESERIAL and !inInt and enabled==1){
            if(badTime){
//                pcSerial.printf("b%f\n",dt-PERIOD); //(dt-PERIOD)
                badTime = 0;
            }

        //    pcSerial.printf("%f, %f, %f, %f, %f, %f, %f, %f\r\n", rcCommandInputsRaw[0], rcCommandInputsRaw[1], rcCommandInputsRaw[2], rcCommandInputsRaw[3], rcCommandInputsRaw[4], rcCommandInputsRaw[5], rcCommandInputsRaw[6], rcCommandInputsRaw[7]);
        //    pcSerial.printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\r\n", rcCommandInputs[0], rcCommandInputs[1], rcCommandInputs[2], rcCommandInputs[3], rcCommandInputs[4], rcCommandInputs[5], rcCommandInputs[6], rcCommandInputs[7]);
        //    pcSerial.printf("%.4f,%.4f,%.4f,%.2f,%.2f,%.2f, %.1f\r\n",gx,gy,gz,ax,ay,az, w_cmd);
        //    pcSerial.printf("%.4f,%.4f,%.4f,%.4f\r\n", -0.5*rcCommandInputs[1],gz, w_cmd, w_int);
        //    pcSerial.printf("%f, %f, %f, \n", roll*RAD2DEG, pitch*RAD2DEG, yaw*RAD2DEG);
           pcSerial.printf("%f, %f, %f, %f, %f \n", rollDesired*RAD2DEG, roll*RAD2DEG, rollDot*RAD2DEG, segCmd, driveCmds[0]);
        //    pcSerial.printf("%f, %f, %f, ", rollDot, pitchDot, yawDot);
        //    pcSerial.printf("%f, %f, %f, %f, %f, %f, %f, %f, %i\n", ax, ay, az, sqrt(ax*ax + ay*ay + az*az));
        }
    }
}