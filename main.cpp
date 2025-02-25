/*
Front Toward Enemy
2023

Authors: 
Daniel J. Gonzalez - dgonzrobotics@gmail.com

Designed for NucleoL432KC
*/
#include "main.h"
#include <cstdlib>

using namespace FastMath;

void setupServo() {
    float myRange = .000500; //0.000675
    float myDegrees = 90;
    drive0.calibrate(myRange, myDegrees);
    drive1.calibrate(myRange, myDegrees);
    drive2.calibrate(myRange, myDegrees);
    weapon.calibrate(myRange, myDegrees);
}

void setupIMU() {
    
    roll = 0;
    pitch = 0; 
    yaw = 0;
    rollDot = 0; 
    pitchDot = 0;
    yawDot = 0;
    gyro_comp = 0.8;

    if(mpu.getID()==0x68) {
        printf("MPU6050 OK\n");
        //  wait(1);
        ThisThread::sleep_for(1s);
     } else {
        printf("MPU6050 error ID=0x%x\r\n",mpu.getID());
     }
     mpu.start();
     printf("Zeroing in 5...");
     //  wait(1);
     ThisThread::sleep_for(1s);
     printf("4...");
    //  wait(1);
     ThisThread::sleep_for(1s);
     printf("3...");
    //  wait(1);
     ThisThread::sleep_for(1s);
     printf("2...");
    //  wait(1);
     ThisThread::sleep_for(1s);
     printf("1...");
    //  wait(1);
     ThisThread::sleep_for(1s);
     printf("Go!\n");
     gx_z = 0;
     gy_z = 0;
     gz_z = 0;
     for(int i = 0; i<100; i++){
         mpu.read(&gx,&gy,&gz,&ax,&ay,&az);
         ThisThread::sleep_for(10ms);
         gx_z += gx;
         gy_z += gy;
         gz_z += gz;
     }
     gx_z /= 100.0f;
     gy_z /= 100.0f;
     gz_z /= 100.0f;
     printf("Zeroing Complete.\n");
}

void setup(){
    pcSerial.set_baud(115200);
    printf("----------- Start! -----------\n");
    
    //Create instance of PPM class
    //Pass in interrupt pin, minimum output value, maximum output value, minimum pulse time from transmitter, maximum pulse time from transmitter, number of channels, throttle channel (used for failsafe)
    ppmInputs = new PPM(PPMinterruptPin, 0, 1, 1000, 1900, 8, 3);
    printf("PPM Set Up\n");
    setupIMU();
    printf("IMU Set Up\n");

    //------------------------------    Servo Out
    setupServo();  
    drive0.write(0.5);
    drive1.write(0.5);
    drive2.write(0.5);
    weapon.write(0.0);
    printf("Servos Set Up\n");
    
    //set up timer
    timer.start();
    tPrev = timer.read_us()/1000000.0;
    t = timer.read_us()/1000000.0;
    dt = t - tPrev;
    printf("Timer Set Up\n"); 
    
    enabled = 1;
}

int main() {
    setup();
    printf("----------- Loop Start! -----------\n");
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
        }else if(i==3){ //if LRL
            rcCommandInputs[i] = map(rcCommandInputs[i], LRL_MIN, LRL_MAX,-100,100);
        }else if(i==4 || i==5){ //if Pot1 or Pot 2
            rcCommandInputs[i] = map(rcCommandInputs[i], LRL_MIN, LRL_MAX,0.0,1.0); //Do scaling factor for potentiometers
        }else if(i == 6 || i == 7){
            rcCommandInputs[i] = rcCommandInputs[i] > (LRL_MAX/2); //Toggle Switches sendityeet
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
//        vCalc1 = (l_states.h1.p - prevPos1)/dt;
//        prevPos1 = l_states.h1.p;
        
        tPrev = t;
        badTime = (dt>1.10f*PERIOD); //If more than 10% off
        
        getRCInputs();
        
//        if(!inInterrupt1){
//            t1.write(1);
//            pulse1 = 1;
//            wait_us(10);
//            t1.write(0);
//        }

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

                // if(gy == gy_prev){
                //     printf("IMU Failure %X\n", mpu.getID());
                //     IMUCounter = 1;
                // }
            }else{
                printf("IMU Failure %X\n", mpu.getID());
                IMUCounter = 1;
            }
            roll = (roll + gx*dt);
            pitch = (pitch + gy*dt);
            yaw = (yaw + gz*dt);
        }else{
            mpu.start();
            IMUCounter = 0;
        }
        
        
        if(CONTROL_MODE == 0){ //Do nothing (Debug Mode)
            // Channels: LUD, RRL, RUD, LRL, Pot1, Pot2, LeftSwitch, Right trigger
            //----    Servo Out
            drive0.write(0.5);
            drive1.write(0.5);
            drive2.write(0.5);
            weapon.write(0.0);
        }else if(CONTROL_MODE==1){ //FPS Controls + Weapon
            //             0    1    2    3     4     5           6              7
            // Channels: LUD, RRL, RUD, LRL, Pot1, Pot2, LeftSwitch, Right trigger
            
            //Pre Scaling of sticks with Potentiometer
            for(int i=0; i<4; i++){
                rcCommandInputs[i]*=rcCommandInputs[5];
            }
            
//            w_int += (-0.5*rcCommandInputs[1]/(K_W) - gz)*dt;
            
            //Limit the integrator.
//            if (w_int > W_INT_LIMIT) w_int = W_INT_LIMIT;
//            if (w_int < -W_INT_LIMIT) w_int = -W_INT_LIMIT;

            w_cmd = -0.375*rcCommandInputs[1] - KP_W*K_W*(0.125*rcCommandInputs[1]/(K_W) - gy); // + 0.0f*w_int;

            //Failsafe for controller instability
            if((rcCommandInputs[5]< 0.1f) || (rcCommandInputs[1] < 10.0f && rcCommandInputs[1] > -10.0f)){
                w_cmd = -0.375*rcCommandInputs[1];
            }

            //----    Drive Out [-100 to 100]
            driveCmds[0] = w_cmd - (COS60)*rcCommandInputs[3] + 1.0f*rcCommandInputs[0]; // LRL+LUD
            driveCmds[1] = w_cmd - (COS60)*rcCommandInputs[3] - 1.0f*rcCommandInputs[0]; //COS30
            driveCmds[2] = w_cmd +         rcCommandInputs[3] + 0.0f*rcCommandInputs[0];

            //----    Weapon out [0 to 1]
            weaponComp = 0.0;
            if(fabs(gy)>2.285){ //2.285 is the max allowable turn rate without decelerating the weapon at all
                weaponComp = (fabs(gy) - 2.285)/(0.3*34.9 - 2.285); 
                //34.9 rad/s is max measurable gyro rate 
                //0.3 is the scaling for when to kick in full weaponComp
                //gz when flat, gz when up on side
                if (weaponComp>0.666) weaponComp = 0.666; //limit weapon compensation
            }
            weaponCmd = rcCommandInputs[6]*(0.333+0.666*rcCommandInputs[4] - weaponComp); 
            
            //Skid Steer Scaling
            scaleSkidSteer(3);
            
//            //Final Scaling with Potentiometer
//            for(int i=0; i<3; i++){
//                driveCmds[i]*=rcCommandInputs[4];
//            }
            drive0.write(map(driveCmds[0],-100,100,0,1));
            drive1.write(map(driveCmds[1],-100,100,0,1));
            drive2.write(map(driveCmds[2],-100,100,0,1));
            weapon.write(map(weaponCmd, 0, 1, 0.5, 0)); //Unidirectional for QWINOUT, bidirectional for BotBitz
            //weapon.write(map(weaponCmd, 0, 1, 0.5, 1.0)); //Unidirectional for QWINOUT, bidirectional for BotBitz
            
        }
        loopCounter = 0;
        gy_prev = gy;
    }
    if((t-tPrevSerial)>= SERIAL_PERIOD){

        //Telemetry
        tPrevSerial = t;
        if(USESERIAL and !inInt and enabled==1){
            if(badTime){
//                printf("b%f\n",dt-PERIOD); //(dt-PERIOD)
                badTime = 0;
            }

        //    printf("%f, %f, %f, %f, %f, %f, %f, %f\r\n", rcCommandInputsRaw[0], rcCommandInputsRaw[1], rcCommandInputsRaw[2], rcCommandInputsRaw[3], rcCommandInputsRaw[4], rcCommandInputsRaw[5], rcCommandInputsRaw[6], rcCommandInputsRaw[7]);
        //    printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\r\n", rcCommandInputs[0], rcCommandInputs[1], rcCommandInputs[2], rcCommandInputs[3], rcCommandInputs[4], rcCommandInputs[5], rcCommandInputs[6], rcCommandInputs[7]);
        //    printf("%.3f, %.3f, %.3f, %.3f\r\n", map(rcCommandInputs[4], 0, 1, 0.5, 1),  map(-driveCmds[0],-100,100,1,0), map(-driveCmds[1],-100,100,1,0), map(-driveCmds[2],-100,100,1,0));
        //    printf("%.4f,%.4f,%.4f,%.2f,%.2f,%.2f, %.1f, %.3f\r\n",gx,gy,gz,ax,ay,az, w_cmd, 100*weaponComp);
        printf("%.1f, %.3f, %.3f, %.3f\r\n",w_cmd, 100*weaponComp, weaponCmd, w_cmd);
        //    printf("%.4f,%.4f,%.4f,%.4f\r\n", -0.5*rcCommandInputs[1],gz, w_cmd, w_int);
        //    printf("%f, %f, %f, \n", roll, pitch, yaw);
        //    printf("%f, %f, %f, ", rollDot, pitchDot, yawDot);
        //    printf("%f, %f, %f, %f, %f, %f, %f, %f, %i\n", ax, ay, az, sqrt(ax*ax + ay*ay + az*az));
        }
    }
}