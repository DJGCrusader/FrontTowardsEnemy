/*
Front Towards Enemy
December 2022

Authors: 
Dr. Daniel J. Gonzalez - dgonzrobotics@gmail.com

Design for NucleoL432KC
*/
#include "main.h"

using namespace FastMath;

void setupServo() {
    float myRange = .000675;
    float myDegrees = 90;
    drive0.calibrate(myRange, myDegrees);
    drive1.calibrate(myRange, myDegrees);
    drive2.calibrate(myRange, myDegrees);
    weapon.calibrate(myRange, myDegrees);
}

void setupIMU() {
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
    drive2.write(0.5);
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
    
    
    //Set up distance sensors
//    t1.write(0);
//    e1.rise(&e1IntRise);
//    e1.fall(&e1IntFall);
    
    enabled = 1;
}

int main() {
    setup();
    pcSerial.printf("----------- Loop Start! -----------\n");
    wait(0.5);
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
    jumpScale = rcCommandInputs[5];
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
                gx -= gx_z;
                gy -= gy_z;
                gz -= gz_z;
                gx *= PI/180.0f; // radians/second
                gy *= PI/180.0f;
                gz *= PI/180.0f;
                ax *= GRAV; // meters/second
                ay *= GRAV;
                az *= GRAV;
            }else{
                pcSerial.printf("IMU Failure %X", mpu.getID());
                mpu.start();
                IMUCounter = 1;
            }
        }else{
            IMUCounter++;
            if(IMUCounter == 100){
                    IMUCounter = 0;
                }
            }
        
        
        if(CONTROL_MODE == 0){ //Do nothing (Debug Mode)
            // Channels: LUD, RRL, RUD, LRL, Pot1, Pot2, LeftSwitch, Right trigger
            //----    Servo Out
            drive0.write(0.5);
            drive1.write(0.5);
            drive2.write(0.5);
        }else if(CONTROL_MODE==1){ //FPS Controls + Weapon
            //             0    1    2    3     4     5           6              7
            // Channels: LUD, RRL, RUD, LRL, Pot1, Pot2, LeftSwitch, Right trigger
            
            //Pre Scaling with Potentiometer
            for(int i=0; i<4; i++){
                rcCommandInputs[i]*=rcCommandInputs[5];
            }
            
//            w_int += (-0.5*rcCommandInputs[1]/(K_W) - gz)*dt;
            
            //Limit the integrator.
//            if (w_int > W_INT_LIMIT) w_int = W_INT_LIMIT;
//            if (w_int < -W_INT_LIMIT) w_int = -W_INT_LIMIT;
            
            w_cmd = -0.25*rcCommandInputs[1] + 2.0f*K_W*(-0.25*rcCommandInputs[1]/(K_W) - gz); // + 0.0f*w_int;
//            w_cmd *= rcCommandInputs[5];
            
            
            //----    Drive Out
            driveCmds[0] = w_cmd - (COS60)*rcCommandInputs[3] + 1.0f*rcCommandInputs[0]; // LRL+LUD
            driveCmds[1] = w_cmd - (COS60)*rcCommandInputs[3] - 1.0f*rcCommandInputs[0]; //COS30
            driveCmds[2] = w_cmd + rcCommandInputs[3]         + 0.0f*rcCommandInputs[0];
            
            //Skid Steer Scaling
            scaleSkidSteer(3);
            
//            //Final Scaling with Potentiometer
//            for(int i=0; i<3; i++){
//                driveCmds[i]*=rcCommandInputs[4];
//            }
            
            drive0.write(map(-driveCmds[0],-100,100,1,0));
            drive1.write(map(-driveCmds[1],-100,100,1,0));
            drive2.write(map(driveCmds[2],-100,100,1,0));
            weapon.write(rcCommandInputs[4]);
            
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
//            pcSerial.printf("%f, %f, %f, %f, %f, %f, %f, %f\r\n", rcCommandInputsRaw[0], rcCommandInputsRaw[1], rcCommandInputsRaw[2], rcCommandInputsRaw[3], rcCommandInputsRaw[4], rcCommandInputsRaw[5], rcCommandInputsRaw[6], rcCommandInputsRaw[7]);
//            pcSerial.printf("%.4f,%.4f,%.4f,%.2f,%.2f,%.2f, %.1f\r\n",gx,gy,gz,ax,ay,az, w_cmd);
//            pcSerial.printf("%.4f,%.4f,%.4f,%.4f\r\n", -0.5*rcCommandInputs[1],gz, w_cmd, w_int);
//            pcSerial.printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\r\n", rcCommandInputs[0], rcCommandInputs[1], rcCommandInputs[2], rcCommandInputs[3], rcCommandInputs[4], rcCommandInputs[5], rcCommandInputs[6], rcCommandInputs[7]);
//            pcSerial.printf("%i %.2f %.2f %.1f %.2f %.3f\n",state, l_states.h1.p*RAD2DEG, l_states.h1.v, l_states.h1.t, jumpScale, parabola_dt);
//            pcSerial.printf("%i %.3f %.3f\n",state, l_controls.h1.t_ff, l_states.h1.t);

//            pcSerial.printf("%6f %6f %6f\n",l_states.h1.v, vCalc, vCalc1); //Velocity Tuning

//            pcSerial.printf("%.3f %i %.3f %.3f %.1f %.3f %.1f\n",t, state, l_states.h1.p, l_states.h1.v, l_states.h1.t, l_controls.h1.p_des, l_controls.h1.t_ff);
//            pcSerial.printf("%.3f %.3f\n",l_states.h1.t,l_controls.h1.t_ff);

//            pcSerial.printf("%i %i %i%.3f %.1f %.3f %.3f %.1f %.1f %.3f\n", state, (int)rcCommandInputs[6], (int)rcCommandInputs[7], rcCommandInputs[5], l_controls.h1.p_des*RAD2DEG, l_controls.h1.t_ff, l_controls.h1.kp, l_states.h1.p*RAD2DEG, l_states.h1.v*RAD2DEG, l_states.h1.t);
//            pcSerial.printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, \n", rcCommandInputs[0], rcCommandInputs[1], rcCommandInputs[3], rcCommandInputs[4], rcCommandInputs[5], odrvCmds[0], odrvCmds[1], odrvCmds[2], odrvCmds[3]);
//            pcSerial.printf("%f, %f, %f, \n", roll, pitch, yaw);
//            pcSerial.printf("%f, %f, %f, ", rollDot, pitchDot, yawDot);
//            pcSerial.printf("%f, %f, %f, %f, %f, %f, %f, %f, %i\n",
//                            accX, accY, accZ, accXAvg, accYAvg, accZAvg,
//                            sqrt(accX*accX + accY*accY + accZ*accZ), sqrt(accXAvg*accXAvg + accYAvg*accYAvg + accZAvg*accZAvg), state);
//            pcSerial.printf("%f, %f, %f, \n", roll, pitch, yaw);
            

//            pcSerial.printf("%f, %f, ", roll, pitch);
//            pcSerial.printf("%f, %f, ", rollDot, pitchDot);
//            pcSerial.printf("%f, %f, %f, %f ", odrvCmds[0], odrvCmds[1], odrvCmds[2], odrvCmds[3]);
//            pcSerial.printf("%f, %i \n", sqrt(accX*accX + accY*accY + accZ*accZ), state);
            
//            pcSerial.printf("%f, %f, %f, %f, %f, %f, %f, %f, %i\n",
//                            accX, accY, accZ, accXAvg, accYAvg, accZAvg,
//                            sqrt(accX*accX + accY*accY + accZ*accZ), sqrt(accXAvg*accXAvg + accYAvg*accYAvg + accZAvg*accZAvg), state);
        }
    }
}

//void e1IntRise(){
//    t_e1 = timer.read_us();
//    inInterrupt1 = 1;
//}
//
//void e1IntFall(){
//    int pulseWidth = (timer.read_us() - t_e1);
//    
//    if( pulseWidth < 23200){ // If within reasonable range
//        d_e1 = pulseWidth / 5800.0f; // meters
//    }
//    
//    inInterrupt1 = 0;
//    pulse1 = 0;
//}