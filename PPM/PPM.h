#include "mbed.h"

#ifndef PPM_H
#define PPM_H

class PPM
{
    public:
        //Constructor
        PPM(InterruptIn *ppmPin, float minimumOutput, float maximumOutput, int minimumPulseTime, int maximumPulseTime, int numberOfChannels, int throttleChannel);
        
    private:
        //Interrupt
        void SignalRise();
        
        //Interrupt pin
        InterruptIn *_ppmPin;
        
        //Timer, times length of pulses
        Timer _timer;
        //Number of channels in PPM signal
        int _numberOfChannels;
        //Current channel
        char _currentChannel;  
        //Stores channel times
        int _times[100];
        //Stores most recent complete frame times
        int _completeTimes[100];
        //Keeps track of time between PPM interrupts
        int _timeElapsed; 
        //Minimum time of frame
        int _minFrameTime;
        //If the pulse time for a channel is this short, something is wrong uS
        int _shortTime;
        //Minimum pulse time uS
        int _minimumPulseTime;
        //Maximum pulse time uS
        int _maximumPulseTime;
        //Minimum output
        float _minimumOutput;
        //Maximum output
        float _maximumOutput;
        //Throttle channel - used for fail safe
        int _throttleChannel;
        
    public:
        //Get channel data
        void GetChannelData(float * channelData);
        
    private:
        float Map(float input, float inputMin, float inputMax, float outputMin, float outputMax);
};

#endif


