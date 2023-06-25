#include "PPM.h"    

//PPM reader by Joe Roberts, based on work by John Wolter
//This program takes the PPM Signal (Pulse Position Modulation) from your RC transmitter and outputs the data between the min/max outputs passed into the constructor
//See 
PPM::PPM(InterruptIn *ppmPin, float minimumOutput, float maximumOutput, int minimumPulseTime, int maximumPulseTime, int numberOfChannels, int throttleChannel) 
{    
    //Assign local variables passed into constructor
    _ppmPin = ppmPin;
    _minimumOutput = minimumOutput;
    _maximumOutput = maximumOutput;
    _minimumPulseTime = minimumPulseTime;
    _maximumPulseTime = maximumPulseTime;
    _numberOfChannels = numberOfChannels;
    _throttleChannel = throttleChannel;
        
    //Set other variables
    _currentChannel = 0;  
    _timeElapsed = 0; 
    _minFrameTime = 6000;
    _shortTime = 800;

    //Initialise arrays
    for(int i = 0; i < _numberOfChannels; i++)
    {
        _times[i] = 0;
        _completeTimes[i] = 0;
    }
    
    //Assign interrupt
    _ppmPin->mode (PullUp);
    _ppmPin->rise (this, &PPM::SignalRise);

    //Start timer
    _timer.start();
}
   
//Here is where all the work decoding the PPM signal takes place
void PPM::SignalRise()
{
    //Get the time taken since the last interrupt
    _timeElapsed = _timer.read_us();
    
    //If time is less than _shortTime then the channel timing is too short - ignore
    if (_timeElapsed < _shortTime) return;
    
    //Disable the interrupt
    _ppmPin->rise(NULL);

    //Reset the timer
    _timer.reset();
    
    //Check for a new frame signal, if before start of new frame then its a glitch - start a new frame
    if ((_timeElapsed > _minFrameTime) && (_currentChannel != 0)) _currentChannel = 0;
    
    //Check for a new frame signal, if it is the start of a new frame then start new frame
    if ((_timeElapsed > _minFrameTime ) && (_currentChannel == 0))
    {
        //Assign interrupt
        _ppmPin->rise (this, &PPM::SignalRise);
        return;
    }
 
    //Save the time to the times array
    _times[_currentChannel] = _timeElapsed;
    _currentChannel++;
    
    //Check for a complete frame
    if (_currentChannel == _numberOfChannels)
    {
        //Set channel iterator to 0
        _currentChannel = 0;
        //Copy times array to complete times array
        memcpy(_completeTimes, _times, sizeof(_times));
    }

    //Assign interrupt
    _ppmPin->rise(this, &PPM::SignalRise);
    return;
}

//Place mapped channel data into the passed in array
void PPM::GetChannelData(float * channelData)
{
    //Iterate over the channel times array
    for(int i = 0; i < _numberOfChannels; i++)
    {
        //Check the transmitter is still connected by checking the thottle
        if((i == _throttleChannel - 1) && (_completeTimes[i] < _minimumPulseTime)) channelData[i] = -1;
        else
        {
            //Map the channel times to value between the channel min and channel max
            channelData[i] = Map(_completeTimes[i] ,_minimumPulseTime, _maximumPulseTime, _minimumOutput, _maximumOutput);
        }
    }
    
    return; 
}

float PPM::Map(float input, float inputMin, float inputMax, float outputMin, float outputMax)
{
    return (input - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
}