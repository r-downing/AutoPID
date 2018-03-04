#ifndef AUTOPID_H
#define AUTOPID_H
#include <Arduino.h>

class AutoPID {

  public:
    // Constructor - takes pointer inputs for control variales, so they are updated automatically
    AutoPID(double *input, double *setpoint, double *output, double outputMin, double outputMax,
            double Kp, double Ki, double Kd);
    // Allows manual adjustment of gains
    void setGains(double Kp, double Ki, double Kd);
    // Sets bang-bang control ranges, separate upper and lower offsets, zero for off
    void setBangBang(double bangOn, double bangOff);
    // Sets bang-bang control range +-single offset
    void setBangBang(double bangRange);
    // Allows manual readjustment of output range
    void setOutputRange(double outputMin, double outputMax);
    // Allows manual adjustment of time step (default 1000ms)
    void setTimeStep(unsigned long timeStep);
    // Returns true when at set point (+-threshold)
    bool atSetPoint(double threshold);
    // Runs PID calculations when needed. Should be called repeatedly in loop.
    // Automatically reads input and sets output via pointers
    void run();
    // Stops PID functionality, output sets to 
    void stop();
    void reset();
    bool isStopped();

    double getIntegral();
    void setIntegral(double integral);

  private:
    double _Kp, _Ki, _Kd;
    double _integral, _previousError;
    double _bangOn, _bangOff;
    double *_input, *_setpoint, *_output;
    double _outputMin, _outputMax;
    unsigned long _timeStep, _lastStep;
    bool _stopped;

};//class AutoPID

class AutoPIDRelay : public AutoPID {
  public:

    AutoPIDRelay(double *input, double *setpoint, bool *relayState, double pulseWidth, double Kp, double Ki, double Kd)
      : AutoPID(input, setpoint, &_pulseValue, 0, 1.0, Kp, Ki, Kd) {
      _relayState = relayState;
      _pulseWidth = pulseWidth;
    };

    void run();

    double getPulseValue();

  private:
    bool * _relayState;
    unsigned long _pulseWidth, _lastPulseTime;
    double _pulseValue;
};//class AutoPIDRelay

#endif
