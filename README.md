# AutoPID
Arduino AutoPID library
- [AutoPID](#autopid)
- [Installation](#installation)
	- [Via Arduino IDE Library Manager](#via-arduino-ide-library-manager)
- [Documentation](#documentation)
	- [AutoPID Functions](#autopid-functions)
		- [AutoPID::AutoPID Constructor](#autopidautopid-constructor)
		- [AutoPID::setGains](#autopidsetgains)
		- [AutoPID::setBangBang](#autopidsetbangbang)
		- [AutoPID::setOutputRange](#autopidsetoutputrange)
		- [AutoPID::setTimeStep](#autopidsettimestep)
		- [AutoPID::atSetPoint](#autopidatsetpoint)
		- [AutoPID::run](#autopidrun)
		- [AutoPID::stop](#autopidstop)
		- [AutoPID::reset](#autopidreset)
		- [AutoPID::isStopped](#autopidisstopped)
	- [AutoPIDRelay Functions](#autopidrelay-functions)
		- [AutoPIDRelay::AutoPIDRelay Constructor](#autopidrelayautopidrelay-constructor)
		- [AutoPIDRelay::getPulseValue](#autopidrelaygetpulsevalue)
- [Examples](#examples)
 
# Installation
## Via Arduino IDE Library Manager
**Sketch** -> **Include Library** -> **Manage Libraries...** -> search for "autopid"
![Arduino Library Manager screen](https://i.imgur.com/w2jX1HL.png)

# Documentation

## AutoPID Functions

### AutoPID::AutoPID Constructor
> Creates a new AutoPID object
```cpp
AutoPID(double *input, double *setpoint, double *output, 
  double outputMin, double outputMax, 
  double Kp, double Ki, double Kd)
```
 - `input`, `setpoint`, and `output` are pointers to the variables holding these values. When they are changed in elswhere in the program, the PID updates itself on the next calculation. 
   - `input` and `setpoint` should be in the same units.
 - `outputMin` and `outputMax` are the range of values your output can be set to
   - They should use the same units as `output`
 - `Kp`, `Ki`, and `Kd` are the PID proportional, integral, and derivative gains.
 
### AutoPID::setGains
> Manual adjustment of PID gains
```cpp
void setGains(double Kp, double Ki, double Kd)
```
 - `Kp`, `Ki`, and `Kd` are the PID proportional, integral, and derivative gains.
 
### AutoPID::setBangBang
> Set the bang-bang control thresholds
```cpp
void setBangBang(double bangOn, double bangOff)
```
 - `bangOn`, `bangOff` are the upper and lower offsets from the setpoint. 
   - If input is below `(setpoint - bangOn)` the PID will set `output` to `outputMax`
   - If input is above `(setpoint + bangOff)` the PID will set `output` to `outputMin`
 
```cpp
void setBangBang(double bangRange)
```
 - `bangRange` is the absolute offset from the setpoint. 
   - If input is below `(setpoint - bangRange)` the PID will set `output` to `outputMax`
   - If input is above `(setpoint + bangRange)` the PID will set `output` to `outputMin`
 
### AutoPID::setOutputRange
> Manual (re)adjustment of output range
```cpp
void setOutputRange(double outputMin, double outputMax)
```
- `outputMin` and `outputMax` are the range of values your output can be set to
  - They should use the same units as `output`
   
### AutoPID::setTimeStep
> Manual adjustment of PID time interval for calculations
```cpp
void setTimeStep(unsigned long timeStep)
```
 - `timestep` is the time interval at which PID calculations are allowed to run in milliseconds. Default is *1000*
  
### AutoPID::atSetPoint
> Indicates if input has reached desired setpoint
```cpp
bool atSetPoint(double threshold)
```
 - `threshold` is the absolute offset from setpoint that the input should be at
 - `return` *true* if input is within +-(`threshold`) of `setpoint`
  
### AutoPID::run
> Automatically runs PID calculations at certain time interval. Reads input, setpoint, and updates output
```cpp
void run()
```
 - Should be called repeatedly from loop. Will only actually perform calculations when time interval has passed.
 
### AutoPID::stop
> Stops PID calculations and resets internal PID calculation values (integral, derivative)
```cpp
void stop()
```
 - Can be resumed with `run()`

### AutoPID::reset
> Resets internal PID calculation values (integral, derivative)
```cpp
void reset()
```
 - Only clears current calculations, does not stop running

### AutoPID::isStopped
> Indicates if PID calculations have been stopped
```cpp
bool isStopped()
```
 - `return` *true* if PID has been stopped
 
## AutoPIDRelay Functions

### AutoPIDRelay::AutoPIDRelay Constructor
```cpp
AutoPIDRelay(double *input, double *setpoint, bool *relayState,
  double pulseWidth, double Kp, double Ki, double Kd)
```
 - `input`, `setpoint`, and `relayState` are pointers to the variables holding these values. When they are changed in elswhere in the program, the PID updates itself on the next calculation.
 - `pulseWidth` is the PWM pulse witdh in milliseconds
 - `Kp`, `Ki`, and `Kd` are the PID proportional, integral, and derivative gains.

### AutoPIDRelay::getPulseValue
```cpp
double getPulseValue();
```
 - `return` the current pulse length. Relay state is already managed internally, but this can be used as well


# Examples
