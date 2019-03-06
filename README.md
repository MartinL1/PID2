# PID2
PID Controller class that implements both the classic and "derivative on measurement" floating point PID controllers.

The PID Controller was designed for the various control loops on the Falcon 1, 2 and 3 multi-rotor (drone) flight controllers:

<img src="https://static.rcgroups.net/forums/attachments/6/0/1/5/8/7/a10249648-147-Falcon2BlackSplash.jpg" alt="Falcon 2 Flight Controller" width="50%" height="50%"><img src="https://static.rcgroups.net/forums/attachments/6/0/1/5/8/7/a11105699-25-Falcon3%20Mode%20Settings.png" alt="Falcon 3 Flight Controller" width="50%" height="50%">

## __Version__

- Version V1.0.0 -- Intial release

## __Arduino Compatibility__

- All Arduino Boards

## __Installation__

After download simply un-zip the file and place the "PID2" directory in your ...Arduino/libraries folder. The Arduino folder is the one where your sketches are usually located.

## __Usage__

### __PID2 Library__

Simply include the PID2.h file at the beginning of your sketch:

```
#include <PID2.h>
```
---
### __Initialisation___

The PID object is created (instantiated) by declaring it, for example:

```
PID myPID;
```

The PID control loop is then initialised using the "begin" function together with the PID gains, as well as the integral and PID limit arguments:

```
myPID.begin(kp, ki, kd, iMinLimit, iMaxLimit, pidMinLimit, pidMaxLimit);
```

The purpose of the integral gain limits is to prevent integral wind-up. Integral wind-up can occur if the system isn't able to, or doesn't resposed quickly enough to a change in the setpoint. In this condition the integral term can steady grow to a large value and disrupt the control system. The PID controller's output limits are also be bounded limits.

Alternatively, it's also possible to also pass the arguments to the constructor and forgo having to call the "begin" member function:

```
PID myPID(kp, ki, kd, iMinLimit, iMaxLimit, pidMinLimit, pidMaxLimit);
```
---
### Control Loop Options

This PID library offers the classic P and PID, as well as "derivative on measurement" PID controllers.
 
The difference between the classic and derivative on measurement controllers, is the the classic controller calculates the derivate term by subtracting the previous error from the current error, before multiplying by the derivate gain "Kd" and dividing by the sample time "dt":

```
float dTerm = kd * (error - prevError) / dt;
```

Whereas the derivative on measurement controller, subtracts the previous input from the current input:

```
float inputError = input - prevInput;  
float dTerm = kd * -inputError / dt;
```

The purpose of the derivative on measurement controller is to remove the derivative kick that can sometimes happen due to fast changes in the setpoint.

To compute the PID output call either the pCalculate(), pidCalculate(), or for derivate on measurement the pidCalculateDOM() member functions. For example the classic PID function takes the setpoint, input and sample time "dt" as arguments:

```
float pidOutput = myPID.pidCalculate(setpoint, input, dt);
```

The sample time dt is the time in seconds and must be externally supplied. 

This PID library doesn't provide the sample time itself, as there may be a number of PID controller instances that require the same sample time, or conversely a number of instances that require differnet times.

To calculate the sample time using the Arduino micros() function:

```
uint32_t timeMicros = micros();  
dt = (timeMicros - lastTime) / 1.0e6f;  
lastTime = timeMicros;
```

where "lastTime" is a **_uint32_t_** data type and "dt" is a **_float_**.

The simpler P controller doesn't require the "dt" sample time and only the setpoint and input are necessary:

```
float pidOutput = myPID.pCalculate(setpoint, input);
```
---
### __Example Code__

None.
