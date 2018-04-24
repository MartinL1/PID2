# Under Construction

# PID2
PID Controller class that implements both the classic and "derivative on measurement" floating point PID controllers.

### __Version__

- Version V1.0.0 -- Intial release

### __Arduino Compatibility__

- All Arduino Boards

### __Installation__

After download simply un-zip the file and place the "PID2" directory in your ...Arduino/libraries folder. The Arduino folder is the one where your sketches are usually located.

### __Usage__

Simply include the PID2.h file at the beginning of your sketch:

**_#include <PID2.h>_**

The PID object is created (instantiated) declaring it, for example:

**_PID myPID;_**

The PID control loop is then initialised using the "begin" function together with the PID gain, as well as the integral and PID limit arguments:

**_myPID.begin(kp, ki, kd, iMinLimit, iMaxLimit, pidMinLimit, pidMaxLimit);_**

Alternatively, it's also possible to also pass the arguments to the constructor and forgo having to call the "begin" member function:

**_PID myPID(kp, ki, kd, iMinLimit, iMaxLimit, pidMinLimit, pidMaxLimit);_**

This PID library offers the classic P and PID, as well as "derivative on measurement" PID controllers.
 
The difference between the classic and derivative on measurement controllers, is the the classic controller calculates the derivate term by subtracting the previous error from the current error, before multiplying by the derivate gain "Kd" and dividing by the sample time "dt":

**_float dTerm = kd * (error - prevError) / dt;_**

Whereas the the derivative on measurement controller, subtracts the previous input from the current input:

**_float inputError = input - prevInput;_**
**_float dTerm = kd * -inputError / dt;_**

The purpose of the derivative on measurement controller is to remove the derivative kick that can sometimes happen due to fast changes in the setpoint.

To compute the PID output call either: pCalculate(), pidCalculate(), or for derivate on measurement: pidCalculateDOM() functions. For example the classic PID function take the setpoint, input and sample time "dt" as arguments:

**_myPID.pidCalculate(setpoint, input, dt);

The sample time dt is the time in seconds and must be externally supplied. This PID library doesn't provide the sample time itself, as there may be a number of PID instances that require the same sample time, or conversely a number that require differnet times.

To calculate the sample time using the Arduino micros() function

The kp, ki and kd arguments are float data types and represent the PID control loop's proportial (P), integral (I) and derivate (D) gains. The iLimit and pidLimit arguments are also float data types. ilimit prevents the integral term from the integral termprevents integral wind-up. The pidLimit

### __Example Code__

None.
