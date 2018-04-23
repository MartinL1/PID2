# Under Construction

# PID2
PID Controller class that implements both the classic and "derivative on measurement" PID controllers.

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

The PID control loop is then initialised using the "begin" function together with the PID gain and limit arguments:

**_myPID.begin(kp, ki, kd, iLimit, pidLimit);_**

The kp, ki and kd arguments are float data types and represent the PID control loop's proportial (P), integral (I) and derivate (D) gains. The iLimit and pidLimit arguments are also float data types. ilimit prevents the integral term from the integral termprevents integral wind-up. The pidLimit

### __Example Code__

None.
