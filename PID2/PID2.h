/*
  PID Controller class that implements both the classic and the 
	"derivative on measurement" PID controllers.
	
	Copyright (C) Martin Lindupp 2018
	
	V1.0.0 -- Initial release 									

	The MIT License (MIT)

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#ifndef _PID2_h
#define _PID2_h

// PID Controller class that implements both the classic and the "derivative on measurement" PID controllers
class PID
{
	public:																														
		PID();																														// Default constructor
		PID(float kp, float pMinLimit, float pMaxLimit);									// P controller constructors
		PID(float kp, float ki, float kd, float iMinLimit, 								// PID controller constructors
			float iMaxLimit, float pidMinLimit, float pidMaxLimit);
		void begin(float kp, float pMinLimit, float pMaxLimit);						// Initialise the P controller
		void begin(float kp, float ki, float kd, float iMinLimit,					// Initialise the PID controller
			float iMaxLimit, float pidMinLimit, float pidMaxLimit);
		float pCalculate(float setpoint, float input);										// Calculate the classic P control loop
		float pidCalculate(float setpoint, float input, float dt);				// Calculate the classic PID control loop
		float pidCalculateDOM(float setpoint, float input, float dt);			// Calculate the "Derivative on Measurement" PID control loop
		void setKp(float kp);																							// Set the proportional P gain
		void setKi(float ki);																							// Set the integral I gain
		void setKd(float kd);																							// Set the derivative D gain
		void setILimits(float iMinLimit, float iMaxLimit);								// Set the integral I minimum and maximum limits to prevent integral wind-up																																				
		void setPIDLimits(float pidMinLimit, float pidMaxLimit);					// Set the PID controller minimum and maximum output limits
		void setIMinLimit(float iMinLimit);																// Set the integral I minimum limit
		void setIMaxLimit(float iMaxLimit);																// Set the integral I maximum limit
		void setPIDMinLimit(float pidMinLimit);														// Set the PID controller minimum output limit
		void setPIDMaxLimit(float pidMaxLimit);														// Set the PID controller maximum output limit
		void setPrevError(float prevError);																// Set the previous error value
		void setPrevInput(float prevInput);																// Set the previous input value
		float getKp();																										// Get the proportional P gain
		float getKi();																										// Get the integral I gain
		float getKd();																										// Get the derivative D gain
		float getILimit();																								// Get the integral limits
		float getPIDLimit();																							// Get the PID output limits
		float getIMinLimit();																							// Get the integral minimum limit
		float getIMaxLimit();																							// Get the integral maximum limit
		float getPIDMinLimit();																						// Get the PID output minimum limit
		float getPIDMaxLimit();																						// Get the PID output maximum limit
		float getPrevError();																							// Get the previous error
		float getPrevInput();																							// Get the previous input
	//private:																													
		float kp, ki, kd;																									// PID gain terms
		float iTerm;																											// The summing integral I term
		float prevError, prevInput;																				// Previous error and input used to calculate the derivative D term										
		float iMinLimit, iMaxLimit, pidMinLimit, pidMaxLimit;							// Integral I and PID minimum and maximum output limits
};

#endif
