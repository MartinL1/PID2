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
		PID(float kp, float pLimit);																			// P controller constructor
		PID(float kp, float ki, float kd, float iLimit, float pidLimit);	// PID controller constructor
		void begin(float kp, float pLimit);																// Initialise the P controller
		void begin(float kp, float ki, float kd, 													// Initialise the PID controller
			float iLimit, float pidLimit);																								
		float pCalculate(float setpoint, float input);										// Calculate the classic P control loop
		float pidCalculate(float setpoint, float input, float dt);				// Calculate the classic PID control loop
		float pidCalculateDOM(float setpoint, float input, float dt);			// Calculate the "Derivative on Measurement" PID control loop
		void setKp(float kp);																							// Set the proportional P gain
		void setKi(float ki);																							// Set the integral I gain
		void setKd(float kd);																							// Set the derivative D gain
		void setILimit(float iLimit);																			// Set the integral I limit to prevent integral wind-up
		void setPIDLimit(float pidLimit);																	// Set the PID controller output limit
		void setPrevError(float prevError);																// Set the previous error value
		void setPrevInput(float prevInput);																// Set the previous input value
	//private:																													
		float kp, ki, kd;																									// PID gain terms
		float iTerm;																											// The summing integral I term
		float prevError, prevInput;																				// Previous error and input used to calculate the derivative D term										
		float iLimit, pidLimit;																						// I and PID output limits
};

#endif
