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

#include <PID2.h>

// Default constructor
PID::PID() {}

// P Controller constructor
PID::PID(float kp, float pLimit) : kp(kp), pidLimit(pidLimit) {}

// PID Controller constructor
PID::PID(float kp, float ki, float kd, float iLimit, float pidLimit) : 
	kp(kp), ki(ki), kd(kd), iLimit(iLimit), pidLimit(pidLimit) {}

// Initialise the P controller
void PID::begin(float kp, float pidLimit)
{
	this->kp = kp;																						// Set the proportional P gain to the new value
	this->pidLimit = pidLimit;																// Set the proportional P output limit
}

// Initialise the PID controller
void PID::begin(float kp, float ki, float kd, float iLimit, float pidLimit)
{
	this->kp = kp;																						// Set the proportional P gain to the new value
	this->ki = ki;																						// Set the integral I gain to the new value
	this->kd = kd;																						// Set the derivative D gain to the new value
	this->iLimit = iLimit;																		// Set the integral I output limit
	this->pidLimit = pidLimit;																// Set the combined PID output limit
}
	
// Calculate the classic P control loop
float PID::pCalculate(float setpoint, float input)
{
  float error = setpoint - input;														// Calculate the error = setpoint - input
	float pTerm = kp * error;                                 // pTerm = (setpoint - input) * proportional gain 
  pTerm = pTerm > pidLimit ? pidLimit : pTerm;              // Check that the positive output limit is not exceeded
  pTerm = pTerm < -pidLimit ? -pidLimit : pTerm;            // Check that the negative output limit is not exceeded
  return pTerm;                                             // Return the PID term
}

// Calculate the classic PID control loop 
float PID::pidCalculate(float setpoint, float input, float dt)
{
  float error = setpoint - input;														// Calculate the error = setpoint - input
	float pTerm = kp * error;                                 // pTerm = (setpoint - input) * proportional gain 
  iTerm += ki * error * dt;                                 // iTerm += (setpoint - input) * integral gain * loop time (dt)
  iTerm = iTerm > iLimit ? iLimit : iTerm;                  // Check that the positive integral limit is not exceeded
  iTerm = iTerm < -iLimit ? -iLimit : iTerm;                // Check that the negative integral limit is not exceeded
  float dTerm = kd * (error - prevError) / dt;              // dTerm = derivative gain * (current error - previous error) / loop time (dt)
  float pidTerm = pTerm + iTerm + dTerm;                    // Sum the PID terms
  pidTerm = pidTerm > pidLimit ? pidLimit : pidTerm;        // Check that the positive output limit is not exceeded
  pidTerm = pidTerm < -pidLimit ? -pidLimit : pidTerm;      // Check that the negative output limit is not exceeded
	prevError = error;																				// Update the previous error value
  return pidTerm;                                           // Return the PID term
}

// Calculate the "Derivative on Measurement" PID control loop with the input error (rather than error - prevError) as the derivative
// This removes the "derivative kick" induced by a sudden change in the setpoint
float PID::pidCalculateDOM(float setpoint, float input, float dt)
{
  float error = setpoint - input;														// Calculate the error = setpoint - input
	float pTerm = kp * error;                                 // pTerm = (setpoint - input) * proportional gain 
  iTerm += ki * error * dt;                                 // iTerm += (setpoint - input) * integral gain * loop time (dt)
  iTerm = iTerm > iLimit ? iLimit : iTerm;                  // Check that the positive integral limit is not exceeded
  iTerm = iTerm < -iLimit ? -iLimit : iTerm;                // Check that the negative integral limit is not exceeded
  float inputError = input - prevInput;											// Calculate the input error = input - previous input
	float dTerm = kd * -inputError / dt;                      // dTerm = derivative gain * (-inputError) / loop time (dt)
  float pidTerm = pTerm + iTerm + dTerm;                    // Sum the PID terms
  pidTerm = pidTerm > pidLimit ? pidLimit : pidTerm;        // Check that the positive output limit is not exceeded
  pidTerm = pidTerm < -pidLimit ? -pidLimit : pidTerm;      // Check that the negative output limit is not exceeded
  prevInput = input;																				// Update the previous input
	return pidTerm;                                           // Return the PID term
}

// Set the proportional P gain
void PID::setKp(float kp)
{
	this->kp = kp;																						// Set the proportional P gain
}

// Set the integral I gain
void PID::setKi(float ki)
{
	this->ki = ki;																						// Set the integral I gain
}

// Set the derivative D gain
void PID::setKd(float kd)
{
	this->kd = kd;																						// Set the derivative D gain
}

// Set the integral I limit to prevent integral wind-up
void PID::setILimit(float iLimit)
{
	this->iLimit = iLimit;																		// Set the integral I limit to prevent integral wind-up
}

// Set the PID controller output limit
void PID::setPIDLimit(float pidLimit)
{
	this->pidLimit = pidLimit;																// Set the PID controller output limit
}

// Set the previous error value
void PID::setPrevError(float prevError)
{
	this->prevError = prevError;															// Set the previous error value
}

// Set the previous input value
void PID::setPrevInput(float prevInput)
{
	this->prevInput = prevInput;															// Set the previous input value
}
