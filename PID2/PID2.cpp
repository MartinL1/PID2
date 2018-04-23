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

// P Controller constructor - with symmetrical PID output limits
PID::PID(float kp, float pidLimit) : 
	kp(kp), pidLowerLimit(-pidLimit), pidUpperLimit(pidLimit) {}

// P Controller constructor - with defined lower and upper PID output limits 
PID::PID(float kp, float pidLowerLimit, float pidUpperLimit) : 
	kp(kp), pidLowerLimit(pidLowerLimit), pidUpperLimit(pidUpperLimit) {}

// PID Controller constructor - with symmetrical integral and PID output limits
PID::PID(float kp, float ki, float kd, float iLimit, float pidLimit) : 
	kp(kp), ki(ki), kd(kd), iLowerLimit(-iLimit), iUpperLimit(iLimit), 
	pidLowerLimit(-pidLimit), pidUpperLimit(pidLimit) {}
	
// PID Controller constructor - with defined lower and upper integral and PID output limits
PID::PID(float kp, float ki, float kd, float iLowerLimit, float iUpperLimit, 
	float pidLowerLimit, float pidUpperLimit) : 
	kp(kp), ki(ki), kd(kd), iLowerLimit(iLowerLimit), iUpperLimit(iUpperLimit), 
	pidLowerLimit(pidLowerLimit), pidUpperLimit(pidUpperLimit) {}	

// Initialise the P controller - with symmetrical PID output limits
void PID::begin(float kp, float pidLimit)
{
	this->kp = kp;																								// Set the proportional P gain to the new value
	this->pidLowerLimit = -pidLimit;															// Set the proportional P lower output limit
	this->pidLowerLimit = pidLimit;																// Set the proportional P upper output limit
}

// Initialise the P controller - with defined lower and upper PID output limits 
void PID::begin(float kp, float pidLowerLimit, float pidUpperLimit)
{
	this->kp = kp;																								// Set the proportional P gain to the new value
	this->pidLowerLimit = pidLowerLimit;													// Set the proportional P lower output limit
	this->pidUpperLimit = pidUpperLimit;													// Set the proportional P upper output limit
}

// Initialise the PID controller - with symmetrical integral and PID output limits
void PID::begin(float kp, float ki, float kd, float iLimit, float pidLimit)
{
	this->kp = kp;																								// Set the proportional P gain to the new value
	this->ki = ki;																								// Set the integral I gain to the new value
	this->kd = kd;																								// Set the derivative D gain to the new value
	this->iLowerLimit = -iLimit;																	// Set the integral I lower output limit
	this->iUpperLimit = iLimit;																		// Set the integral I upper output limit
	this->pidLowerLimit = -pidLimit;															// Set the combined PID lower output limit
	this->pidUpperLimit = pidLimit;																// Set the combined PID upper output limit
}
	
// Initialise the PID controller - with defined lower and upper integral and PID output limits
void PID::begin(float kp, float ki, float kd, float iLowerLimit, float iUpperLimit, 
	float pidLowerLimit, float pidUpperLimit)
{
	this->kp = kp;																								// Set the proportional P gain to the new value
	this->ki = ki;																								// Set the integral I gain to the new value
	this->kd = kd;																								// Set the derivative D gain to the new value
	this->iLowerLimit = iLowerLimit;															// Set the integral I lower output limit
	this->iUpperLimit = iUpperLimit;															// Set the integral I upper output limit
	this->pidLowerLimit = pidLowerLimit;													// Set the combined PID lower output limit
	this->pidUpperLimit = pidUpperLimit;													// Set the combined PID upper output limit
}	
	
// Calculate the classic P control loop
float PID::pCalculate(float setpoint, float input)
{
  float error = setpoint - input;																// Calculate the error = setpoint - input
	float pTerm = kp * error;                                 		// pTerm = (setpoint - input) * proportional gain 
  pTerm = pTerm > pidUpperLimit ? pidUpperLimit : pTerm;    		// Check that the upper output limit is not exceeded
  pTerm = pTerm < pidLowerLimit ? pidLowerLimit : pTerm;  			// Check that the lower output limit is not exceeded
  return pTerm;                                             		// Return the PID term
}

// Calculate the classic PID control loop - dt is the sample time in seconds
float PID::pidCalculate(float setpoint, float input, float dt)
{
  float error = setpoint - input;																// Calculate the error = setpoint - input
	float pTerm = kp * error;                                 		// pTerm = (setpoint - input) * proportional gain 
  iTerm += ki * error * dt;                                 		// iTerm += (setpoint - input) * integral gain * loop time (dt)
  iTerm = iTerm > iUpperLimit ? iUpperLimit : iTerm;            // Check that the upper integral limit is not exceeded
  iTerm = iTerm < iLowerLimit ? iLowerLimit : iTerm;            // Check that the lower integral limit is not exceeded
  float dTerm = kd * (error - prevError) / dt;              		// dTerm = derivative gain * (current error - previous error) / loop time (dt)
  float pidTerm = pTerm + iTerm + dTerm;                    		// Sum the PID terms
  pidTerm = pidTerm > pidUpperLimit ? pidUpperLimit : pidTerm;  // Check that the upper output limit is not exceeded
  pidTerm = pidTerm < pidLowerLimit ? pidLowerLimit : pidTerm;  // Check that the lower output limit is not exceeded
	prevError = error;																						// Update the previous error value
  return pidTerm;                                           		// Return the PID term
}

// Calculate the "Derivative on Measurement" PID control loop with the input error (rather than error - prevError) as the derivative
// This removes the "derivative kick" induced by a sudden change in the setpoint - dt is the sample time in seconds
float PID::pidCalculateDOM(float setpoint, float input, float dt)
{
  float error = setpoint - input;																// Calculate the error = setpoint - input
	float pTerm = kp * error;                                 		// pTerm = (setpoint - input) * proportional gain 
  iTerm += ki * error * dt;                                 		// iTerm += (setpoint - input) * integral gain * loop time (dt)
  iTerm = iTerm > iUpperLimit ? iUpperLimit : iTerm;            // Check that the upper integral limit is not exceeded
  iTerm = iTerm < -iLowerLimit ? -iLowerLimit : iTerm;          // Check that the lower integral limit is not exceeded
  float inputError = input - prevInput;													// Calculate the input error = input - previous input
	float dTerm = kd * -inputError / dt;                      		// dTerm = derivative gain * (-inputError) / loop time (dt)
  float pidTerm = pTerm + iTerm + dTerm;                    		// Sum the PID terms
  pidTerm = pidTerm > pidUpperLimit ? pidUpperLimit : pidTerm;  // Check that the upper output limit is not exceeded
  pidTerm = pidTerm < pidLowerLimit ? pidLowerLimit : pidTerm;  // Check that the lower output limit is not exceeded
  prevInput = input;																						// Update the previous input
	return pidTerm;                                           		// Return the PID term
}

void PID::setKp(float kp) { this->kp = kp; }										// Set the proportional P gain
void PID::setKi(float ki) { this->ki = ki; } 										// Set the integral I gain
void PID::setKd(float kd) { this->kd = kd; }   									// Set the derivative D gain

void PID::setILimit(float iLimit)
{
	this->iLowerLimit = -iLimit;																	// Set the integral I lower and upper limits to prevent integral wind-up
	this->iUpperLimit = iLimit;
}

void PID::setPIDLimit(float pidLimit)
{
	this->pidLowerLimit = -pidLimit;															// Set the PID controller lower and upper output limit
	this->pidUpperLimit = pidLimit;
}

void PID::setILowerLimit(float iLowerLimit)
{
	this->iLowerLimit = iLowerLimit;															// Set the integral I lower limit to prevent integral wind-up
}

void PID::setIUpperLimit(float iUpperLimit)
{
	this->iUpperLimit = iUpperLimit;															// Set the integral I upper limit to prevent integral wind-up
}

void PID::setPIDLowerLimit(float pidLowerLimit)
{
	this->pidLowerLimit = pidLowerLimit;													// Set the PID controller lower output limit
}

void PID::setPIDUpperLimit(float pidUpperLimit)
{
	this->pidUpperLimit = pidUpperLimit;													// Set the PID controller upper output limit
}

void PID::setPrevError(float prevError)
{
	this->prevError = prevError;																	// Set the previous error value
}

void PID::setPrevInput(float prevInput)
{
	this->prevInput = prevInput;																	// Set the previous input value
}

float PID::getKp() { return kp; }																// Get the proportional P gain
float PID::getKi() { return ki; }																// Get the integral I gain
float PID::getKd()	{ return kd; }															// Get the derivative D gain
float PID::getILimit() { return iUpperLimit; }									// Get the integral limits
float PID::getPIDLimit() { return pidUpperLimit; }							// Get the PID output limits
float PID::getILowerLimit() { return iLowerLimit; }							// Get the integral lower limit
float PID::getIUpperLimit() { return iUpperLimit; }							// Get the integral upper limit
float PID::getPIDLowerLimit() { return pidLowerLimit; }					// Get the PID output lower limit
float PID::getPIDUpperLimit() { return pidUpperLimit; }					// Get the PID output upper limit
float PID::getPrevError() { return prevError; }									// Get the previous error value
float PID::getPrevInput() { return prevInput; }									// Get the previous input value
