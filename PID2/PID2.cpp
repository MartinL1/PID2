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

// P Controller constructor - with defined minimum and maximum PID output limits 
PID::PID(float kp, float pidMinLimit, float pidMaxLimit) : 
	kp(kp), pidMinLimit(pidMinLimit), pidMaxLimit(pidMaxLimit) {}
	
// PID Controller constructor - with defined minimum and maximum integral and PID output limits
PID::PID(float kp, float ki, float kd, float iMinLimit, float iMaxLimit, 
	float pidMinLimit, float pidMaxLimit) : 
	kp(kp), ki(ki), kd(kd), iMinLimit(iMinLimit), iMaxLimit(iMaxLimit), 
	pidMinLimit(pidMinLimit), pidMaxLimit(pidMaxLimit) {}	

// Initialise the P controller - with defined minimum and maximum PID output limits 
void PID::begin(float kp, float pidMinLimit, float pidMaxLimit)
{
	this->kp = kp;																								// Set the proportional P gain to the new value
	this->pidMinLimit = pidMinLimit;															// Set the proportional P minimum output limit
	this->pidMaxLimit = pidMaxLimit;															// Set the proportional P maximum output limit
}
	
// Initialise the PID controller - with defined minimum and maximum integral and PID output limits
void PID::begin(float kp, float ki, float kd, float iMinLimit, float iMaxLimit, 
	float pidMinLimit, float pidMaxLimit)
{
	this->kp = kp;																								// Set the proportional P gain to the new value
	this->ki = ki;																								// Set the integral I gain to the new value
	this->kd = kd;																								// Set the derivative D gain to the new value
	this->iMinLimit = iMinLimit;																	// Set the integral I minimum output limit
	this->iMaxLimit = iMaxLimit;																	// Set the integral I maximum output limit
	this->pidMinLimit = pidMinLimit;															// Set the combined PID minimum output limit
	this->pidMaxLimit = pidMaxLimit;															// Set the combined PID maximum output limit
}	
	
// Calculate the classic P control loop
float PID::pCalculate(float setpoint, float input)
{
  float error = setpoint - input;																// Calculate the error = setpoint - input
	float pTerm = kp * error;                                 		// pTerm = (setpoint - input) * proportional gain 
  pTerm = pTerm > pidMaxLimit ? pidMaxLimit : pTerm;    				// Check that the maximum output limit is not exceeded
  pTerm = pTerm < pidMinLimit ? pidMinLimit : pTerm;  					// Check that the minimum output limit is not exceeded
  return pTerm;                                             		// Return the PID term
}

// Calculate the classic PID control loop, (dt is the sample time in seconds)
float PID::pidCalculate(float setpoint, float input, float dt)
{
  float error = setpoint - input;																// Calculate the error = setpoint - input
	float pTerm = kp * error;                                 		// pTerm = (setpoint - input) * proportional gain 
  iTerm += ki * error * dt;                                 		// iTerm += (setpoint - input) * integral gain * loop time (dt)
  iTerm = iTerm > iMaxLimit ? iMaxLimit : iTerm;            		// Check that the maximum integral limit is not exceeded
  iTerm = iTerm < iMinLimit ? iMinLimit : iTerm;            		// Check that the minimum integral limit is not exceeded
  float dTerm = kd * (error - prevError) / dt;              		// dTerm = derivative gain * (current error - previous error) / loop time (dt)
  float pidTerm = pTerm + iTerm + dTerm;                    		// Sum the PID terms
  pidTerm = pidTerm > pidMaxLimit ? pidMaxLimit : pidTerm;  		// Check that the maximum output limit is not exceeded
  pidTerm = pidTerm < pidMinLimit ? pidMinLimit : pidTerm;  		// Check that the minimum output limit is not exceeded
	prevError = error;																						// Update the previous error value
  return pidTerm;                                           		// Return the PID term
}

// Calculate the "Derivative on Measurement" PID control loop with the input error (rather than error - prevError) as the derivative
// This removes the "derivative kick" induced by a sudden change in the setpoint, (dt is the sample time in seconds)
float PID::pidCalculateDOM(float setpoint, float input, float dt)
{
  float error = setpoint - input;																// Calculate the error = setpoint - input
	float pTerm = kp * error;                                 		// pTerm = (setpoint - input) * proportional gain 
  iTerm += ki * error * dt;                                 		// iTerm += (setpoint - input) * integral gain * loop time (dt)
  iTerm = iTerm > iMaxLimit ? iMaxLimit : iTerm;            		// Check that the maximum integral limit is not exceeded
  iTerm = iTerm < -iMinLimit ? -iMinLimit : iTerm;          		// Check that the minimum integral limit is not exceeded
  float inputError = input - prevInput;													// Calculate the input error = input - previous input
	float dTerm = kd * -inputError / dt;                      		// dTerm = derivative gain * (-inputError) / loop time (dt)
  float pidTerm = pTerm + iTerm + dTerm;                    		// Sum the PID terms
  pidTerm = pidTerm > pidMaxLimit ? pidMaxLimit : pidTerm;  		// Check that the maximum output limit is not exceeded
  pidTerm = pidTerm < pidMinLimit ? pidMinLimit : pidTerm;  		// Check that the minimum output limit is not exceeded
  prevInput = input;																						// Update the previous input
	return pidTerm;                                           		// Return the PID term
}

void PID::setKp(float kp) { this->kp = kp; }										// Set the proportional P gain
void PID::setKi(float ki) { this->ki = ki; } 										// Set the integral I gain
void PID::setKd(float kd) { this->kd = kd; }   									// Set the derivative D gain

void PID::setILimits(float iMinLimit, float iMaxLimit)
{
	this->iMinLimit = iMinLimit;																	// Set the integral I minimum and maximum limits to prevent integral wind-up
	this->iMaxLimit = iMaxLimit;
}

void PID::setPIDLimits(float pidMinLimit, float pidMaxLimit)
{
	this->pidMinLimit = pidMinLimit;															// Set the PID controller minimum and maximum output limits
	this->pidMaxLimit = pidMaxLimit;
}

void PID::setIMinLimit(float iMinLimit)
{
	this->iMinLimit = iMinLimit;																	// Set the integral I minimum limit to prevent integral wind-up
}

void PID::setIMaxLimit(float iMaxLimit)
{
	this->iMaxLimit = iMaxLimit;																	// Set the integral I maximum limit to prevent integral wind-up
}

void PID::setPIDMinLimit(float pidMinLimit)
{
	this->pidMinLimit = pidMinLimit;															// Set the PID controller minimum output limit
}

void PID::setPIDMaxLimit(float pidMaxLimit)
{
	this->pidMaxLimit = pidMaxLimit;															// Set the PID controller maximum output limit
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
float PID::getILimit() { return iMaxLimit; }										// Get the integral limits
float PID::getPIDLimit() { return pidMaxLimit; }								// Get the PID output limits
float PID::getIMinLimit() { return iMinLimit; }									// Get the integral minimum limit
float PID::getIMaxLimit() { return iMaxLimit; }									// Get the integral maximum limit
float PID::getPIDMinLimit() { return pidMinLimit; }							// Get the PID output minimum limit
float PID::getPIDMaxLimit() { return pidMaxLimit; }							// Get the PID output maximum limit
float PID::getPrevError() { return prevError; }									// Get the previous error value
float PID::getPrevInput() { return prevInput; }									// Get the previous input value
