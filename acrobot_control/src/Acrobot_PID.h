#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#ifndef GRAVITY
#define GRAVITY 9.81
#endif

#ifndef PI
#define PI 3.141592
#endif

#define SamplingTime 0.001
#define Deg2Rad PI/180
#define Rad2Deg 180/PI


class Acrobot
{
private: // variables
	
	float Time;
	float DesQ[2];
	float DesQdot[2];
	float ActQ[2];
	float ActQdot[2];
	float ControlTorque[2];
	float Error[2];
	float ErrorDot[2];
	float SumError[2];
	float Freq;
	float Mag;
	float KP[2];
	float KD[2];
	float KI[2];

public: // functions

	void InitParameter()
	{

		for(int i=0; i<2; i++)
		{
			DesQ[i] = 0;
			DesQdot[i] = 0;
			ActQ[i];
			ActQdot[i];
			ControlTorque[i] = 0;
			Error[i] = 0;
			ErrorDot[i] = 0;
			SumError[i] = 0;
		}

		Time = 0;
		Freq = 0.5;
		Mag = 45;

		KP[0] = 0.7;
		KD[0] = 0.06;
		KI[0] = 0.003;

		KP[1] = 0.28;
		KD[1] = 0.045;
		KI[1] = 0.0025;		
	}
	
	void Trajectory(float *fActQ, float *fActQdot)
	{
		Time += SamplingTime;
		
		for(int i=0; i<2; i++)
		{
			ActQ[i] = fActQ[i];
			ActQdot[i] = fActQdot[i];
		}

		DesQ[0] = Mag*sin(2*PI*Freq*Time);
		DesQdot[0] = 2*PI*Freq*Mag*cos(2*PI*Freq*Time);

		DesQ[1] = Mag*sin(2*PI*Freq*Time);
		DesQdot[1] = 2*PI*Freq*Mag*cos(2*PI*Freq*Time);

		for(int i=0; i<2; i++)
		{
			Error[i] = DesQ[i] - ActQ[i];
			ErrorDot[i] = DesQdot[i] - ActQdot[i];
			SumError[i] += Error[i];
		}

		if(Time > 10*1/Freq)
			Time = 0;

	}

	float calcControlTorque1()
	{

		ControlTorque[0] = KP[0]*Error[0] + KD[0]*ErrorDot[0] + KI[0]*SumError[0];
		
		return ControlTorque[0];
	}
	
	float calcControlTorque2()
	{

		ControlTorque[1] = KP[1]*Error[1] + KD[1]*ErrorDot[1] + KI[1]*SumError[1];
		
		return ControlTorque[1];
	}

	float getPosCmd1()
	{
		return DesQ[0];
	}

	float getPosCmd2()
	{
		return DesQ[1];
	}

	float getPosAct1()
	{
		return ActQ[0];
	}

	float getPosAct2()
	{
		return ActQ[1];
	}

	float getVelCmd1()
	{
		return DesQdot[0];
	}

	float getVelCmd2()
	{
		return DesQdot[1];
	}

	float getVelAct1()
	{
		return ActQdot[0];
	}

	float getVelAct2()
	{
		return ActQdot[1];
	}

};
