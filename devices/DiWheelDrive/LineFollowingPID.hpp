#ifndef AMIRO_LINEFOLLOWING_PID_H_
#define AMIRO_LINEFOLLOWING_PID_H_

#include <ch.hpp>
#include <amiroosconf.h>
#include <chprintf.h>
#include <shell.h>

namespace LineFollowingPID {
	
	enum LineColor : uint8_t 
	{
		BLACK=0,
		WHITE=1
	};
	extern float Kp;
	extern float Ki;
	extern float Kd;
	
	extern int MaxSpeed;
	
	//Funktionen fuer die Linenverfolgung
	void readProximitySensors();
	LineColor ColorMapping(int value);
	float CalculateError();
	float PIDControler(float Error);
	void MotorControlLineFollowing();
	void MainLineFollowing();
	
	//Weitere Funktionen
	void changePID(float Kp_User, float Ki_User, float Kd_User);
	void showPID();
	void changeSpeed(int MaxSpeed);
}



#endif // AMIRO_LINEFOLLOWING_PID_H_
