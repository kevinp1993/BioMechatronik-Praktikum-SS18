//~ #include <chprintf.h>
//~ #include <shell.h>
#include "userthread.hpp"
#include "global.hpp"
#include "LineFollowingPID.hpp"

extern Global global;

namespace LineFollowingPID
{
//----------------------------------------------------------
//Variablen fuer die Linienverfolgung
//----------------------------------------------------------
//Maximale Geschwindigkeit 
int MaxSpeed = 10000;
	
//Schwellwert zwischen Schwarz und Weiß
const int BW_Threshold = 1600;

//Gewichtung fuer Error-Berechnung
const float weights[4] = {-1, -0.5, 0.5, 1};

//P, I und D Parameter fuer PID Regler
float Kp = 50;
float Ki = 0.01;
float Kd = 0.001;
	
//Speicher für Sensormesswerte
int vcnl4020Proximity_Raw[4] = {0};
int vcnl4020Proximity_Mapped[4] = {0};

//Speicher für PID
float lastError = 0;
float sumError = 0;


//----------------------------------------------------------
//Funktionen fuer die Linienverfolgung
//----------------------------------------------------------

//Zuordnung, ob Sensor schwarz oder weiß erkennt 
LineColor ColorMapping(int value) 
{
	if (value < BW_Threshold)
		return BLACK;
	else
		return WHITE;
}

//Rohwerte der Abstandssensoren auslesen und in Schwarz/Weiß einordnen
void readProximitySensors()
{
	//Alle 4 Abstandssensoren auslesen und einer Farbe zuordnen
	for (int i = 0; i < 4; i++) 
	{
		vcnl4020Proximity_Raw[i] = global.vcnl4020[i].getProximityScaledWoOffset();
		vcnl4020Proximity_Mapped[i] = ColorMapping(vcnl4020Proximity_Raw[i]); 
	}
}

//Abweichung zwischen Soll und Ist berechnen
//Soll: Die Beiden mittleren (vorne) Abstandssensoren zeigen schwarz an --> Error = 0
float CalculateError()
{
	//Abweichung k berechnen und Anzahl der "1" (weiße Werte) bestimmen
	float k = 0;
	unsigned int num_ones = 0;
	for (int i = 0; i < 4; i++)
	{
		k = k + (weights[i] * vcnl4020Proximity_Mapped[i]);
		
		if (vcnl4020Proximity_Mapped[i] == 1)
		{
			num_ones++;
		}
	}
	//Error berechnen
	float Error = k/num_ones;
	return Error;
}

//Stellgroesse ermitteln
float PIDControler(float Error)
{
	sumError = sumError + Error;
	//Stellgroesse ermitteln
	float u = Kp * Error + Ki * sumError + Kd * (Error - lastError);
	lastError = Error;
	
	return u;
}

//Stellglied (Motoren) ansteuern mit veränderbare Stellgroesse 
//u=0 --> Geradeaus
//u>0 --> Nach Links abbiegen
//u<0 --> Nach Rechts abbiegen
void MotorControlLineFollowing(float u)
{
	int rpmSpeed[2] = {1,1};
	if (u<0)
	{
		//Nach rechts abbiegen (Linkes Rad --> Max ------ Rechtes Rad --> Max - u)
		global.motorcontrol.setTargetRPM(rpmSpeed[constants::DiWheelDrive::LEFT_WHEEL] * MaxSpeed, 
		rpmSpeed[constants::DiWheelDrive::RIGHT_WHEEL] * MaxSpeed + u);
	}
	else if (u>0)
	{
		//Nach links abbiegen (Rechtes Rad --> Max ------ Linkes Rad --> Max -u)
		global.motorcontrol.setTargetRPM(rpmSpeed[constants::DiWheelDrive::LEFT_WHEEL] * MaxSpeed -u, 
		rpmSpeed[constants::DiWheelDrive::RIGHT_WHEEL] * MaxSpeed);
	}
	else
	{
		//Geradeaus (Linkes Rad --> Max ------ Rechtes Rad --> Max)
		global.motorcontrol.setTargetRPM(rpmSpeed[constants::DiWheelDrive::LEFT_WHEEL] * MaxSpeed, 
		rpmSpeed[constants::DiWheelDrive::RIGHT_WHEEL] * MaxSpeed);
	}
}

void MainLineFollowing()
{
	//Rohwerte der Abstandssensoren auslesen und in Schwarz/Weiß einordnen
	readProximitySensors();
	//Abweichung zwischen Soll und Ist berechnen
	float Error = CalculateError();
	//Stellgroesse mit PID Regler bestimmen aus Abweichung
	float correction = PIDControler(Error);
	//Motoren mit geregelter Stellgroesse ansteuern
	MotorControlLineFollowing(correction);
}


//----------------------------------------------------------
//Funktionen zur Aenderung der Systemparameter
//----------------------------------------------------------

void changePID(float Kp_User, float Ki_User, float Kd_User)
{	
	Kp = Kp_User;
	Ki = Ki_User;
	Kd = Kd_User;
}

void showPID()
{
	chprintf((BaseSequentialStream*)&global.sercanmux1, "P = %f \t I = %f \t D = %f \n",Kp,Ki,Kd);
}

void changeSpeed(int MaxSpeed_User)
{
	MaxSpeed = MaxSpeed_User;
}


}//Nampespace schliessen



