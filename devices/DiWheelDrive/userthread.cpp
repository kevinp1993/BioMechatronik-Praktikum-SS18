#include "userthread.hpp"
#include "LineFollowingPID.hpp"
#include "global.hpp"

using namespace amiro;
//using namespace LineFollowingPID;

extern Global global;

//// State machine states
//enum states : uint8_t {
	//IDLE,
	//GO_RIGHT,
	//GO_STRAIGHT,
	//PARKING,
	//PARKING_RIGHT,
	//PARKING_LEFT,
	//GO_LEFT,
	//SPINNING_PARKING,
	//SPINNING
//};

//// Policy
//states policy[] = {
  //GO_STRAIGHT,
  //GO_RIGHT,
  //GO_RIGHT,
  //GO_STRAIGHT,
  //GO_RIGHT,
  //GO_STRAIGHT,
  //GO_RIGHT,
  //GO_STRAIGHT,
  //GO_STRAIGHT,
  //GO_RIGHT,
  //GO_STRAIGHT,
  //GO_RIGHT,
  //GO_STRAIGHT
//};

// The different classes (or members) of color discrimination
// BLACK is the line itselfe
// GREY is the boarder between the line and the surface
// WHITE is the common surface
//enum colorMember : uint8_t {
	//BLACK=0,
	//GREY=1,
	//WHITE=2
//};

// a buffer for the z-value of the accelerometer
//int16_t accel_z;
bool running;

// Get some information about the policy
//const int sizeOfPolicy = sizeof(policy) / sizeof(states);
//int policyCounter = 0; // Do not change this, it points to the beginning of the policy

//// Different speed settings (all values in "rounds per minute")
//const int rpmForward[2] = {25,25};
//const int rpmSoftLeft[2] = {15,25};
//const int rpmHardLeft[2] = {10,25};
//const int rpmSoftRight[2] = {rpmSoftLeft[1],rpmSoftLeft[0]};
//const int rpmHardRight[2] = {rpmHardLeft[1],rpmHardLeft[0]};
//const int rpmTurnLeft[2] = {-10, 10};
//const int rpmTurnRight[2] = {rpmTurnLeft[1],rpmTurnLeft[0]};
//const int rpmHalt[2] = {0, 0};

//// Definition of the fuzzyfication function
////  | Membership
//// 1|_B__   G    __W__
////  |    \  /\  /
////  |     \/  \/
////  |_____/\__/\______ Sensor values
//// SEE MATLAB SCRIPT "fuzzyRule.m" for adjusting the values
//// All values are "raw sensor values"
//* Use these values for white ground surface (e.g. paper) */

//const int blackStartFalling = 0x1000; // Where the black curve starts falling
//const int blackOff = 0x1800; // Where no more black is detected
//const int whiteStartRising = 0x2800; // Where the white curve starts rising
//const int whiteOn = 0x6000; // Where the white curve has reached the maximum value
//const int greyMax = (whiteOn + blackStartFalling) / 2; // Where grey has its maximum
//const int greyStartRising = blackStartFalling; // Where grey starts rising
//const int greyOff = whiteOn; // Where grey is completely off again

//* Use these values for gray ground surfaces */
//*
//const int blackStartFalling = 0x1000; // Where the black curve starts falling
//const int blackOff = 0x2800; // Where no more black is detected
//const int whiteStartRising = 0x4000; // Where the white curve starts rising
//const int whiteOn = 0x5000; // Where the white curve starts rising
//const int greyMax = (whiteOn + blackStartFalling) / 2; // Where grey has its maximum
//const int greyStartRising = blackStartFalling; // Where grey starts rising
//const int greyOff = whiteOn; // Where grey is completely off again
//*/

//int vcnl4020AmbientLight[4] = {0};
//int vcnl4020Proximity[4] = {0};

//// Border for the discrimination between black and white
//const int discrBlackWhite = 16000; // border in "raw sensor values"
//// Discrimination between black and white (returns BLACK or WHITE)
//// The border was calculated by a MAP-decider
//colorMember discrimination(int value) {
	//if (value < discrBlackWhite)
		//return BLACK;
	//else
		//return WHITE;
//}

//// Copy the speed from the source to the target array
//void copyRpmSpeed(const int (&source)[2], int (&target)[2]) {
	//target[constants::DiWheelDrive::LEFT_WHEEL] = source[constants::DiWheelDrive::LEFT_WHEEL];
	//target[constants::DiWheelDrive::RIGHT_WHEEL] = source[constants::DiWheelDrive::RIGHT_WHEEL];
//}

//// Fuzzyfication of the sensor values
//void fuzzyfication(int sensorValue, float (&fuzziedValue)[3]) {
	//if (sensorValue < blackStartFalling ) {
		//// Only black value
		//fuzziedValue[BLACK] = 1.0f;
		//fuzziedValue[GREY] = 0.0f;
		//fuzziedValue[WHITE] = 0.0f;
	//} else if (sensorValue > whiteOn ) {
		//// Only white value
		//fuzziedValue[BLACK] = 0.0f;
		//fuzziedValue[GREY] = 0.0f;
		//fuzziedValue[WHITE] = 1.0f;
	//} else if ( sensorValue < greyMax) {
		//// Some greyisch value between black and grey

		//// Black is going down
		//if ( sensorValue > blackOff) {
			//fuzziedValue[BLACK] = 0.0f;
		//} else {
			//fuzziedValue[BLACK] = static_cast<float>(sensorValue-blackOff) / (blackStartFalling-blackOff);
		//}

		//// Grey is going up
		//if ( sensorValue < greyStartRising) {
			//fuzziedValue[GREY] = 0.0f;
		//} else {
			//fuzziedValue[GREY] = static_cast<float>(sensorValue-greyStartRising) / (greyMax-greyStartRising);
		//}

		//// White is absent
		//fuzziedValue[WHITE] = 0.0f;

	//} else if ( sensorValue >= greyMax) {
		//// Some greyisch value between grey white

		//// Black is absent
		//fuzziedValue[BLACK] = 0.0f;

		//// Grey is going down
		//if ( sensorValue < greyOff) {
			//fuzziedValue[GREY] = static_cast<float>(sensorValue-greyOff) / (greyMax-greyOff);
		//} else {
			//fuzziedValue[GREY] = 0.0f;
		//}

		//// White is going up
		//if ( sensorValue < whiteStartRising) {
			//fuzziedValue[WHITE] = 0.0f;
		//} else {
			//fuzziedValue[WHITE] = static_cast<float>(sensorValue-whiteStartRising) / (whiteOn-whiteStartRising);
		//}
	//}
//}

//// Return the color, which has the highest fuzzy value
//colorMember getMember(float (&fuzzyValue)[3]) {
	//colorMember member;

	//if (fuzzyValue[BLACK] > fuzzyValue[GREY])
		//if (fuzzyVreadProximitySensors()alue[BLACK] > fuzzyValue[WHITE])
			//member = BLACK;
		//else
			//member = WHITE;
	//else
		//if (fuzzyValue[GREY] > fuzzyValue[WHITE])
			//member = GREY;
		//else
			//member = WHITE;

	//return member;
//}

//// Get a crisp output for the steering commands
//void defuzzyfication(colorMember (&member)[4], int (&rpmFuzzyCtrl)[2]) {

	//// all sensors are equal
	//if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == member[constants::DiWheelDrive::PROX_FRONT_LEFT] &&
	    //member[constants::DiWheelDrive::PROX_FRONT_LEFT] == member[constants::DiWheelDrive::PROX_FRONT_RIGHT] &&
	    //member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == member[constants::DiWheelDrive::PROX_WHEEL_RIGHT]) {
		//// something is wrong -> stop
		//copyRpmSpeed(rpmHalt, rpmFuzzyCtrl);
	//// both front sensor detect a line
	//} else if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == BLACK &&
	    //member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == BLACK) {
		//// straight
		//copyRpmSpeed(rpmForward, rpmFuzzyCtrl);
	//// exact one front sensor detects a line
	//} else if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == BLACK ||
	           //member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == BLACK) {
		//// soft correction
		//if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == GREY) {
			//// soft right
			//copyRpmSpeed(rpmSoftRight, rpmFuzzyCtrl);
		//} else if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == WHITE) {
			//// hard right
			//copyRpmSpeed(rpmHardRight, rpmFuzzyCtrl);
		//} else if (member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == GREY) {
			//// soft left
			//copyRpmSpeed(rpmSoftLeft, rpmFuzzyCtrl);
		//} else if (member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == WHITE) {
			//// hard left
			//copyRpmSpeed(rpmHardLeft, rpmFuzzyCtrl);
		//}
	//// both wheel sensors detect a line
	//} else if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == BLACK &&
	           //member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == BLACK) {
		//// something is wrong -> stop
		//copyRpmSpeed(rpmHalt, rpmFuzzyCtrl);
	//// exactly one wheel sensor detects a line
	//} else if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == BLACK ||
	           //member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == BLACK) {
		//if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == BLACK) {
			//// turn left
			//copyRpmSpeed(rpmTurnLeft, rpmFuzzyCtrl);
		//} else if (member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == BLACK) {
			//// turn right
			//copyRpmSpeed(rpmTurnRight, rpmFuzzyCtrl);
		//}
	//// both front sensors may detect a line
	//} else if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == GREY &&
	           //member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == GREY) {
		//if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == GREY) {
			//// turn left
			//copyRpmSpeed(rpmTurnLeft, rpmFuzzyCtrl);
		//} else if (member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == GREY) {
			//// turn right
			//copyRpmSpeed(rpmTurnRight, rpmFuzzyCtrl);
		//}
	//// exactly one front sensor may detect a line
	//} else if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == GREY ||
	           //member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == GREY) {
		//if (member[constants::DiWheelDrive::PROX_FRONT_LEFT] == GREY) {
			//// turn left
			//copyRpmSpeed(rpmTurnLeft, rpmFuzzyCtrl);
		//} else if (member[constants::DiWheelDrive::PROX_FRONT_RIGHT] == GREY) {
			//// turn right
			//copyRpmSpeed(rpmTurnRight, rpmFuzzyCtrl);
		//}
	//// both wheel sensors may detect a line
	//} else if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == GREY &&
	           //member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == GREY) {
		//// something is wrong -> stop
		//copyRpmSpeed(rpmHalt, rpmFuzzyCtrl);
	//// exactly one wheel sensor may detect a line
	//} else if (member[constants::DiWheelDrive::PROX_WHEEL_LEFT] == GREY ||
	           //member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == GREY) {
		//if (member[readProximitySensors()constants::DiWheelDrive::PROX_WHEEL_LEFT] == GREY) {
			//// turn left
			//copyRpmSpeed(rpmTurnLeft, rpmFuzzyCtrl);
		//} else if (member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] == GREY) {
			//// turn right
			//copyRpmSpeed(rpmTurnRight, rpmFuzzyCtrl);
		//}
	//// no sensor detects anything 
	//} else {
		//// line is lost -> stop
		//copyRpmSpeed(rpmHalt, rpmFuzzyCtrl);
	//}

	//return;
//}

//Color memberToLed(colorMember member) {
	//switch (member) {
		//case BLACK:
			//return Color(Color::GREEN);
		//case GREY:
			//return Color(Color::YELLOW);
		//case WHITE:
			//return Color(Color::RED);
		//default:
			//return Color(Color::WHITE);
	//}
//}

//// Line following by a fuzzy controler
//void lineFollowing(int (&proximity)[4], int (&rpmFuzzyCtrl)[2]) {
	//// FUZZYFICATION
	//// First we need to get the fuzzy value for our 3 values {BLACK, GREY, WHITE}
	//float leftWheelFuzzyMemberValues[3], leftFrontFuzzyMemberValues[3], rightFrontFuzzyMemberValues[3], rightWheelFuzzyMemberValues[3];
	//fuzzyfication(proximity[constants::DiWheelDrive::PROX_WHEEL_LEFT], leftWheelFuzzyMemberValues);
	//fuzzyfication(proximity[constants::DiWheelDrive::PROX_FRONT_LEFT], leftFrontFuzzyMemberValues);
	//fuzzyfication(proximity[constants::DiWheelDrive::PROX_FRONT_RIGHT], rightFrontFuzzyMemberValues);
	//fuzzyfication(proximity[constants::DiWheelDrive::PROX_WHEEL_RIGHT], rightWheelFuzzyMemberValues);

	//// INFERENCE RULE DEFINITION
	//// Get the member for each sensor
	//colorMember member[4];
	//member[constants::DiWheelDrive::PROX_WHEEL_LEFT] = getMember(leftWheelFuzzyMemberValues);
	//member[constants::DiWheelDrive::PROX_FRONT_LEFT] = getMember(leftFrontFuzzyMemberValues);
	//member[constants::DiWheelDrive::PROX_FRONT_RIGHT] = getMember(rightFrontFuzzyMemberValues);
	//member[constants::DiWheelDrive::PROX_WHEEL_RIGHT] = getMember(rightWheelFuzzyMemberValues);

	//// visualize sensors via LEDs
	//global.robot.setLightColor(constants::LightRing::LED_WNW, memberToLed(member[constants::DiWheelDrive::PROX_WHEEL_LEFT]));
	//global.robot.setLightColor(constants::LightRing::LED_NNW, memberToLed(member[constants::DiWheelDrive::PROX_FRONT_LEFT]));
	//global.robot.setLightColor(constants::LightRing::LED_NNE, memberToLed(member[constants::DiWheelDrive::PROX_FRONT_RIGHT]));
	//global.robot.setLightColor(constants::LightRing::LED_ENE, memberToLed(member[constants::DiWheelDrive::PROX_WHEEL_RIGHT]));

////	chprintf((BaseSequentialStream*) &SD1, "Left: BLACK: %f, GREY: %f, WHITE: %f\r\n", leftFuzzyMemberValues[BLACK], leftFuzzyMemberValues[GREY], leftFuzzyMemberValues[WHITE]);
////	chprintf((BaseSequentialStream*) &SD1, "Right: BLACK: %f, GREY: %f, WHITE: %f\r\n", rightFuzzyMemberValues[BLACK], rightFuzzyMemberValues[GREY], rightFuzzyMemberValues[WHITE]);

	//// DEFUZZYFICATION
	//defuzzyfication(member, rpmFuzzyCtrl);
//}

//// Set the speed by the array
//void setRpmSpeed(const int (&rpmSpeed)[2]) {
	//global.motorcontrol.setTargetRPM(rpmSpeed[constants::DiWheelDrive::LEFT_WHEEL] * 1000000, rpmSpeed[constants::DiWheelDrive::RIGHT_WHEEL] * 1000000);
//}

//// Get the next policy rule
//states getNextPolicy() {
	//// If the policy is over, start again
	//if (policyCounter >= sizeOfPolicy)
		//policyCounter = 3;

	//return policy[policyCounter++];
//}



UserThread::UserThread() :
  chibios_rt::BaseStaticThread<USER_THREAD_STACK_SIZE>()
{
}

UserThread::~UserThread()
{
}

msg_t
UserThread::main()
{
	/*
	 * SETUP
	 */
	//int rpmFuzzyCtrl[2] = {0};
    for (uint8_t led = 0; led < 8; ++led) {
		global.robot.setLightColor(led, Color(Color::BLACK));
    }
    running = false;

	/*
	 * LOOP
	 */
	while (!this->shouldTerminate())
	{
		
		LineFollowingPID::readProximitySensors();
        //*
         //* read accelerometer z-value
         //*/
        //accel_z = global.lis331dlh.getAccelerationForce(LIS331DLH::AXIS_Z);

        //*
         //* evaluate the accelerometer
         //*/
        //if (accel_z < -900 /*-0.9g*/) {
            //if (running) {
                //// stop the robot
                //running = false;
                //global.motorcontrol.setTargetRPM(0, 0);
            //} else {
                //// start the robot
                //running = true;
            //}
            //// set the front LEDs to blue for one second
            //global.robot.setLightColor(constants::LightRing::LED_SSW, Color(Color::BLACK));
            //global.robot.setLightColor(constants::LightRing::LED_WSW, Color(Color::BLACK));
            //global.robot.setLightColor(constants::LightRing::LED_WNW, Color(Color::WHITE));
            //global.robot.setLightColor(constants::LightRing::LED_NNW, Color(Color::WHITE));
            //global.robot.setLightColor(constants::LightRing::LED_NNE, Color(Color::WHITE));
            //global.robot.setLightColor(constants::LightRing::LED_ENE, Color(Color::WHITE));
            //global.robot.setLightColor(constants::LightRing::LED_ESE, Color(Color::BLACK));
            //global.robot.setLightColor(constants::LightRing::LED_SSE, Color(Color::BLACK));
            //this->sleep(MS2ST(1000));
            //global.robot.setLightColor(constants::LightRing::LED_WNW, Color(Color::BLACK));
            //global.robot.setLightColor(constants::LightRing::LED_NNW, Color(Color::BLACK));
            //global.robot.setLightColor(constants::LightRing::LED_NNE, Color(Color::BLACK));
            //global.robot.setLightColor(constants::LightRing::LED_ENE, Color(Color::BLACK));
        //}

        //if (running) {
            //// Read the proximity values
            //for (int i = 0; i < 4; i++) {
                //vcnl4020AmbientLight[i] = global.vcnl4020[i].getAmbientLight();
                //vcnl4020Proximity[i] = global.vcnl4020[i].getProximityScaledWoOffset();
            //}

////            chprintf((BaseSequentialStream*) &SD1, "0x%04X 0x%04X 0x%04X 0x%04X\n",
////                     vcnl4020Proximity[constants::DiWheelDrive::PROX_WHEEL_LEFT],
////                     vcnl4020Proximity[constants::DiWheelDrive::PROX_FRONT_LEFT],
////                     vcnl4020Proximity[constants::DiWheelDrive::PROX_FRONT_RIGHT],
////                     vcnl4020Proximity[constants::DiWheelDrive::PROX_WHEEL_RIGHT]);

            //lineFollowing(vcnl4020Proximity, rpmFuzzyCtrl);
            //setRpmSpeed(rpmFuzzyCtrl);
        //}

		this->sleep(CAN::UPDATE_PERIOD);
	}

  return RDY_OK;
}

