#include "userthread.h"

#include "global.hpp"
#include <array>
#include <chprintf.h>
#include <cmath>

using namespace amiro;

extern Global global;

volatile UserThread::State current_state;
volatile UserThread::State next_state;
types::kinematic kinematic;

namespace obstacle_avoidance {

uint16_t constexpr proxThresholdLow = 0x0000;
uint16_t constexpr proxThresholdHigh = 0x1000;
uint16_t constexpr proxRange = proxThresholdHigh - proxThresholdLow;

std::array< std::array<float, 2>, 8> constexpr namMatrix = {
    /*                              x     w_z */
    std::array<float, 2>/* SSW */{ 0.00f,  0.00f},
    std::array<float, 2>/* WSW */{ 0.25f, -0.25f},
    std::array<float, 2>/* WNW */{-0.75f, -0.50f},
    std::array<float, 2>/* NNW */{-0.75f, -1.00f},
    std::array<float, 2>/* NNE */{-0.75f,  1.00f},
    std::array<float, 2>/* ENE */{-0.75f,  0.50f},
    std::array<float, 2>/* ESE */{ 0.25f,  0.25f},
    std::array<float, 2>/* SSE */{ 0.00f,  0.00f}
};
uint32_t constexpr baseTranslation = 100e3; // 2cm/s
uint32_t constexpr baseRotation = 1e6; // 1rad/s
types::kinematic constexpr defaultKinematic = {
    /*  x  [µm/s]   */ baseTranslation,
    /*  y  [µm/s]   */ 0,
    /*  z  [µm/s]   */ 0,
    /* w_x [µrad/s] */ 0,
    /* w_y [µrad/s] */ 0,
    /* w_z [µrad/s] */ 0
};

inline uint8_t ProxId2LedId(const uint8_t proxId) {
    return (proxId < 4) ? proxId+4 : proxId-4;
}

Color Prox2Color(const float prox) {
  float p = 0.0f;
  if (prox < 0.5f) {
    p = 2.0f * prox;
    return Color(0x00, p*0xFF, (1.0f-p)*0xFF);
  } else {
    p = 2.0f * (prox - 0.5f);
    return Color(p*0xFF, (1.0f-p)*0xFF, 0x00);
  }
}

} /* namespace obstacle_avoidance */

namespace wii_steering {

BluetoothWiimote wiimote(&global.wt12, RX_TX);
BluetoothSerial btserial(&global.wt12, RX_TX);

float deadzone;
char bt_address[18] = {'\0'};
float wiimoteCalib[3] = {0.0f};
uint8_t principal_axis = 1;
int8_t axis_direction = -1;

uint32_t constexpr maxTranslation = 500e3;
uint32_t constexpr maxRotation = 3.1415927f * 1000000.0f * 2.0f;

}

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
   * initialize some variables
   */
  current_state = IDLE;

  /*
   * set all LEDs black (off)
   */
  for (uint8_t led = 0; led < 8; ++led) {
    global.robot.setLightColor(led, Color(Color::BLACK));
  }

  /*
   * thread loop
   */
  while (!this->shouldTerminate()) {
    /*
     * handle changes of the state
     */
    if (next_state != current_state) {
      switch (current_state) {
        case IDLE:
        {
          if (next_state == OBSTACLE_AVOIDANCE) {
            // set all LEDs to white for one second
            for (uint8_t led = 0; led < 8; ++led) {
              global.robot.setLightColor(led, Color(Color::WHITE));
            }
            this->sleep(MS2ST(1000));
            for (uint8_t led = 0; led < 8; ++led) {
              global.robot.setLightColor(led, Color(Color::BLACK));
            }
          }
          /* if (this->next_state == WII_STEERING) */ else {
            // setup bluetooth
            wii_steering::wiimote.bluetoothWiimoteListen(wii_steering::bt_address);
            wii_steering::btserial.bluetoothSerialListen("ALL");

            // set LEDs: front = green; rear = red; sides = blue
            global.robot.setLightColor(constants::LightRing::LED_NNW, Color(Color::GREEN));
            global.robot.setLightColor(constants::LightRing::LED_NNE, Color(Color::GREEN));
            global.robot.setLightColor(constants::LightRing::LED_SSW, Color(Color::RED));
            global.robot.setLightColor(constants::LightRing::LED_SSE, Color(Color::RED));
            global.robot.setLightColor(constants::LightRing::LED_WNW, Color(Color::BLUE));
            global.robot.setLightColor(constants::LightRing::LED_WSW, Color(Color::BLUE));
            global.robot.setLightColor(constants::LightRing::LED_ENE, Color(Color::BLUE));
            global.robot.setLightColor(constants::LightRing::LED_ESE, Color(Color::BLUE));

            chprintf((BaseSequentialStream*)&global.sercanmux1, "press buttons '1' and '2' to connect\n");
          }
          break;
        }
        case OBSTACLE_AVOIDANCE:
        {
          if (next_state == IDLE) {
            // stop the robot
            kinematic = {0, 0, 0, 0, 0, 0};
            global.robot.setTargetSpeed(kinematic);

            // set all LEDs to white for one second
            for (uint8_t led = 0; led < 8; ++led) {
              global.robot.setLightColor(led, Color(Color::WHITE));
            }
            this->sleep(MS2ST(1000));
            for (uint8_t led = 0; led < 8; ++led) {
              global.robot.setLightColor(led, Color(Color::BLACK));
            }
          }
          /* if (this->next_state == WII_STEERING) */ else {
            // must turn off obstacle avoidance first
            chprintf((BaseSequentialStream*)&global.sercanmux1, "ERROR: turn off obstacle avoidance first!\n");
            next_state = OBSTACLE_AVOIDANCE;
          }
          break;
        }
        case WII_STEERING: {
          if (next_state == IDLE) {
            // stop the robot
            kinematic = {0, 0, 0, 0, 0, 0};
            global.robot.setTargetSpeed(kinematic);

            // disconnect from Wiimote controller
            wii_steering::wiimote.bluetoothWiimoteDisconnect(wii_steering::bt_address);
            wii_steering::btserial.bluetoothSerialStop();
            wii_steering::wiimote.bluetoothWiimoteStop();

            // set all LEDs to black
            for (uint8_t led = 0; led < 8; ++led) {
              global.robot.setLightColor(led, Color(Color::BLACK));
            }
          }
          /* if (this->next_state == OBSTACLE_AVOIDANCE) */ else {
            // must turn off wii steering first
            chprintf((BaseSequentialStream*)&global.sercanmux1, "ERROR: turn off wii steering first!\n");
            next_state = WII_STEERING;
          }
          break;
        }
      }
      current_state = next_state;
    }

    // sleep here so the loop is executed as quickly as possible
    this->sleep(CAN::UPDATE_PERIOD);

    /*
     * exeute behaviour depending on the current state
     */
    switch (current_state) {
      case IDLE:
      {
        // read touch sensors
        if (global.mpr121.getButtonStatus() == 0x0F) {
          next_state = OBSTACLE_AVOIDANCE;
        }
        break;
      }
      case OBSTACLE_AVOIDANCE:
      {
        // read touch sensors
        if (global.mpr121.getButtonStatus() == 0x0F) {
          next_state = IDLE;
          break;
        }

        // initialize some variables
        uint8_t sensor = 0;
        std::array<uint16_t, 8> proximity;
        std::array<float, 8> proxNormalized;
        float factor_x = 0.0f;
        float factor_wz = 0.0f;

        // read proximity values
        for (sensor = 0; sensor < 8; ++sensor) {
          proximity[sensor] = global.vcnl4020[sensor].getProximityScaledWoOffset();
        }

        // normalize proximity values
        for (sensor = 0; sensor < 8; ++sensor) {
          register uint16_t prox = proximity[sensor];
          // limit to high treshold
          if (prox > obstacle_avoidance::proxThresholdHigh)
              prox = obstacle_avoidance::proxThresholdHigh;
          // limit to low threshold
          else if (prox < obstacle_avoidance::proxThresholdLow)
              prox = obstacle_avoidance::proxThresholdLow;
          // apply low threshold
          prox -= obstacle_avoidance::proxThresholdLow;
          // normalize to [0, 1]
          proxNormalized[sensor] = float(prox) / float(obstacle_avoidance::proxRange);
        }

        // map the sensor values to the top LEDs
        for (sensor = 0; sensor < 8; ++sensor) {
          global.robot.setLightColor(obstacle_avoidance::ProxId2LedId(sensor), obstacle_avoidance::Prox2Color(proxNormalized[sensor]));
        }

        // evaluate NAM
        for (sensor = 0; sensor < 8; ++sensor) {
          factor_x += proxNormalized[sensor] * obstacle_avoidance::namMatrix[sensor][0];
          factor_wz += proxNormalized[sensor] * obstacle_avoidance::namMatrix[sensor][1];
        }

        // set motor commands
        kinematic = obstacle_avoidance::defaultKinematic;
        kinematic.x += (factor_x * obstacle_avoidance::baseTranslation) + 0.5f;
        kinematic.w_z += (factor_wz * obstacle_avoidance::baseRotation) + 0.5f;
        global.robot.setTargetSpeed(kinematic);

        break;
      }
      case WII_STEERING:
      {
        // if not yet connected to the Wiimote controller
        if (!wii_steering::wiimote.bluetoothWiimoteIsConnected()) {
          // try to connect
          chprintf((BaseSequentialStream*)&global.sercanmux1, "connecting...\n");
          wii_steering::wiimote.bluetoothWiimoteConnect(wii_steering::bt_address);

          if (wii_steering::wiimote.bluetoothWiimoteIsConnected()) {
            chprintf((BaseSequentialStream*)&global.sercanmux1, "connection established\n");
            chprintf((BaseSequentialStream*)&global.sercanmux1, "\n");
            chprintf((BaseSequentialStream*)&global.sercanmux1, "Wiimote control:\n");
            chprintf((BaseSequentialStream*)&global.sercanmux1, "\tpress 'home' to calibrate\n");
            chprintf((BaseSequentialStream*)&global.sercanmux1, "\thold 'A' to steer\n");
          }
        }
        // steer AMiRo using the Wiimote controller like a joystick
        else {
          // initialize some variables
          float wiimoteAcc[3] = {0.0f, 0.0f, 0.0f};

          // get Wiimote accelerometer data
          wiimoteAcc[0] = wii_steering::wiimote.getAccelerometer()->x_axis;
          wiimoteAcc[1] = wii_steering::wiimote.getAccelerometer()->y_axis;
          wiimoteAcc[2] = wii_steering::wiimote.getAccelerometer()->z_axis;

          // calibrate accelerometer offset
          if (wii_steering::wiimote.getButtons()->home) {
            chprintf((BaseSequentialStream*)&global.sercanmux1, "%f | %f | %f\n", wiimoteAcc[0], wiimoteAcc[1], wiimoteAcc[2]);

            // detect principal axis
            if (std::fabs(wiimoteAcc[0]) > std::fabs(wiimoteAcc[1]) && std::fabs(wiimoteAcc[0]) > std::fabs(wiimoteAcc[2])) {
              wii_steering::principal_axis = 0;
            } else if (std::fabs(wiimoteAcc[1]) > std::fabs(wiimoteAcc[0]) && std::fabs(wiimoteAcc[1]) > std::fabs(wiimoteAcc[2])) {
              wii_steering::principal_axis = 1;
            } else if (std::fabs(wiimoteAcc[2]) > std::fabs(wiimoteAcc[0]) && std::fabs(wiimoteAcc[2]) > std::fabs(wiimoteAcc[1])) {
              wii_steering::principal_axis = 2;
            }
            wii_steering::axis_direction = (wiimoteAcc[wii_steering::principal_axis] >= 0) ? 1 : -1;

            // get calibration offset
            wii_steering::wiimoteCalib[0] = wiimoteAcc[0];
            wii_steering::wiimoteCalib[1] = wiimoteAcc[1];
            wii_steering::wiimoteCalib[2] = wiimoteAcc[2];
            wii_steering::wiimoteCalib[wii_steering::principal_axis] += -100.0f * wii_steering::axis_direction;

            // print information
            chprintf((BaseSequentialStream*)&global.sercanmux1, "accelerometer calibrated:\n");
            chprintf((BaseSequentialStream*)&global.sercanmux1, "\tprincipal axis: %c\n", (wii_steering::principal_axis == 0) ? 'X' : (wii_steering::principal_axis == 1) ? 'Y' : 'Z');
            chprintf((BaseSequentialStream*)&global.sercanmux1, "\tX = %d\n", (int32_t)wii_steering::wiimoteCalib[0]);
            chprintf((BaseSequentialStream*)&global.sercanmux1, "\tY = %d\n", (int32_t)wii_steering::wiimoteCalib[1]);
            chprintf((BaseSequentialStream*)&global.sercanmux1, "\tZ = %d\n", (int32_t)wii_steering::wiimoteCalib[2]);
          }

          for (uint8_t axis = 0; axis < 3; ++axis) {
            // apply calibration values
            wiimoteAcc[axis] -= wii_steering::wiimoteCalib[axis];

            // normalize to (-1, 1)
            wiimoteAcc[axis] /= 100.0f;

            // limit to 1G
            if (wiimoteAcc[axis] > 1.0f) {
              wiimoteAcc[axis] = 1.0f;
            } else if (wiimoteAcc[axis] < -1.0f) {
              wiimoteAcc[axis] = -1.0f;
            }

            // apply deadzone
            if (std::fabs(wiimoteAcc[axis]) < wii_steering::deadzone) {
              wiimoteAcc[axis] = 0.0f;
            }

            /*
             * the value is now in (-1 .. -deazone, 0, deadzone .. 1)
             * note the gaps [-deadzone .. 0] and [0 .. deadzone]
             */

            // normalize (deadzone, 1) to (0, 1) and (-1, -deadzone) tpo (-1, 0)
            if (wiimoteAcc[axis] > 0) {
              wiimoteAcc[axis] -= wii_steering::deadzone;
            } else if (wiimoteAcc[axis] < 0){
              wiimoteAcc[axis] += wii_steering::deadzone;
            }
            wiimoteAcc[axis] *= (1.0f / (1.0f - wii_steering::deadzone));
          }

          // only move when A is pressed
          if (wii_steering::wiimote.getButtons()->A || wii_steering::wiimote.getButtons()->B) {
            // set kinematic relaive to maximum speeds
            switch (wii_steering::principal_axis) {
              case 1:
                if (wii_steering::axis_direction == -1) {
                  kinematic.x = wii_steering::maxTranslation * wiimoteAcc[2];
                  kinematic.w_z = wii_steering::maxRotation * wiimoteAcc[0] * ((wiimoteAcc[2] < 0.0f) ? 1.0f : -1.0f);
                  break;
                }
              case 2:
                if (wii_steering::axis_direction == 1) {
                  kinematic.x = wii_steering::maxTranslation * wiimoteAcc[1];
                  kinematic.w_z = wii_steering::maxRotation * wiimoteAcc[0] * ((wiimoteAcc[1] < 0.0f) ? 1.0f : -1.0f);
                  break;
                }
              default:
                kinematic = {0, 0, 0, 0, 0, 0};
                break;
            }
          } else {
            kinematic = {0, 0, 0, 0, 0, 0};
          }

          // set speed
          global.robot.setTargetSpeed(kinematic);
        }

        break;
      }
    }
  }

  // stop the robot
  kinematic = {0, 0, 0, 0, 0, 0};
  global.robot.setTargetSpeed(kinematic);

  return RDY_OK;
}

void
UserThread::setNextState(const UserThread::State state)
{
  next_state = state;
  return;
}

UserThread::State
UserThread::getCurrenState() const
{
  return current_state;
}

msg_t
UserThread::setWiiAddress(const char* address)
{
  if (strlen(address) != 17) {
    return RDY_RESET;
  }
  else {
    strcpy(wii_steering::bt_address, address);
    return RDY_OK;
  }
}

float
UserThread::setWiiDeadzone(const float deadzone)
{
  // check for negative value and limit to zero
  float dz = (deadzone < 0.0f) ? 0.0f : deadzone;

  // if value is >1, range is assumed to be (0, 100)
  if (dz > 1.0f) {
    // limit to 100
    if (dz > 100.0f) {
      dz = 100.0f;
    }
    dz /= 100.0f;
  }

  // set value and return it
  wii_steering::deadzone = dz;
  return dz;
}

