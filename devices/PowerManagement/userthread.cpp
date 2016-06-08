#include "userthread.h"

#include "global.hpp"
#include <array>

using namespace amiro;

extern Global global;

uint16_t touch;
std::array<uint16_t, 8> proximity;
std::array<float, 8> proxNormalized;

bool running;

uint16_t constexpr proxThresholdLow = 0x0000;
uint16_t constexpr proxThresholdHigh = 0x2000;
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
uint32_t constexpr baseTranslation = 50e3; // 2cm/s
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
    uint8_t sensor = 0;
    float factor_x = 0.0f;
    float factor_wz = 0.0f;
    types::kinematic kinematic = defaultKinematic;

    for (uint8_t led = 0; led < 8; ++led) {
      global.robot.setLightColor(led, Color(Color::BLACK));
    }
    running = false;

    while (!this->shouldTerminate())
    {
        /*
         * read touch sensor values
         */
        touch = global.mpr121.getButtonStatus();

        /*
         * evaluate touch input
         */
        if (touch == 0x0F) {
            if (running) {
                // stop the robot
                running = false;
                kinematic = {0, 0, 0, 0, 0, 0};
                global.robot.setTargetSpeed(kinematic);
            } else {
                // start the robot
                running = true;
            }

            // set all LEDs to white for one second
            for (uint8_t led = 0; led < 8; ++led) {
                global.robot.setLightColor(led, Color(Color::WHITE));
            }
            this->sleep(MS2ST(1000));
            for (uint8_t led = 0; led < 8; ++led) {
                global.robot.setLightColor(led, Color(Color::BLACK));
            }
        }

        if (running) {
            /*
             * read proximity values
             */
            for (sensor = 0; sensor < 8; ++sensor) {
                proximity[sensor] = global.vcnl4020[sensor].getProximityScaledWoOffset();
                //proxNormalized[sensor] += 2.0f * (proxNormalized[sensor] * (1.0f - proxNormalized[sensor])); // non linearity
            }

            /*
             * normalize proximity values
             */
            for (sensor = 0; sensor < 8; ++sensor) {
                register uint16_t prox = proximity[sensor];
                // limit to high treshold
                if (prox > proxThresholdHigh)
                    prox = proxThresholdHigh;
                // limit to low threshold
                else if (prox < proxThresholdLow)
                    prox = proxThresholdLow;
                // apply low threshold
                prox -= proxThresholdLow;
                // normalize to [0, 1]
                proxNormalized[sensor] = float(prox) / float(proxRange);
            }

            /*
             * map the sensor values to the top LEDs
             */
            for (sensor = 0; sensor < 8; ++sensor) {
                global.robot.setLightColor(ProxId2LedId(sensor), Prox2Color(proxNormalized[sensor]));
            }

            /*
             * evaluate NAM
             */
            factor_x = 0.0f;
            factor_wz = 0.0f;
            for (sensor = 0; sensor < 8; ++sensor) {
                factor_x += proxNormalized[sensor] * namMatrix[sensor][0];
                factor_wz += proxNormalized[sensor] * namMatrix[sensor][1];
            }

            /*
             * set motor commands
             */
            kinematic = defaultKinematic;
            kinematic.x += (factor_x * baseTranslation) + 0.5f;
            kinematic.w_z += (factor_wz * baseRotation) + 0.5f;
            global.robot.setTargetSpeed(kinematic);
        }

        this->sleep(MS2ST(100));
    }

  return RDY_OK;
}

