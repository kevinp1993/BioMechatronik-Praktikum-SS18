#ifndef TYPES_H_
#define TYPES_H_

/*! \brief Types to use if you work with the AMiRo platform
 *
 *  This header contains types which has to be used
 *  if they suit the problem.
 *  All physical constants (therefore all values with a
 *  physical unit) are implicitly in µ iff the variable
 *  is of type integer, unless it is explicitly named in
 *  the variable.
 *  All physical constants (therefore all values with a
 *  physical unit) are implicitly without prefix (e.g. µ)
 *  iff the variable is of type float, unless it is
 *  explicitly named in the variable.
 *  For the types, the SI prefix is used, iff the variable
 *  is of type float and therefor in SI units.
 */

namespace types {

  /**
   * A structure to represent the position and orientation of the robot
   */
  struct position {
    /*@{*/
    int32_t x;   /**< the x coordinate in µm */
    int32_t y;   /**< the y coordinate in µm */
    int32_t z;   /**< the z coordinate in µm */
    int32_t f_x; /**< the f_x orientation in µrad */
    int32_t f_y; /**< the f_y orientation in µrad */
    int32_t f_z; /**< the f_z orientation in µrad */
    /*@}*/
  };

  /**
   * A structure to represent the position and orientation of the robot
   */
  struct positionSI {
    /*@{*/
    float x;   /**< the x coordinate in m */
    float y;   /**< the y coordinate in m */
    float z;   /**< the z coordinate in m */
    float f_x; /**< the f_x coordinate in rad */
    float f_y; /**< the f_y coordinate in rad */
    float f_z; /**< the f_z coordinate in rad */
    /*@}*/
  };

  /**
   * A structure to represent the kinematics of the robot
   */
  struct kinematic {
    /*@{*/
    int32_t x;   /**< the x velocity in µm/s */
    int32_t y;   /**< the y velocity in µm/s */
    int32_t z;   /**< the z velocity in µm/s */
    int32_t w_x; /**< the w_x angular velocity in µrad/s */
    int32_t w_y; /**< the w_y angular velocity in µrad/s */
    int32_t w_z; /**< the w_z angular velocity in µrad/s */
    /*@}*/
  };

  /**
   * A structure to represent the kinematics of the robot
   */
  struct kinematicSI {
    /*@{*/
    float x;   /**< the x velovity in m/s */
    float y;   /**< the y velocity in m/s */
    float z;   /**< the z velocity in m/s */
    float f_x; /**< the f_x angular velocity in rad/s */
    float f_y; /**< the f_y angular velocity in rad/s */
    float f_z; /**< the f_z angular velocity in rad/s */
    /*@}*/
  };

  /**
   * A structure to represent the power status of the robot
   */
  struct power_status {
    /**
     * @brief A union to encode the current charging status
     */
    union ChargingState {
      uint8_t value = 0;
      struct {
        uint8_t powermanagement_plugged_in      : 1;  /**< status flag, whether the PowerManagement board is plugged in (physical plug detected) */
        uint8_t powermanagement_charging        : 1;  /**< status flag, whether the batteries are currently charged via the PowerManagement board */
        uint8_t diwheeldrive_enable_power_path  : 1;  /**< a flag to enable/disable charging via the DiWheelDrive board */
        uint8_t diwheeldrive_charging           : 1;  /**< status flag, whether the batteries are currently charged via the DiWheelDrive board */
        uint8_t vsys_higher_than_9V             : 1;  /**< status flag, whether vsys is currently higher than 9V */
        uint8_t rsvd                            : 3;
      } content;
    };

    /*@{*/
    ChargingState charging_flags; /**< charging status */
    uint8_t state_of_charge;      /**< state of charge of the batteries in percent */
    uint16_t minutes_remaining;   /**< remaining time until the batteries are fully charged/discharged in minutes*/
    uint16_t power_consumption;   /**< current power consumption in mW */
    /*@}*/
  };

}

#endif /* TYPES_ */
