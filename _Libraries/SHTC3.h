#ifndef SHTC3_H
#define SHTC3_H

#include "wiced.h"
#include "data_types.h"
#include "wiced_hal_i2c.h"

//==============================================================================

typedef struct SHTC3 {
    uint8_t         i2c_address;
    uint16_t        sensor_id;
}SHTC3_t;

//==============================================================================

typedef enum {
    SHTC3_NO_ERROR       = 0x00,    // no error
    SHTC3_ACK_ERROR      = 0x01,    // no acknowledgment error
    SHTC3_CHECKSUM_ERROR = 0x02     // checksum mismatch error
}SHTC3_state_t;

typedef enum {
  READ_ID            = 0xEFC8,  // command: read ID register
  SOFT_RESET         = 0x805D,  // soft reset
  SLEEP              = 0xB098,  // sleep
  WAKEUP             = 0x3517,  // wakeup
  MEAS_T_RH_POLLING  = 0x7866,  // meas. read T first, clock stretching disabled
  MEAS_T_RH_CLOCKSTR = 0x7CA2,  // meas. read T first, clock stretching enabled
  MEAS_RH_T_POLLING  = 0x58E0,  // meas. read RH first, clock stretching disabled
  MEAS_RH_T_CLOCKSTR = 0x5C24   // meas. read RH first, clock stretching enabled
}SHTC3_commands_t;

//==============================================================================

#define SHTC3_I2C_ADDRESS 0x70
#define CRC_POLYNOMIAL  0x131       // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

//==============================================================================
SHTC3_state_t SHTC3_Init(SHTC3_t* SHTC3, uint8_t i2c_address, uint16_t sensor_id);
//==============================================================================
// Initializes the I2C bus for communication with the sensor.
//------------------------------------------------------------------------------

//==============================================================================
SHTC3_state_t SHTC3_GetID(SHTC3_t* SHTC3);
//==============================================================================
// Gets the ID from the sensor.
//------------------------------------------------------------------------------
// input:  *id          pointer to a integer, where the id will be stored
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      NO_ERROR       = no error

//==============================================================================
SHTC3_state_t SHTC3_GetTempAndHumi(SHTC3_t* SHTC3, float *temp, float *humi);
//==============================================================================
// Gets the temperature [C] and the humidity [%RH].
//------------------------------------------------------------------------------
// input:  *temp        pointer to a floating point value, where the calculated
//                      temperature will be stored
//         *humi        pointer to a floating point value, where the calculated
//                      humidity will be stored
//
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      CHECKSUM_ERROR = checksum mismatch
//                      NO_ERROR       = no error
//
// remark: If you use this function, then the sensor blocks the I2C-bus with
//         clock stretching during the measurement.

//==============================================================================
SHTC3_state_t SHTC3_Sleep(SHTC3_t* SHTC3);
SHTC3_state_t SHTC3_Wakeup(SHTC3_t* SHTC3);
//==============================================================================

//==============================================================================
SHTC3_state_t SHTC3_SoftReset(SHTC3_t* SHTC3);
//==============================================================================
// Calls the soft reset mechanism that forces the sensor into a well-defined
// state without removing the power supply.
//------------------------------------------------------------------------------
// return: error:       ACK_ERROR      = no acknowledgment from sensor
//                      NO_ERROR       = no error

#endif
