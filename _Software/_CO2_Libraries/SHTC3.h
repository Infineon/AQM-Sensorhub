#ifndef SHTC3_H
#define SHTC3_H

#include "wiced.h"
#include "data_types.h"
#include "wiced_hal_i2c.h"

#define SHTC3_I2C_ADDRESS 0x70
#define CRC_POLYNOMIAL  0x131       // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

//==============================================================================

typedef struct SHTC3 {
    uint8_t         i2c_address;
    uint16_t        sensor_id;
}SHTC3_t;

//==============================================================================

typedef enum {
    SHTC3_NO_ERROR       = 0x00,
    SHTC3_ACK_ERROR      = 0x01,
    SHTC3_CHECKSUM_ERROR = 0x02
}SHTC3_state_t;

typedef enum {
  READ_ID            = 0xEFC8,
  SOFT_RESET         = 0x805D,
  SLEEP              = 0xB098,
  WAKEUP             = 0x3517,
  MEAS_T_RH_POLLING  = 0x7866,
  MEAS_T_RH_CLOCKSTR = 0x7CA2,
  MEAS_RH_T_POLLING  = 0x58E0,
  MEAS_RH_T_CLOCKSTR = 0x5C24
}SHTC3_commands_t;

//==============================================================================

SHTC3_state_t SHTC3_Init(SHTC3_t* SHTC3, uint8_t i2c_address, uint16_t sensor_id);
SHTC3_state_t SHTC3_GetID(SHTC3_t* SHTC3);
SHTC3_state_t SHTC3_GetTempAndHumi(SHTC3_t* SHTC3, float *temp, float *humi);
SHTC3_state_t SHTC3_Sleep(SHTC3_t* SHTC3);
SHTC3_state_t SHTC3_Wakeup(SHTC3_t* SHTC3);
SHTC3_state_t SHTC3_SoftReset(SHTC3_t* SHTC3);

#endif
