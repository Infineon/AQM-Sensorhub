#include "SHTC3.h"

static uint8_t SHTC3_readRegister(SHTC3_t* SHTC3, uint8_t address);
static SHTC3_state_t SHTC3_WriteCommand(SHTC3_t* SHTC3, SHTC3_commands_t cmd);
static SHTC3_state_t SHTC3_CheckCRC(uint8_t data[], uint8_t nbrOfBytes, uint8_t Checksum);
static float SHTC3_CalcTemperature(uint16_t rawValue);
static float SHTC3_CalcHumidity(uint16_t rawValue);

//------------------------------------------------------------------------------
SHTC3_state_t SHTC3_Init(SHTC3_t* SHTC3, uint8_t i2c_address, uint16_t sensor_id){
    SHTC3->i2c_address = i2c_address;
    SHTC3->sensor_id = sensor_id;

    return SHTC3_NO_ERROR;
}

//------------------------------------------------------------------------------
SHTC3_state_t SHTC3_GetTempAndHumi(SHTC3_t* SHTC3, float *temp, float *humi){
    uint16_t rawValueTemp;
    uint16_t rawValueHumi;

    uint8_t Recv_Buffer[6] = {0};
    uint8_t reg[2] = {0};
    reg[0] = MEAS_T_RH_CLOCKSTR >> 8;
    reg[1] = MEAS_T_RH_CLOCKSTR & 0xFF;

    wiced_hal_i2c_write((uint8_t *) &reg, sizeof(reg), SHTC3->i2c_address);
    wiced_hal_i2c_read((uint8_t *) &Recv_Buffer, sizeof(Recv_Buffer), SHTC3->i2c_address);

    rawValueTemp = (Recv_Buffer[0] << 8) | Recv_Buffer[1];
    rawValueHumi = (Recv_Buffer[3] << 8) | Recv_Buffer[4];

    if (!SHTC3_CheckCRC(&Recv_Buffer[0], 2, Recv_Buffer[2]) && !SHTC3_CheckCRC(&Recv_Buffer[3], 2, Recv_Buffer[5])) {
        *temp = SHTC3_CalcTemperature(rawValueTemp);
        *humi = SHTC3_CalcHumidity(rawValueHumi);
        return SHTC3_NO_ERROR;
    } else {
        *temp = 0;
        *humi = 0;
        return SHTC3_CHECKSUM_ERROR;
    }
}

//------------------------------------------------------------------------------
SHTC3_state_t SHTC3_GetID(SHTC3_t* SHTC3){
    uint8_t Recv_Buffer[3] = {0};
    uint8_t reg[2] = {0};
    reg[0] = READ_ID >> 8;
    reg[1] = READ_ID & 0xFF;

    wiced_hal_i2c_combined_read((uint8_t *) &Recv_Buffer, sizeof(Recv_Buffer), (uint8_t *) &reg, sizeof(reg), SHTC3_I2C_ADDRESS);
    uint16_t ID = (Recv_Buffer[0] << 8) | Recv_Buffer[1];

    uint8_t Data_Bytes[2] = {0};
    Data_Bytes[0] = Recv_Buffer[0];
    Data_Bytes[1] = Recv_Buffer[1];

    if (SHTC3_CheckCRC(Data_Bytes, 2, Recv_Buffer[2]))      // Check CRC for Errors
        return SHTC3_CHECKSUM_ERROR;
    else {
        if ((ID & 0x083F) != 0x0807) {    // Check correct ID
            SHTC3->sensor_id = 0;
            return SHTC3_ACK_ERROR;
        } else {
            SHTC3->sensor_id = (ID & 0x083F);
            return SHTC3_NO_ERROR;
        }
    }
}

//------------------------------------------------------------------------------
SHTC3_state_t SHTC3_Sleep(SHTC3_t* SHTC3) {
    return SHTC3_WriteCommand(&SHTC3, SLEEP);       // Send SLEEP Command to SHTC3
}

//------------------------------------------------------------------------------
SHTC3_state_t SHTC3_Wakeup(SHTC3_t* SHTC3) {
    return SHTC3_WriteCommand(&SHTC3, WAKEUP);       // Send WAKEUP Command to SHTC3
}

//------------------------------------------------------------------------------
SHTC3_state_t SHTC3_SoftReset(SHTC3_t* SHTC3){
    return SHTC3_WriteCommand(&SHTC3, SOFT_RESET);       // Send SOFT_RESET Command to SHTC3
}

//------------------------------------------------------------------------------
uint8_t SHTC3_readRegister(SHTC3_t* SHTC3, uint8_t address){
    wiced_hal_i2c_write((uint8_t *) &address, sizeof(address), SHTC3->i2c_address);

    uint16_t Recv_Buffer = 0;
    wiced_hal_i2c_read((uint8_t *) &Recv_Buffer, sizeof(Recv_Buffer), SHTC3->i2c_address);

    return Recv_Buffer;
}
//------------------------------------------------------------------------------
void SHTC3_read(SHTC3_t* SHTC3, uint8_t address, uint8_t* data, uint8_t size) {
    wiced_hal_i2c_combined_read((uint8_t *) data, size, (uint8_t *) &address, sizeof(address), SHTC3->i2c_address);
}
//------------------------------------------------------------------------------
static SHTC3_state_t SHTC3_WriteCommand(SHTC3_t* SHTC3, SHTC3_commands_t cmd){
  SHTC3_state_t Error;      // Error code

  uint8_t Data[2];
  Data[0] = cmd >> 8;
  Data[1] = cmd & 0xFF;

  Error = wiced_hal_i2c_write((uint8_t *) &Data, sizeof(Data), SHTC3->i2c_address);
  return Error;
}

//------------------------------------------------------------------------------
static SHTC3_state_t SHTC3_CheckCRC(uint8_t data[], uint8_t nbrOfBytes, uint8_t Checksum) {
    uint8_t CRC = 0xFF;     // Initialize Checksum
    uint8_t byteCtr;        // Byte Counter

    // Calculates 8-Bit Checksum with given CRC Polynomial
    for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
        CRC ^= (data[byteCtr]);

        for(uint8_t Bit = 8; Bit > 0; --Bit) {
            if(CRC & 0x80)
                CRC = (CRC << 1) ^ CRC_POLYNOMIAL;
            else
                CRC = (CRC << 1);
        }
    }

    if(CRC != Checksum)     // Verify checksum & Return Error code
        return SHTC3_CHECKSUM_ERROR;
    else
        return SHTC3_NO_ERROR;
}

//------------------------------------------------------------------------------
static float SHTC3_CalcTemperature(uint16_t rawValue){
  return (175 * (float) rawValue / 65536.0f - 45.0f);     // Calculate Temperature in degC - T = (175 * rawValue / 2^16) - 45
}

//------------------------------------------------------------------------------
static float SHTC3_CalcHumidity(uint16_t rawValue){
  return (100 * (float) rawValue / 65536.0f);     // Calculate Relative Humidity in % - RH = 100 * (rawValue / 2^16)
}
