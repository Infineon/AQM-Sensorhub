/*
 * pasco2.c
 *
 *  Created on: 2 Nov 2020
 *      Author: aamark
 */

#include "pasco2.h"
#include "wiced_rtos.h"
#include <stdio.h>

uint8_t pasco2_readRegister(PASCO2_t* co2, uint8_t address)
{
    uint8_t Recv_Buffer = 0;

    // Wait till sensor is ready
    uint8_t reg_sens_sts = SENS_STS;
    do{
        wiced_rtos_delay_milliseconds(100, ALLOW_THREAD_TO_SLEEP );
        wiced_hal_i2c_combined_read(&Recv_Buffer, 1, (uint8_t *) &reg_sens_sts, sizeof(reg_sens_sts), co2->address);
    }while( ((Recv_Buffer & SENS_STS_SEN_RDY_MSK) >> SENS_STS_SEN_RDY_POS) != 1 );

    // Read value only if requested register isn't the previously read SENS_STS register
    if(address != reg_sens_sts){
        wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );
        wiced_hal_i2c_combined_read(&Recv_Buffer, 1, (uint8_t *) &address, sizeof(address), co2->address);
    }

    return Recv_Buffer;
}

void pasco2_read(PASCO2_t* co2, uint8_t address, uint8_t* data, uint8_t size)
{
    // Wait till sensor is ready
    uint8_t reg_sens_sts = SENS_STS;
    do{
        wiced_rtos_delay_milliseconds(100, ALLOW_THREAD_TO_SLEEP );
        wiced_hal_i2c_combined_read((uint8_t *) data, 1, (uint8_t *) &reg_sens_sts, sizeof(reg_sens_sts), co2->address);
    }while( ((*data & SENS_STS_SEN_RDY_MSK) >> SENS_STS_SEN_RDY_POS) != 1 );

    // Read value
    wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );
    wiced_hal_i2c_combined_read((uint8_t *) data, size, (uint8_t *) &address, sizeof(address), co2->address);
}

void pasco2_writeRegister(PASCO2_t* co2, uint8_t reg, uint8_t val)
{
    uint8_t Data[2];
    Data[0] = reg;
    Data[1] = val;

     // Wait till sensor is ready
    uint8_t Recv_Buffer = 0;
    uint8_t reg_sens_sts = SENS_STS;
    do{
        wiced_rtos_delay_milliseconds(100, ALLOW_THREAD_TO_SLEEP );
        wiced_hal_i2c_combined_read(&Recv_Buffer, 1, (uint8_t *) &reg_sens_sts, sizeof(reg_sens_sts), co2->address);
    }while( ((Recv_Buffer & SENS_STS_SEN_RDY_MSK) >> SENS_STS_SEN_RDY_POS) != 1 );

    // Write and check until value is OK
    do{
        wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );
        wiced_hal_i2c_write((uint8_t *) &Data, sizeof(Data), co2->address);

        wiced_rtos_delay_milliseconds(100, ALLOW_THREAD_TO_SLEEP );
        wiced_hal_i2c_combined_read(&Recv_Buffer, 1, (uint8_t *) &reg, sizeof(reg), co2->address);
    }while( Recv_Buffer != val );
}

void pasco2_write(PASCO2_t* co2, uint8_t address, uint8_t* data, uint8_t size)
{
    uint8_t Data[(size + 1)];

    // Wait till sensor is ready
   int8_t Recv_Buffer = 0;
   uint8_t reg_sens_sts = SENS_STS;
   do{
       wiced_rtos_delay_milliseconds(100, ALLOW_THREAD_TO_SLEEP );
       wiced_hal_i2c_combined_read(&Recv_Buffer, 1, (uint8_t *) &reg_sens_sts, sizeof(reg_sens_sts), co2->address);
   }while( ((Recv_Buffer & SENS_STS_SEN_RDY_MSK) >> SENS_STS_SEN_RDY_POS) != 1 );

    // Prepare and write value
    Data[0] = address;
	for(uint8_t i = 0; i <= size; i++)
	    Data[i + 1] = data[i];

	wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );
	wiced_hal_i2c_write((uint8_t *) &Data, sizeof(Data), co2->address);
}

void pasco2_writeClearRegister(PASCO2_t* co2, uint8_t reg, uint8_t val)
{
    // The clear flag of status sticky bits and reset register are read back as 0 when set. Therefore no check executed done here!

    uint8_t Data[2];
    Data[0] = reg;
    Data[1] = val;

     // Wait till sensor is ready
    uint8_t Recv_Buffer = 0;
    uint8_t reg_sens_sts = SENS_STS;
    do{
        wiced_rtos_delay_milliseconds(100, ALLOW_THREAD_TO_SLEEP );
        wiced_hal_i2c_combined_read(&Recv_Buffer, 1, (uint8_t *) &reg_sens_sts, sizeof(reg_sens_sts), co2->address);
    }while( ((Recv_Buffer & SENS_STS_SEN_RDY_MSK) >> SENS_STS_SEN_RDY_POS) != 1 );

    // Write value
    wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );
    wiced_hal_i2c_write((uint8_t *) &Data, sizeof(Data), co2->address);
}

void pasco2_init(PASCO2_t* co2, uint8_t i2cAddress)
{
	co2->address = i2cAddress;
}

uint8_t pasco2_getProductId(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, PROD_ID) & PROD_ID_PROD_MSK) >> PROD_ID_PROD_POS;
}

uint8_t pasco2_getRevisionId(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, PROD_ID) & PROD_ID_REV_MSK) >> PROD_ID_REV_POS;
}

uint8_t pasco2_isSensorReady(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, SENS_STS) & SENS_STS_SEN_RDY_MSK) >> SENS_STS_SEN_RDY_POS;
}

uint8_t pasco2_getPwmDisPinStatus(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, SENS_STS) & SENS_STS_PWM_DIS_ST_MSK) >> SENS_STS_PWM_DIS_ST_POS;
}

uint8_t pasco2_isTemperatureOutOfRange(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, SENS_STS) & SENS_STS_ORTMP_MSK) >> SENS_STS_ORTMP_POS; // SENS_STS_ORTMP_CLR_POS;
}

uint8_t pasco2_isVdd12OutOfRange(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, SENS_STS) & SENS_STS_ORVS_MSK) >> SENS_STS_ORVS_POS;
}

uint8_t pasco2_isCommunicationError(PASCO2_t* co2)
{
    return (pasco2_readRegister(co2, SENS_STS) & SENS_STS_ICCER_MSK) >> SENS_STS_ICCER_POS;
}

void pasco2_clearTemperatureOutOfRange(PASCO2_t* co2)
{
    pasco2_writeClearRegister(co2, SENS_STS, SENS_STS_ORTMP_CLR_MSK);
}

void pasco2_clearVdd12OutOfRange(PASCO2_t* co2)
{
    pasco2_writeClearRegister(co2, SENS_STS, SENS_STS_ORVS_CLR_MSK);
}

void pasco2_clearCommunicationError(PASCO2_t* co2)
{
    pasco2_writeClearRegister(co2, SENS_STS, SENS_STS_ICCER_CLR_MSK);
}

int16_t pasco2_getMeasurementPeriod(PASCO2_t* co2)
{
	uint8_t data[2] = {0};
	pasco2_read(co2, MEAS_RATE_H, data, 2);

	return (data[0] << 8) | data[1];
}

void pasco2_setMeasurementPeriod(PASCO2_t* co2, int16_t period)
{
    // This will restart the measurement when changing the period value. Needed for continuous mode (set OP-Mode IDLE and back to CONTINUOUS)

    // Store current configuration and set mode idle
    uint8_t reg = pasco2_readRegister(co2, MEAS_CFG);
    pasco2_setOperationMode(co2, PASCO2_OP_MODE_IDLE);

    // Set measurement period
	uint8_t data[2] = { period >> 8, period };
	pasco2_write(co2, MEAS_RATE_H, data, 2);

	// Restore configuration
	pasco2_writeRegister(co2, MEAS_CFG, reg);
}

void pasco2_setMeasurementPeriodUnsafe(PASCO2_t* co2, int16_t period)
{
    // Set measurement period without affecting the current operation mode

    uint8_t data[2] = { period >> 8, period };
    pasco2_write(co2, MEAS_RATE_H, data, 2);
}

uint8_t pasco2_getPwmEnable(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, MEAS_CFG) & MEAS_CFG_PWM_OUTEN_MSK) >> MEAS_CFG_PWM_OUTEN_POS;
}

void pasco2_setPwmEnable(PASCO2_t* co2, uint8_t enable)
{
	uint8_t reg = pasco2_readRegister(co2, MEAS_CFG);

	reg &= ~MEAS_CFG_PWM_OUTEN_MSK;

	if (enable)
		reg |= MEAS_CFG_PWM_OUTEN_MSK;

	pasco2_writeRegister(co2, MEAS_CFG, reg);
}

pasco2_pwm_mode_t pasco2_getPwmMode(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, MEAS_CFG) & MEAS_CFG_PWM_MODE_MSK) >> MEAS_CFG_PWM_MODE_POS;
}

void pasco2_setPwmMode(PASCO2_t* co2, pasco2_pwm_mode_t mode)
{
	uint8_t reg = pasco2_readRegister(co2, MEAS_CFG);

	reg &= ~MEAS_CFG_PWM_MODE_MSK;
	reg |= mode << MEAS_CFG_PWM_MODE_POS;

	pasco2_writeRegister(co2, MEAS_CFG, reg);
}

pasco2_boc_cfg_t pasco2_getBaselineOffsetCompensation(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, MEAS_CFG) & MEAS_CFG_BOC_CFG_MSK) >> MEAS_CFG_BOC_CFG_POS;
}

void pasco2_setBaselineOffsetCompensation(PASCO2_t* co2, pasco2_boc_cfg_t boc)
{
	uint8_t reg = pasco2_readRegister(co2, MEAS_CFG);

	reg &= ~MEAS_CFG_BOC_CFG_MSK;
	reg |= boc << MEAS_CFG_BOC_CFG_POS;

	pasco2_writeRegister(co2, MEAS_CFG, reg);
}

pasco2_op_mode_t pasco2_getOperationMode(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, MEAS_CFG) & MEAS_CFG_OP_MODE_MSK) >> MEAS_CFG_OP_MODE_POS;
}

uint8_t pasco2_getRegisterMeas_cfg(PASCO2_t* co2)
{
    return pasco2_readRegister(co2, MEAS_CFG);
}

void pasco2_startFCS(PASCO2_t* co2)
{
    // Configure continuous mode + FC
    uint8_t reg = 0x0A;
    pasco2_writeRegister(co2, MEAS_CFG, reg);
}

void pasco2_setOperationMode(PASCO2_t* co2, pasco2_op_mode_t mode)
{
	uint8_t reg = pasco2_readRegister(co2, MEAS_CFG);

	// Set all operation mode flags zero
	reg &= ~MEAS_CFG_OP_MODE_MSK;
	// Merge current measurement configuration and given mode setting
	reg |= mode << MEAS_CFG_OP_MODE_POS;

	pasco2_writeRegister(co2, MEAS_CFG, reg);
}

int16_t pasco2_getCo2Concentration(PASCO2_t* co2)
{
	uint8_t data[2] = {0};

	pasco2_read(co2, CO2PPM_H, data, 2);

	return (data[0] << 8) | data[1];
}

uint8_t pasco2_isDataReady(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, MEAS_STS) & MEAS_STS_DRDY_MSK) >> MEAS_STS_DRDY_POS;
}

uint8_t pasco2_getIntPinStatus(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, MEAS_STS) & MEAS_STS_INT_STS_MSK) >> MEAS_STS_INT_STS_POS;
}

uint8_t pasco2_getAlarmNotification(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, MEAS_STS) & MEAS_STS_ALARM_MSK) >> MEAS_STS_ALARM_POS;
}

void pasco2_clearIntPinStatus(PASCO2_t* co2)
{
    pasco2_writeClearRegister(co2, MEAS_STS, MEAS_STS_INT_STS_CLR_MSK);
}

void pasco2_clearAlarmNotification(PASCO2_t*co2)
{
    pasco2_writeClearRegister(co2, MEAS_STS, MEAS_STS_ALARM_CLR_MSK);
}

pasco2_int_typ_t pasco2_getIntTyp(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, INT_CFG) & INT_CFG_INT_TYP_MSK) >> INT_CFG_INT_TYP_POS;
}

void pasco2_setIntTyp(PASCO2_t* co2, pasco2_int_typ_t typ)
{
	uint8_t reg = pasco2_readRegister(co2, INT_CFG);

	reg &= ~INT_CFG_INT_TYP_MSK;
	reg |= typ << INT_CFG_INT_TYP_POS;

	pasco2_writeRegister(co2, INT_CFG, reg);
}

pasco2_int_func_t pasco2_getIntFunction(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, INT_CFG) & INT_CFG_INT_FUNC_MSK) >> INT_CFG_INT_FUNC_POS;
}

void pasco2_setIntFunction(PASCO2_t* co2, pasco2_int_func_t func)
{
	uint8_t reg = pasco2_readRegister(co2, INT_CFG);

	reg &= ~INT_CFG_INT_FUNC_MSK;
	reg |= func << INT_CFG_INT_FUNC_POS;

	pasco2_writeRegister(co2, INT_CFG, reg);
}

pasco2_alarm_typ_t pasco2_getAlarm(PASCO2_t* co2)
{
	return (pasco2_readRegister(co2, INT_CFG) & INT_CFG_ALARM_TYP_MSK) >> INT_CFG_ALARM_TYP_POS;
}

void pasco2_setAlarm(PASCO2_t* co2, pasco2_alarm_typ_t alarm)
{
	uint8_t reg = pasco2_readRegister(co2, INT_CFG);

	reg &= ~INT_CFG_ALARM_TYP_MSK;
	reg |= alarm << INT_CFG_ALARM_TYP_POS;

	pasco2_writeRegister(co2, INT_CFG, reg);
}

int16_t pasco2_getAlarmThreshold(PASCO2_t* co2)
{
	uint8_t data[2] = {0};
	pasco2_read(co2, ALARM_TH_H, data, 2);

	return (data[0] << 8) | data[1];
}

void pasco2_setAlarmThreshold(PASCO2_t* co2, int16_t threshold)
{
	uint8_t data[2] = { threshold >> 8, threshold};
	pasco2_write(co2, ALARM_TH_H, data, 2);
}

uint16_t pasco2_getPressureCompensation(PASCO2_t* co2)
{
	uint8_t data[2] = {0};
	pasco2_read(co2, PRESS_REF_H, data, 2);

	return (data[0] << 8) | data[1];
}

void pasco2_setPressureCompensation(PASCO2_t* co2, uint16_t pressure)
{
    // This will restart the measurement when changing the pressure value. Needed for continuous mode (set OP-Mode IDLE and back to CONTINUOUS)

    // Store current configuration and set mode idle
    uint8_t reg = pasco2_readRegister(co2, MEAS_CFG);
    pasco2_setOperationMode(co2, PASCO2_OP_MODE_IDLE);

    // Set pressure compensation
    uint8_t data[2] = { pressure >> 8, pressure};
    pasco2_write(co2, PRESS_REF_H, data, 2);

    // Restore configuration
    pasco2_writeRegister(co2, MEAS_CFG, reg);
}

void pasco2_setPressureCompensationUnsafe(PASCO2_t* co2, uint16_t pressure)
{
    // Set pressure compensation without affecting the current operation mode

	uint8_t data[2] = { pressure >> 8, pressure};
	pasco2_write(co2, PRESS_REF_H, data, 2);
}

int16_t pasco2_getAutomaticBaselineCompensation(PASCO2_t* co2)
{
	uint8_t data[2] = {0};
	pasco2_read(co2, CALIB_REF_H, data, 2);

	return (data[0] << 8) | data[1];
}

void pasco2_setAutomaticBaselineCompensation(PASCO2_t* co2, int16_t ref)
{
	uint8_t data[2] = { ref >> 8, ref};
	pasco2_write(co2, CALIB_REF_H, data, 2);
}

uint8_t pasco2_getScratchPad(PASCO2_t* co2)
{
	return pasco2_readRegister(co2, SCRATCH_PAD);
}

void pasco2_setScratchPad(PASCO2_t* co2, uint8_t scratch)
{
	pasco2_writeRegister(co2, SCRATCH_PAD, scratch);
}

void pasco2_reset(PASCO2_t* co2, pasco2_srtrg_t rst)
{
    pasco2_writeClearRegister(co2, SENS_RST, rst);
}

#define BYTE_TO_BINARY(byte)  (byte & 0x80 ? '1' : '0'), (byte & 0x40 ? '1' : '0'), (byte & 0x20 ? '1' : '0'), (byte & 0x10 ? '1' : '0'), (byte & 0x08 ? '1' : '0'), (byte & 0x04 ? '1' : '0'), (byte & 0x02 ? '1' : '0'), (byte & 0x01 ? '1' : '0')

void pasco2_getAllRegisters(PASCO2_t* co2)
{
    uint8_t val;
    WICED_BT_TRACE("0x00 - %d \n", pasco2_readRegister(co2, 0x00));

    val = pasco2_readRegister(co2,    0x01);
    WICED_BT_TRACE("0x01 - %c%c%c%c %c%c%c%c - %d\n", BYTE_TO_BINARY(val), val);
    WICED_BT_TRACE("0x02 - %d >> 8 \n", pasco2_readRegister(co2, 0x02) << 8);
    WICED_BT_TRACE("0x03 - %d \n", pasco2_readRegister(co2, 0x03));

    val = pasco2_readRegister(co2,    0x04);
    WICED_BT_TRACE("0x04 - %c%c%c%c %c%c%c%c - %d\n", BYTE_TO_BINARY(val), val);
    WICED_BT_TRACE("0x05 - %d >> 8 \n", pasco2_readRegister(co2, 0x05) << 8);
    WICED_BT_TRACE("0x06 - %d \n", pasco2_readRegister(co2, 0x06));

    val = pasco2_readRegister(co2,    0x07);
    WICED_BT_TRACE("0x07 - %c%c%c%c %c%c%c%c - %d\n", BYTE_TO_BINARY(val), val);

    val = pasco2_readRegister(co2,    0x08);
    WICED_BT_TRACE("0x08 - %c%c%c%c %c%c%c%c - %d\n", BYTE_TO_BINARY(val), val);
    WICED_BT_TRACE("0x09 - %d >> 8 \n", pasco2_readRegister(co2, 0x09) << 8);
    WICED_BT_TRACE("0x0A - %d \n", pasco2_readRegister(co2, 0x0A));
    WICED_BT_TRACE("0x0B - %d >> 8 \n", pasco2_readRegister(co2, 0x0B) << 8);
    WICED_BT_TRACE("0x0C - %d \n", pasco2_readRegister(co2, 0x0C));
    WICED_BT_TRACE("0x0D - %d >> 8 \n", pasco2_readRegister(co2, 0x0D) << 8);

    WICED_BT_TRACE("0x0E - %d \n", pasco2_readRegister(co2, 0x0E));
    WICED_BT_TRACE("0x0F - %d \n", pasco2_readRegister(co2, 0x0F));
    WICED_BT_TRACE("0x10 - %d \n", pasco2_readRegister(co2, 0x10));
}
