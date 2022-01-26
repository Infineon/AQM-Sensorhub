/*
 * pasco2.c
 *
 *  Created on: 2 Nov 2020
 *      Author: aamark
 */

#include "pasco2.h"

uint8_t pasco2_readRegister(PASCO2_t* co2, uint8_t address)
{
    wiced_hal_i2c_write((uint8_t *) &address, sizeof(address), co2->address);

    uint8_t Recv_Buffer = 0;
    wiced_hal_i2c_read((uint8_t *) &Recv_Buffer, sizeof(Recv_Buffer), co2->address);

    // wiced_hal_i2c_combined_read(UINT8* rx_data, UINT16 rx_data_len, UINT8* tx_data, UINT8 tx_data_len, UINT8 slave)
    // wiced_hal_i2c_combined_read((uint8_t *) Recv_Buffer, sizeof(Recv_Buffer), (uint8_t *) &Data, sizeof(uint8_t), co2->address);
    return Recv_Buffer;
}

void pasco2_read(PASCO2_t* co2, uint8_t address, uint8_t* data, uint8_t size)
{
    wiced_hal_i2c_combined_read((uint8_t *) data, size, (uint8_t *) &address, sizeof(address), co2->address);
}

void pasco2_writeRegister(PASCO2_t* co2, uint8_t reg, uint8_t val)
{
    uint8_t Data[2];
    Data[0] = reg;
    Data[1] = val;

    wiced_hal_i2c_write((uint8_t *) &Data, sizeof(Data), co2->address);
}

void pasco2_write(PASCO2_t* co2, uint8_t address, uint8_t* data, uint8_t size)
{
    uint8_t Data[(size + 1)];

    Data[0] = address;
	for(uint8_t i = 0; i <= size; i++)
	    Data[i + 1] = data[i];

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
	pasco2_writeRegister(co2, SENS_STS, SENS_STS_ORTMP_CLR_MSK);
}

void pasco2_clearVdd12OutOfRange(PASCO2_t* co2)
{
	pasco2_writeRegister(co2, SENS_STS, SENS_STS_ORVS_CLR_MSK);
}

void pasco2_clearCommunicationError(PASCO2_t* co2)
{
	pasco2_writeRegister(co2, SENS_STS, SENS_STS_ICCER_CLR_MSK);
}

int16_t pasco2_getMeasurementPeriod(PASCO2_t* co2)
{
	uint8_t data[2] = {0};
	pasco2_read(co2, MEAS_RATE_H, data, 2);

	return (data[0] << 8) | data[1];
}

void pasco2_setMeasurementPeriod(PASCO2_t* co2, int16_t period)
{
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

void pasco2_setOperationMode(PASCO2_t* co2, pasco2_op_mode_t mode)
{
	uint8_t reg = pasco2_readRegister(co2, MEAS_CFG);

	reg &= ~MEAS_CFG_OP_MODE_MSK;
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
	pasco2_writeRegister(co2, MEAS_STS, MEAS_STS_INT_STS_CLR_MSK);
}

void pasco2_clearAlarmNotification(PASCO2_t*co2)
{
	pasco2_writeRegister(co2, MEAS_STS, MEAS_STS_ALARM_CLR_MSK);
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
	pasco2_writeRegister(co2, SENS_RST, rst);
}
