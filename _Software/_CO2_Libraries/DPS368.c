/*
 * DPS368.c
 *
 *  Created on: 27 May 2020
 *      Author: aamark
 */

#include "DPS368.h"

const int32_t DPS368_Scale_Factors[] =
{
	524288,
	1572864,
	3670016,
	7864320,
	253952,
	516096,
	1040384,
	2088960
};

void DPS368_get_twos_complement(int32_t* value, uint8_t len)
{
	if (*value & ((uint32_t)1 << (len - 1)))
		*value -= (uint32_t)1 << len;
}

DPS368_state_t DPS368_get_data(DPS368_t* dps, uint8_t reg, uint8_t* data, int8_t len)
{
	 // wiced_hal_i2c_combined_read(UINT8* rx_data, UINT16 rx_data_len, UINT8* tx_data, UINT8 tx_data_len, UINT8 slave)
	wiced_hal_i2c_combined_read((uint8_t *) data, len, (uint8_t *) &reg, sizeof(uint8_t), dps->i2c_address);

	return DPS368_SUCCESS;
}

DPS368_state_t DPS368_set_reg(DPS368_t* dps, uint8_t reg, uint8_t val)
{
	uint8_t Reg_Val[2];
	Reg_Val[0] = reg;
	Reg_Val[1] = val;

	wiced_hal_i2c_write((uint8_t *) &Reg_Val, 2, dps->i2c_address);

	return DPS368_SUCCESS;
}

uint8_t DPS368_get_reg(DPS368_t* dps, uint8_t reg)
{
	uint8_t data = 0;

	wiced_hal_i2c_combined_read((uint8_t *) &data, sizeof(data), (uint8_t *) &reg, sizeof(uint8_t), dps->i2c_address);

	return data;
}

DPS368_state_t DPS368_init(DPS368_t* dps, uint8_t i2c_address)
{
	DPS368_state_t state = DPS368_ERROR;
	dps->i2c_address = i2c_address;

	wiced_rtos_delay_milliseconds(200);         // delay(100);

	uint8_t product_id = DPS368_get_reg(dps, DPS368_PRODUCTID);

	if (product_id == 0x10)
	{
		DPS368_meas_state_t state = 0;

		//Softreset
		DPS368_set_reg(dps, DPS368_RESET, 0b1001);
		do {
		    wiced_rtos_delay_milliseconds(200);         // delay(100);
			state = DPS368_get_reg(dps, DPS368_MEAS_CFG);
		}while(!(state & (DPS368_COEF_RDY | DPS368_SENSOR_RDY)));

		//get calibrated temp sensor
		dps->tmp_coef_src = DPS368_get_reg(dps, DPS368_COEF_SRCE) >> 7;

		//get calibration coefficients
		uint8_t coeff[18] = {0};
		DPS368_get_data(dps, DPS368_COEF, coeff, sizeof(coeff));
        wiced_rtos_delay_milliseconds(1000);         // delay(100);
        DPS368_get_data(dps, DPS368_COEF, coeff, sizeof(coeff));

		dps->coeff.c0 = (coeff[0] << 4) | (coeff[1] >> 4);
		DPS368_get_twos_complement(&dps->coeff.c0, 12);
		dps->coeff.c1 = ((coeff[1] & 0x0F) << 8) | coeff[2];
		DPS368_get_twos_complement(&dps->coeff.c1, 12);
		dps->coeff.c00 = (coeff[3] << 12) | (coeff[4] << 4) | (coeff[5] >> 4);
		DPS368_get_twos_complement(&dps->coeff.c00, 20);
		dps->coeff.c10 = ((coeff[5] & 0x0F) << 16) | (coeff[6] << 8) | coeff[7];
		DPS368_get_twos_complement(&dps->coeff.c10, 20);
		dps->coeff.c01 = (coeff[8] << 8) | coeff[9];
		DPS368_get_twos_complement(&dps->coeff.c01, 16);
		dps->coeff.c11 = (coeff[10] << 8) | coeff[11];
		DPS368_get_twos_complement(&dps->coeff.c11, 16);
		dps->coeff.c20 = (coeff[12] << 8) | coeff[13];
		DPS368_get_twos_complement(&dps->coeff.c20, 16);
		dps->coeff.c21 = (coeff[14] << 8) | coeff[15];
		DPS368_get_twos_complement(&dps->coeff.c21, 16);
		dps->coeff.c30 = (coeff[16] << 8) | coeff[17];
		DPS368_get_twos_complement(&dps->coeff.c30, 16);

		//pressure configuration
		dps->prs_cfg = DPS368_PM_RATE_16x | DPS368_PM_PRC_16x;
	//	dps->prs_cfg = DPS368_PM_RATE_1x | DPS368_PM_PRC_1x;
		DPS368_set_reg(dps, DPS368_PRS_CFG, dps->prs_cfg);

		//temperature configuration
		dps->tmp_cfg = DPS368_TMP_RATE_1x | DPS368_TMP_PRC_16x;
	//	dps->tmp_cfg = DPS368_TMP_RATE_1x | DPS368_TMP_PRC_1x;
		DPS368_set_reg(dps, DPS368_TMP_CFG, dps->tmp_cfg | (dps->tmp_coef_src << 7));

		//interrupt configuration
		uint8_t cfg_reg = 0;

		if ((dps->prs_cfg & 0x0F) > DPS368_PM_PRC_8x)
			cfg_reg |= 1 << 2;
		if ((dps->tmp_cfg & 0x0F) > DPS368_TMP_PRC_8x)
			cfg_reg |= 1 << 3;

		dps->kP = DPS368_Scale_Factors[(dps->prs_cfg & 0x0F)];
		dps->kT = DPS368_Scale_Factors[(dps->tmp_cfg & 0x0F)];

		DPS368_set_reg(dps, DPS368_CFG_REG, cfg_reg);

		//fix IC with a fuse problem (check if needed with DPS368)
		DPS368_set_reg(dps, 0x0E, 0xA5);
		DPS368_set_reg(dps, 0x0F, 0x96);
		DPS368_set_reg(dps, 0x62, 0x02);
		DPS368_set_reg(dps, 0x0E, 0x00);
		DPS368_set_reg(dps, 0x0F, 0x00);

		//measurement configuration
		DPS368_set_reg(dps, DPS368_MEAS_CFG, 0x07);			// continuous temperature and pressure measurement
	}
	else
		state = DPS368_UNKNOWN_DEVICE;

	return state;
}

DPS368_meas_state_t DPS368_data_ready(DPS368_t* dps)
{
	uint8_t data[3] = {0};

	DPS368_meas_state_t meas_state = DPS368_get_reg(dps, DPS368_MEAS_CFG) & 0xF0;

	if (meas_state & DPS368_TMP_RDY)
	{
		DPS368_get_data(dps, DPS368_TMP_B2, data, 3);
		dps->tmp_raw = (data[0] << 16) | (data[1] << 8) | data[2];
		DPS368_get_twos_complement(&dps->tmp_raw, 24);
	}

	if (meas_state & DPS368_PRS_RDY)
	{
		DPS368_get_data(dps, DPS368_PRS_B2, data, 3);
		dps->prs_raw = (data[0] << 16) | (data[1] << 8) | data[2];
		DPS368_get_twos_complement(&dps->prs_raw, 24);
	}

	dps->tmp_raw_sc = (float)dps->tmp_raw / dps->kT;
	dps->prs_raw_sc = (float)dps->prs_raw / dps->kP;

	dps->pressure = dps->coeff.c00 + dps->prs_raw_sc * (dps->coeff.c10 + dps->prs_raw_sc * (dps->coeff.c20 + dps->prs_raw_sc * dps->coeff.c30)) +
			        dps->tmp_raw_sc * dps->coeff.c01 + dps->tmp_raw_sc * dps->prs_raw_sc * (dps->coeff.c11 + dps->prs_raw_sc * dps->coeff.c21);

	dps->temperature = dps->coeff.c0 / 2 + dps->coeff.c1 * dps->tmp_raw_sc;

	return meas_state;
}
