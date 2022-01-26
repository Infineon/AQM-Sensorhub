/*
 * DPS368.h
 *
 *  Created on: 27 May 2020
 *      Author: aamark
 */

#include "wiced.h"
#include "data_types.h"
#include "wiced_hal_i2c.h"

#ifndef DPS368_H_
#define DPS368_H_

#define DPS368_I2C_ADDRESS 0x76

#define DPS368_PRS_B2 0x00
#define DPS368_PRS_B1 0x01
#define DPS368_PRS_B0 0x02

#define DPS368_TMP_B2 0x03
#define DPS368_TMP_B1 0x04
#define DPS368_TMP_B0 0x05

#define DPS368_PRS_CFG 0x06
#define DPS368_TMP_CFG 0x07
#define DPS368_MEAS_CFG 0x08

#define DPS368_CFG_REG 0x09

#define DPS368_INT_STS  0x0A
#define DPS368_FIFO_STS 0x0B

#define DPS368_RESET 0x0C

#define DPS368_PRODUCTID 0x0D

#define DPS368_COEF 0x10

#define DPS368_COEF_SRCE 0x28

typedef struct DPS368_coeff
{
	int32_t c0;
	int32_t c1;
	int32_t c00;
	int32_t c01;
	int32_t c10;
	int32_t c11;
	int32_t c20;
	int32_t c21;
	int32_t c30;
}DPS368_coeff_t;

typedef enum
{
	DPS368_SUCCESS,
	DPS368_UNKNOWN_DEVICE,
	DPS368_NO_DATA,
	DPS368_ERROR,
}DPS368_state_t;

typedef enum
{
	DPS368_TMP_COEF_ASIC,
	DPS368_TMP_COEF_MEMS,
}DPS368_tmp_coef_src_t;

typedef enum
{
	DPS368_PM_PRC_1x,
	DPS368_PM_PRC_2x,
	DPS368_PM_PRC_4x,
	DPS368_PM_PRC_8x,
	DPS368_PM_PRC_16x,
	DPS368_PM_PRC_32x,
	DPS368_PM_PRC_64x,
	DPS368_PM_PRC_128x,
	DPS368_PM_RATE_1x   = 0x00,
	DPS368_PM_RATE_2x   = 0x10,
	DPS368_PM_RATE_4x   = 0x20,
	DPS368_PM_RATE_8x   = 0x30,
	DPS368_PM_RATE_16x  = 0x40,
	DPS368_PM_RATE_32x  = 0x50,
	DPS368_PM_RATE_64x  = 0x60,
	DPS368_PM_RATE_128x = 0x70,
}DPS368_prs_cfg_t;

typedef enum
{
	DPS368_TMP_PRC_1x,
	DPS368_TMP_PRC_2x,
	DPS368_TMP_PRC_4x,
	DPS368_TMP_PRC_8x,
	DPS368_TMP_PRC_16x,
	DPS368_TMP_PRC_32x,
	DPS368_TMP_PRC_64x,
	DPS368_TMP_PRC_128x,
	DPS368_TMP_RATE_1x   = 0x00,
	DPS368_TMP_RATE_2x   = 0x10,
	DPS368_TMP_RATE_4x   = 0x20,
	DPS368_TMP_RATE_8x   = 0x30,
	DPS368_TMP_RATE_16x  = 0x40,
	DPS368_TMP_RATE_32x  = 0x50,
	DPS368_TMP_RATE_64x  = 0x60,
	DPS368_TMP_RATE_128x = 0x70,
}DPS368_tmp_cfg_t;

typedef enum
{
	DPS368_NONE_RDY = 0x00,
	DPS368_PRS_RDY = 0x10,
	DPS368_TMP_RDY = 0x20,
	DPS368_SENSOR_RDY = 0x40,
	DPS368_COEF_RDY = 0x80
}DPS368_meas_state_t;

typedef struct DPS368
{
	uint8_t               i2c_address;
	DPS368_coeff_t        coeff;
	DPS368_tmp_coef_src_t tmp_coef_src;
	DPS368_prs_cfg_t      prs_cfg;
	DPS368_tmp_cfg_t      tmp_cfg;
	int32_t			      kT;
	int32_t				  kP;
	int32_t				  prs_raw;
	float				  prs_raw_sc;
	float				  pressure;
	int32_t				  tmp_raw;
	float                 tmp_raw_sc;
	float				  temperature;
}DPS368_t;

DPS368_state_t DPS368_init(DPS368_t* dps, uint8_t i2c_address);
DPS368_meas_state_t DPS368_data_ready(DPS368_t* dps);

#endif /* DPS368_H_ */
