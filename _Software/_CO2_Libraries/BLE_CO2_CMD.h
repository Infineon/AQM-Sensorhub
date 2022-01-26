// #include "wiced_hal_puart.h"
// #include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_i2c.h"
#include "wiced_timer.h"
#include "wiced_sleep.h"
#include "wiced_rtos.h"
#include "data_types.h"
#include "wiced.h"

#include "../_CO2_Libraries/SHTC3.h"
#include "../_CO2_Libraries/PASCO2.h"
#include "../_CO2_Libraries/DPS368.h"

/*******************************************************************
 * Debugging Settings
 ******************************************************************/
// #define DEBUG_MODE      1

/*******************************************************************
 * LED PIN Settings
 ******************************************************************/
#define LED_GREEN       WICED_P01
#define LED_ORANGE      WICED_P29
#define LED_RED         WICED_P28
#define LED_BLUE        WICED_P10

/*******************************************************************
 * Constant Definitions
 ******************************************************************/
#define BLE_ADDRESS_SETTING     "11:22:33:44:55:66"
#define MAGIC_KEY               0x4D61726B

#define SPP_NVRAM_ID            WICED_NVRAM_VSID_START
#define CO2_NVRAM_ID            (WICED_NVRAM_VSID_START + 0xE00)

/*******************************************************************
 * Structure Definitions
 ******************************************************************/
/* PAS CO2 Configuration Structure */
typedef struct {
    uint32_t    magicKey;
    uint32_t    threshold;                         // in ppm
    uint16_t    sampleTime;                        // in ms
    int32_t     solderOffsetCompensation;
}PASCO2Config_t;

PASCO2Config_t  PASCO2_Config;
PASCO2_t        PASCO2_Structure;
DPS368_t        DPS368_Structure;
SHTC3_t         SHTC3_Structure;
wiced_timer_t   Measurement_Timer;

uint16_t Connection_ID = 0;

/*******************************************************************
 * Function Declarations
 ******************************************************************/
unsigned char HEX_To_Digit( char ch );
void Initialize_PUART(void);

void BLE_InitPASCO2(PASCO2_t* co2, uint16_t co2_address, uint16_t PressCompensation);
void BLE_ReadPASCO2(PASCO2_t* co2, int16_t* CO2_Value);

void BLE_InitDPS368(DPS368_t* dps368, uint16_t dps368_address);
DPS368_meas_state_t BLE_ReadDPS368(DPS368_t* dps368);

void BLE_InitSHTC3(SHTC3_t* shtc3, uint16_t shtc3_address);
SHTC3_state_t BLE_ReadSHTC3(SHTC3_t* shtc3, float *SHTC3_Temp, float *SHTC3_Temp_Prec, float *SHTC3_Hum, float *SHTC3_Hum_Prec);
