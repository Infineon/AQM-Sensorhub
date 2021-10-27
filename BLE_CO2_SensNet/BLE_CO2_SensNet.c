/** BLE_CO2_SensNet.c
 *
 */

#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_app_hal_common.h"
#include "wiced_platform.h"
#include "wiced_hal_wdog.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_sdp.h"
#include "wiced_timer.h"
#include "sparcommon.h"
#include "string.h"
#include "hci_control_api.h"
#include "wiced_transport.h"
#include "wiced_hal_pspi.h"
#include "BLE_CO2_SensNet_db.h"
#include "wiced_bt_cfg.h"

#include "../_Libraries/BLE_CO2_CMD.h"

/*******************************************************************
 * Constant Definitions
 ******************************************************************/
#define TRANS_UART_BUFFER_SIZE      1024
#define TRANS_UART_BUFFER_COUNT     2

/*******************************************************************
 * Variable Definitions
 ******************************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];
// Transport pool for sending RFCOMM data to host
static wiced_transport_buffer_pool_t* transport_pool = NULL;

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static void                   ble_co2_sensnet_app_init               ( void );
static wiced_bt_dev_status_t  ble_co2_sensnet_management_callback    ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void                   ble_co2_sensnet_set_advertisement_data ( void );
static void                   ble_co2_sensnet_advertisement_stopped  ( void );
static void                   ble_co2_sensnet_reset_device           ( void );

/* GATT Registration Callbacks */
static wiced_bt_gatt_status_t ble_co2_sensnet_write_handler          ( wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id );
static wiced_bt_gatt_status_t ble_co2_sensnet_read_handler           ( wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id );
static wiced_bt_gatt_status_t ble_co2_sensnet_connect_callback       ( wiced_bt_gatt_connection_status_t *p_conn_status );
static wiced_bt_gatt_status_t ble_co2_sensnet_server_callback        ( uint16_t conn_id, wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data );
static wiced_bt_gatt_status_t ble_co2_sensnet_event_handler          ( wiced_bt_gatt_evt_t  event, wiced_bt_gatt_event_data_t *p_event_data );
static uint32_t               hci_control_process_rx_cmd             ( uint8_t* p_data, uint32_t len );
#ifdef HCI_TRACE_OVER_TRANSPORT
static void                   ble_co2_sensnet_trace_callback         ( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
#endif

/*******************************************************************
 * Macro Definitions
 ******************************************************************/
// Macro to extract uint16_t from little-endian byte array
#define LITTLE_ENDIAN_BYTE_ARRAY_TO_UINT16(byte_array) \
        (uint16_t)( ((byte_array)[0] | ((byte_array)[1] << 8)) )

/*******************************************************************
 * Transport Configuration
 ******************************************************************/
wiced_transport_cfg_t transport_cfg =
{
    WICED_TRANSPORT_UART,              /**< Wiced transport type. */
    {
        WICED_TRANSPORT_UART_HCI_MODE, /**<  UART mode, HCI or Raw */
        HCI_UART_DEFAULT_BAUD          /**<  UART baud rate */
    },
    {
        TRANS_UART_BUFFER_SIZE,        /**<  Rx Buffer Size */
        TRANS_UART_BUFFER_COUNT        /**<  Rx Buffer Count */
    },
    NULL,                              /**< Wiced transport status handler.*/
    hci_control_process_rx_cmd,        /**< Wiced transport receive data handler. */
    NULL                               /**< Wiced transport tx complete callback. */
};

/*******************************************************************
 * GATT Initial Value Arrays
 ******************************************************************/
uint8_t ble_co2_sensnet_generic_access_device_name[]                               = {'B','L','E','_','C','O','2','_','S','e','n','s','N','e','t'};
uint8_t ble_co2_sensnet_generic_access_appearance[]                                = {0x00,0x00};
uint8_t ble_co2_sensnet_xensiv_measurement_co2_data[]                              = {0x00,0x00};
uint8_t ble_co2_sensnet_xensiv_measurement_co2_data_user_description[]             = {'M','e','a','s','u','r','e','d',' ','C','O','2',' ','D','a','t','a'};
uint8_t ble_co2_sensnet_xensiv_measurement_co2_data_client_configuration[]         = {BIT16_TO_8(GATT_CLIENT_CONFIG_NONE)};
uint8_t ble_co2_sensnet_xensiv_measurement_press_data[]                            = {0x00,0x00,0x00,0x00};
uint8_t ble_co2_sensnet_xensiv_measurement_press_data_user_description[]           = {'M','e','a','s','u','r','e','d',' ','P','r','e','s','s','u','r','e',' ','D','a','t','a'};
uint8_t ble_co2_sensnet_xensiv_measurement_press_data_client_configuration[]       = {BIT16_TO_8(GATT_CLIENT_CONFIG_NONE)};
uint8_t ble_co2_sensnet_xensiv_measurement_temp_data[]                             = {0x00,0x00,0x00,0x00};
uint8_t ble_co2_sensnet_xensiv_measurement_temp_data_user_description[]            = {'M','e','a','s','u','r','e','d',' ','T','e','m','p','e','r','a','t','u','r','e',' ','D','a','t','a'};
uint8_t ble_co2_sensnet_xensiv_measurement_temp_data_client_configuration[]        = {BIT16_TO_8(GATT_CLIENT_CONFIG_NONE)};
uint8_t ble_co2_sensnet_xensiv_measurement_hum_data[]                              = {0x00,0x00,0x00,0x00};
uint8_t ble_co2_sensnet_xensiv_measurement_hum_data_user_description[]             = {'M','e','a','s','u','r','e','d',' ','H','u','m','i','d','i','t','y',' ','D','a','t','a'};
uint8_t ble_co2_sensnet_xensiv_measurement_hum_data_client_configuration[]         = {BIT16_TO_8(GATT_CLIENT_CONFIG_NONE)};
uint8_t ble_co2_sensnet_xensiv_config_sample_rate[]                                = {0x00,0x00};
uint8_t ble_co2_sensnet_xensiv_config_sample_rate_user_description[]               = {'M','e','a','s','u','r','e','m','e','n','t',' ','R','a','t','e',' ','S','e','t','t','i','n','g'};
uint8_t ble_co2_sensnet_xensiv_config_sample_rate_client_configuration[]           = {BIT16_TO_8(GATT_CLIENT_CONFIG_NONE)};
uint8_t ble_co2_sensnet_xensiv_config_pressure_compensation[]                      = {0x00,0x00,0x00,0x00};
uint8_t ble_co2_sensnet_xensiv_config_pressure_compensation_user_description[]     = {'P','r','e','s','s','u','r','e',' ','C','o','m','p','e','n','s','a','t','i','o','n',' ','S','e','t','t','i','n','g'};
uint8_t ble_co2_sensnet_xensiv_config_pressure_compensation_client_configuration[] = {BIT16_TO_8(GATT_CLIENT_CONFIG_NONE)};
uint8_t ble_co2_sensnet_xensiv_config_alarm_threshold[]                            = {0x00,0x00};
uint8_t ble_co2_sensnet_xensiv_config_alarm_threshold_user_description[]           = {'A','l','a','r','m',' ','T','h','r','e','s','h','o','l','d',' ','S','e','t','t','i','n','g'};
uint8_t ble_co2_sensnet_xensiv_config_alarm_threshold_client_configuration[]       = {BIT16_TO_8(GATT_CLIENT_CONFIG_NONE)};
uint8_t ble_co2_sensnet_xensiv_config_offset_compensation[]                        = {0x00,0x00,0x00,0x00};
uint8_t ble_co2_sensnet_xensiv_config_offset_compensation_user_description[]       = {'O','f','f','s','e','t',' ','C','o','m','p','e','n','s','a','t','i','o','n',' ','S','e','t','t','i','n','g'};
uint8_t ble_co2_sensnet_xensiv_config_offset_compensation_client_configuration[]   = {BIT16_TO_8(GATT_CLIENT_CONFIG_NONE)};

/*******************************************************************
 * GATT Lookup Table
 ******************************************************************/

/* GATT attribute lookup table                                */
/* (attributes externally referenced by GATT server database) */
gatt_db_lookup_table ble_co2_sensnet_gatt_db_ext_attr_tbl[] =
{
    /* { attribute handle,                                          maxlen,                                                                       curlen,                                                                       attribute data } */
    {HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE,                         15,                                                                           15,                                                                           ble_co2_sensnet_generic_access_device_name},
    {HDLC_GENERIC_ACCESS_APPEARANCE_VALUE,                          2,                                                                            2,                                                                            ble_co2_sensnet_generic_access_appearance},
    {HDLC_XENSIV_MEASUREMENT_CO2_DATA_VALUE,                        2,                                                                            2,                                                                            ble_co2_sensnet_xensiv_measurement_co2_data},
    {HDLD_XENSIV_MEASUREMENT_CO2_DATA_USER_DESCRIPTION,             sizeof(ble_co2_sensnet_xensiv_measurement_co2_data_user_description),         sizeof(ble_co2_sensnet_xensiv_measurement_co2_data_user_description),         ble_co2_sensnet_xensiv_measurement_co2_data_user_description},
    {HDLD_XENSIV_MEASUREMENT_CO2_DATA_CLIENT_CONFIGURATION,         2,                                                                            2,                                                                            ble_co2_sensnet_xensiv_measurement_co2_data_client_configuration},
    {HDLC_XENSIV_MEASUREMENT_PRESS_DATA_VALUE,                      4,                                                                            4,                                                                            ble_co2_sensnet_xensiv_measurement_press_data},
    {HDLD_XENSIV_MEASUREMENT_PRESS_DATA_USER_DESCRIPTION,           sizeof(ble_co2_sensnet_xensiv_measurement_press_data_user_description),       sizeof(ble_co2_sensnet_xensiv_measurement_press_data_user_description),       ble_co2_sensnet_xensiv_measurement_press_data_user_description},
    {HDLD_XENSIV_MEASUREMENT_PRESS_DATA_CLIENT_CONFIGURATION,       2,                                                                            2,                                                                            ble_co2_sensnet_xensiv_measurement_press_data_client_configuration},
    {HDLC_XENSIV_MEASUREMENT_TEMP_DATA_VALUE,                       4,                                                                            4,                                                                            ble_co2_sensnet_xensiv_measurement_temp_data},
    {HDLD_XENSIV_MEASUREMENT_TEMP_DATA_USER_DESCRIPTION,            sizeof(ble_co2_sensnet_xensiv_measurement_temp_data_user_description),        sizeof(ble_co2_sensnet_xensiv_measurement_temp_data_user_description),        ble_co2_sensnet_xensiv_measurement_temp_data_user_description},
    {HDLD_XENSIV_MEASUREMENT_TEMP_DATA_CLIENT_CONFIGURATION,        2,                                                                            2,                                                                            ble_co2_sensnet_xensiv_measurement_temp_data_client_configuration},
    {HDLC_XENSIV_MEASUREMENT_HUM_DATA_VALUE,                        4,                                                                            4,                                                                            ble_co2_sensnet_xensiv_measurement_hum_data},
    {HDLD_XENSIV_MEASUREMENT_HUM_DATA_USER_DESCRIPTION,             sizeof(ble_co2_sensnet_xensiv_measurement_hum_data_user_description),         sizeof(ble_co2_sensnet_xensiv_measurement_hum_data_user_description),         ble_co2_sensnet_xensiv_measurement_hum_data_user_description},
    {HDLD_XENSIV_MEASUREMENT_HUM_DATA_CLIENT_CONFIGURATION,         2,                                                                            2,                                                                            ble_co2_sensnet_xensiv_measurement_hum_data_client_configuration},
    {HDLC_XENSIV_CONFIG_SAMPLE_RATE_VALUE,                          2,                                                                            2,                                                                            ble_co2_sensnet_xensiv_config_sample_rate},
    {HDLD_XENSIV_CONFIG_SAMPLE_RATE_USER_DESCRIPTION,               sizeof(ble_co2_sensnet_xensiv_config_sample_rate_user_description),           sizeof(ble_co2_sensnet_xensiv_config_sample_rate_user_description),           ble_co2_sensnet_xensiv_config_sample_rate_user_description},
    {HDLD_XENSIV_CONFIG_SAMPLE_RATE_CLIENT_CONFIGURATION,           2,                                                                            2,                                                                            ble_co2_sensnet_xensiv_config_sample_rate_client_configuration},
    {HDLC_XENSIV_CONFIG_PRESSURE_COMPENSATION_VALUE,                4,                                                                            4,                                                                            ble_co2_sensnet_xensiv_config_pressure_compensation},
    {HDLD_XENSIV_CONFIG_PRESSURE_COMPENSATION_USER_DESCRIPTION,     sizeof(ble_co2_sensnet_xensiv_config_pressure_compensation_user_description), sizeof(ble_co2_sensnet_xensiv_config_pressure_compensation_user_description), ble_co2_sensnet_xensiv_config_pressure_compensation_user_description},
    {HDLD_XENSIV_CONFIG_PRESSURE_COMPENSATION_CLIENT_CONFIGURATION, 2,                                                                            2,                                                                            ble_co2_sensnet_xensiv_config_pressure_compensation_client_configuration},
    {HDLC_XENSIV_CONFIG_ALARM_THRESHOLD_VALUE,                      2,                                                                            2,                                                                            ble_co2_sensnet_xensiv_config_alarm_threshold},
    {HDLD_XENSIV_CONFIG_ALARM_THRESHOLD_USER_DESCRIPTION,           sizeof(ble_co2_sensnet_xensiv_config_alarm_threshold_user_description),       sizeof(ble_co2_sensnet_xensiv_config_alarm_threshold_user_description),       ble_co2_sensnet_xensiv_config_alarm_threshold_user_description},
    {HDLD_XENSIV_CONFIG_ALARM_THRESHOLD_CLIENT_CONFIGURATION,       2,                                                                            2,                                                                            ble_co2_sensnet_xensiv_config_alarm_threshold_client_configuration},
    {HDLC_XENSIV_CONFIG_OFFSET_COMPENSATION_VALUE,                  4,                                                                            4,                                                                            ble_co2_sensnet_xensiv_config_offset_compensation},
    {HDLD_XENSIV_CONFIG_OFFSET_COMPENSATION_USER_DESCRIPTION,       sizeof(ble_co2_sensnet_xensiv_config_offset_compensation_user_description),   sizeof(ble_co2_sensnet_xensiv_config_offset_compensation_user_description),   ble_co2_sensnet_xensiv_config_offset_compensation_user_description},
    {HDLD_XENSIV_CONFIG_OFFSET_COMPENSATION_CLIENT_CONFIGURATION,   2,                                                                            2,                                                                            ble_co2_sensnet_xensiv_config_offset_compensation_client_configuration},
};

// Number of Lookup Table Entries
const uint16_t ble_co2_sensnet_gatt_db_ext_attr_tbl_size = ( sizeof ( ble_co2_sensnet_gatt_db_ext_attr_tbl ) / sizeof ( gatt_db_lookup_table ) );

/*******************************************************************
 * Function Definitions
 ******************************************************************/
void IRQ_Measurement (uint32_t arg) {
    Current_State = STATE_ACTIVE;
    wiced_hal_gpio_set_pin_output(LED_BLUE, ~wiced_hal_gpio_get_pin_output(LED_BLUE));

    int16_t CO2_Value = 0;
    // float DHT22_Temperature, DHT22_Humidity = 0;
    float SHTC3_Temperature, SHTC3_Temperature_Prec, SHTC3_Humidity, SHTC3_Humidity_Prec = 0;

    BLE_ReadDPS368(&DPS368_Structure);
    BLE_ReadPASCO2(&PASCO2_Structure, &CO2_Value, DPS368_Structure.pressure);
    BLE_ReadSHTC3(&SHTC3_Structure, &SHTC3_Temperature, &SHTC3_Temperature_Prec, &SHTC3_Humidity, &SHTC3_Humidity_Prec);
    // BLE_ReadDHT22(&DHT22_Structure, &DHT22_Temperature, &DHT22_Humidity);

    CO2_Value += PASCO2_Config.solderOffsetCompensation;
    ble_co2_sensnet_xensiv_measurement_co2_data[0] = CO2_Value & 0xFF;     // Low Byte
    ble_co2_sensnet_xensiv_measurement_co2_data[1] = (CO2_Value >> 8);       // High Byte

    ble_co2_sensnet_xensiv_measurement_press_data[0] = (int32_t) DPS368_Structure.pressure & 0xFFFFFFFF;
    ble_co2_sensnet_xensiv_measurement_press_data[1] = ((int32_t) DPS368_Structure.pressure >> 8);
    ble_co2_sensnet_xensiv_measurement_press_data[2] = ((int32_t) DPS368_Structure.pressure >> 16);
    ble_co2_sensnet_xensiv_measurement_press_data[3] = ((int32_t) DPS368_Structure.pressure >> 24);

    ble_co2_sensnet_xensiv_measurement_temp_data[0] = (int16_t) SHTC3_Temperature & 0xFF;
    ble_co2_sensnet_xensiv_measurement_temp_data[1] = ((int16_t) SHTC3_Temperature >> 8);
    ble_co2_sensnet_xensiv_measurement_temp_data[2] = (int16_t) SHTC3_Temperature_Prec & 0xFF;
    ble_co2_sensnet_xensiv_measurement_temp_data[3] = ((int16_t) SHTC3_Temperature_Prec >> 8);

    ble_co2_sensnet_xensiv_measurement_hum_data[0] = (int16_t) SHTC3_Humidity & 0xFF;
    ble_co2_sensnet_xensiv_measurement_hum_data[1] = ((int16_t) SHTC3_Humidity >> 8);
    ble_co2_sensnet_xensiv_measurement_hum_data[2] = (int16_t) SHTC3_Humidity_Prec & 0xFF;
    ble_co2_sensnet_xensiv_measurement_hum_data[3] = ((int16_t) SHTC3_Humidity_Prec >> 8);

    wiced_bt_gatt_send_notification(Connection_ID, HDLC_XENSIV_MEASUREMENT_CO2_DATA_VALUE, sizeof(ble_co2_sensnet_xensiv_measurement_co2_data), ble_co2_sensnet_xensiv_measurement_co2_data );
    wiced_bt_gatt_send_notification(Connection_ID, HDLC_XENSIV_MEASUREMENT_TEMP_DATA_VALUE, sizeof(ble_co2_sensnet_xensiv_measurement_temp_data), ble_co2_sensnet_xensiv_measurement_temp_data );
    wiced_bt_gatt_send_notification(Connection_ID, HDLC_XENSIV_MEASUREMENT_PRESS_DATA_VALUE, sizeof(ble_co2_sensnet_xensiv_measurement_press_data), ble_co2_sensnet_xensiv_measurement_press_data );
    wiced_bt_gatt_send_notification(Connection_ID, HDLC_XENSIV_MEASUREMENT_HUM_DATA_VALUE, sizeof(ble_co2_sensnet_xensiv_measurement_hum_data), ble_co2_sensnet_xensiv_measurement_hum_data );

    if (CO2_Value != 0) {
        if (CO2_Value <= ((float) PASCO2_Config.threshold * 0.7f)) {    // Threshold for Orange LED is 70% of Red LED Threshold
            wiced_hal_gpio_set_pin_output(LED_GREEN, 0);        // Turn ON Green Wing Board LED
            wiced_hal_gpio_set_pin_output(LED_ORANGE, 1);       // Turn OFF Orange Wing Board LED
            wiced_hal_gpio_set_pin_output(LED_RED, 1);          // Turn OFF Red Wing Board LED
        } else if(CO2_Value <= PASCO2_Config.threshold) {               // Threshold for Red LED can be set via Android App
            wiced_hal_gpio_set_pin_output(LED_GREEN, 1);        // Turn OFF Green Wing Board LED
            wiced_hal_gpio_set_pin_output(LED_ORANGE, 0);       // Turn ON Orange Wing Board LED
            wiced_hal_gpio_set_pin_output(LED_RED, 1);          // Turn OFF Red Wing Board LED
        } else {
            wiced_hal_gpio_set_pin_output(LED_GREEN, 1);        // Turn OFF Green Wing Board LED
            wiced_hal_gpio_set_pin_output(LED_ORANGE, 1);       // Turn OFF Orange Wing Board LED
            wiced_hal_gpio_set_pin_output(LED_RED, 0);          // Turn ON Red Wing Board LED
        }
    } else {
        wiced_hal_gpio_set_pin_output(LED_GREEN, 1);        // Turn OFF Green Wing Board LED
        wiced_hal_gpio_set_pin_output(LED_ORANGE, 1);       // Turn OFF Orange Wing Board LED
        wiced_hal_gpio_set_pin_output(LED_RED, 1);          // Turn OFF Red Wing Board LED
    }

    wiced_hal_gpio_set_pin_output(LED_BLUE, ~wiced_hal_gpio_get_pin_output(LED_BLUE));
    Current_State = STATE_IDLE;
}

/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
void application_start(void) {
    wiced_result_t result;
    wiced_bt_device_address_t   bda;

    /* Initialize the transport configuration */
    wiced_transport_init( &transport_cfg );
    /* Initialize Transport Buffer Pool */
    transport_pool = wiced_transport_create_buffer_pool ( TRANS_UART_BUFFER_SIZE, TRANS_UART_BUFFER_COUNT );

    wiced_hal_gpio_set_pin_output(LED_GREEN,    1);     // Turn OFF Green Wing Board LED
    wiced_hal_gpio_set_pin_output(LED_ORANGE,   1);     // Turn OFF Orange Wing Board LED
    wiced_hal_gpio_set_pin_output(LED_RED,      1);     // Turn OFF Red Wing Board LED
    wiced_hal_gpio_set_pin_output(LED_BLUE,     1);     // Turn OFF Green Wing Board LED

    #ifdef DEBUG_MODE
        /* Set Debug UART as WICED_ROUTE_DEBUG_TO_PUART to see debug traces on Peripheral UART (PUART) */
        wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
        Initialize_PUART();
        wiced_hal_puart_print("PAS CO2 Initialized!\r\n");
        wiced_hal_puart_print("APPLICATION_START!\r\n");
    #endif

    if (wiced_hal_read_nvram(CO2_NVRAM_ID, sizeof(PASCO2Config_t), (uint8_t*) &PASCO2_Config, &result)) {     // On Successful Configuration Read (Config already existing)
        if (PASCO2_Config.magicKey != MAGIC_KEY) {          // Check Version of Configuration
            PASCO2_Config.magicKey                  = MAGIC_KEY;
            PASCO2_Config.pressureCompensation      = 950;
            PASCO2_Config.threshold                 = 1400;
            PASCO2_Config.sampleTime                = 2000;
            PASCO2_Config.solderOffsetCompensation  = 0;
            wiced_hal_write_nvram(CO2_NVRAM_ID, sizeof(PASCO2Config_t), (uint8_t*) &PASCO2_Config, &result);  // Write Configuration if Wrong One was Present
        }
    } else {        // On Unsuccessful Configuration Read (No Config existing)
        PASCO2_Config.magicKey                  = MAGIC_KEY;
        PASCO2_Config.pressureCompensation      = 950;
        PASCO2_Config.threshold                 = 1400;
        PASCO2_Config.sampleTime                = 2000;
        PASCO2_Config.solderOffsetCompensation  = 0;
        wiced_hal_write_nvram(CO2_NVRAM_ID, sizeof(PASCO2Config_t), (uint8_t*) &PASCO2_Config, &result);      // Write Configuration for the First Time
    }

    /* Write Initial Values into GATT DB */
    ble_co2_sensnet_xensiv_config_sample_rate[0] = PASCO2_Config.sampleTime & 0xFF;     // Low Byte
    ble_co2_sensnet_xensiv_config_sample_rate[1] = (PASCO2_Config.sampleTime >> 8);     // High Byte

    ble_co2_sensnet_xensiv_config_pressure_compensation[0] = PASCO2_Config.pressureCompensation & 0xFF;
    ble_co2_sensnet_xensiv_config_pressure_compensation[1] = (PASCO2_Config.pressureCompensation >> 8);
    ble_co2_sensnet_xensiv_config_pressure_compensation[2] = (PASCO2_Config.pressureCompensation >> 16);
    ble_co2_sensnet_xensiv_config_pressure_compensation[3] = (PASCO2_Config.pressureCompensation >> 24);

    ble_co2_sensnet_xensiv_config_alarm_threshold[0] = PASCO2_Config.threshold & 0xFF;      // Low Byte
    ble_co2_sensnet_xensiv_config_alarm_threshold[1] = (PASCO2_Config.threshold >> 8);      // High Byte

    ble_co2_sensnet_xensiv_config_offset_compensation[0] = PASCO2_Config.solderOffsetCompensation & 0xFF;
    ble_co2_sensnet_xensiv_config_offset_compensation[1] = (PASCO2_Config.solderOffsetCompensation >> 8);
    ble_co2_sensnet_xensiv_config_offset_compensation[2] = (PASCO2_Config.solderOffsetCompensation >> 16);
    ble_co2_sensnet_xensiv_config_offset_compensation[3] = (PASCO2_Config.solderOffsetCompensation >> 24);

    wiced_hal_i2c_init();       // Initialize the I2C interface
    wiced_hal_i2c_set_speed(I2CM_SPEED_100KHZ);
    wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );

    BLE_InitPASCO2(&PASCO2_Structure, CO2_I2C_ADDRESS, PASCO2_Config.pressureCompensation);
    BLE_InitDPS368(&DPS368_Structure, DPS368_I2C_ADDRESS);
    BLE_InitSHTC3(&SHTC3_Structure, SHTC3_I2C_ADDRESS);
    // BLE_InitDHT22(&DHT22_Structure);

    wiced_init_timer(&Measurement_Timer, &IRQ_Measurement, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER);
    wiced_start_timer(&Measurement_Timer, PASCO2_Config.sampleTime);

    /* Initialize Bluetooth Controller and Host Stack */
    wiced_bt_stack_init(ble_co2_sensnet_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);

    if (wiced_hal_read_nvram(SPP_NVRAM_ID+5, sizeof(wiced_bt_device_address_t), bda, &result) != sizeof(wiced_bt_device_address_t)) {
        unsigned char MAC_Buffer[6];
        for( uint idx = 0; idx < sizeof(MAC_Buffer)/sizeof(MAC_Buffer[0]); ++idx ) {
            MAC_Buffer[idx]  = HEX_To_Digit( BLE_ADDRESS_SETTING[     3 * idx ] ) << 4;
            MAC_Buffer[idx] |= HEX_To_Digit( BLE_ADDRESS_SETTING[ 1 + 3 * idx ] );
        }

        bda[0] = MAC_Buffer[0];          /* Valid static random address should have 2 most significant bits set to 1 */
        bda[1] = MAC_Buffer[1];
        bda[2] = MAC_Buffer[2];
        bda[3] = MAC_Buffer[3];
        bda[4] = MAC_Buffer[4];
        bda[5] = MAC_Buffer[5];

        wiced_hal_write_nvram(SPP_NVRAM_ID+5, sizeof(wiced_bt_device_address_t), bda, &result);
    }

    wiced_bt_set_local_bdaddr(bda , BLE_ADDR_PUBLIC);
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
void ble_co2_sensnet_app_init(void)
{
    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Set Advertisement Data */
    ble_co2_sensnet_set_advertisement_data();

    /* Register with stack to receive GATT callback */
    wiced_bt_gatt_register( ble_co2_sensnet_event_handler );

    /* Initialize GATT Database */
    wiced_bt_gatt_db_init( gatt_database, gatt_database_len );

    /* Start Undirected LE Advertisements on device startup.
     * The corresponding parameters are contained in 'wiced_bt_cfg.c' */
    /* TODO: Make sure that this is the desired behavior. */
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
}

/* Set Advertisement Data */
void ble_co2_sensnet_set_advertisement_data( void )
{
    wiced_bt_ble_advert_elem_t adv_elem[2] = { 0 };
    uint8_t adv_flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem = 0; 

    /* Advertisement Element for Flags */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len = sizeof(uint8_t);
    adv_elem[num_elem].p_data = &adv_flag;
    num_elem++;

    /* Advertisement Element for Name */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len = strlen((const char*)BT_LOCAL_NAME);
    adv_elem[num_elem].p_data = BT_LOCAL_NAME;
    num_elem++;

    /* Set Raw Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/* This function is invoked when advertisements stop */
void ble_co2_sensnet_advertisement_stopped( void )
{
    WICED_BT_TRACE("Advertisement stopped\n");

    /* TODO: Handle when advertisements stop */
}

/* TODO: This function should be called when the device needs to be reset */
void ble_co2_sensnet_reset_device( void )
{
    /* TODO: Clear any additional persistent values used by the application from NVRAM */

    // Reset the device
    wiced_hal_wdog_reset_system( );
}

/* Bluetooth Management Event Handler */
wiced_bt_dev_status_t ble_co2_sensnet_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_dev_status_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_dev_ble_pairing_info_t *p_ble_info = NULL;
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;

    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth Controller and Host Stack Enabled */

#ifdef HCI_TRACE_OVER_TRANSPORT
        // There is a virtual HCI interface between upper layers of the stack and
        // the controller portion of the chip with lower layers of the BT stack.
        // Register with the stack to receive all HCI commands, events and data.
        wiced_bt_dev_register_hci_trace(ble_co2_sensnet_trace_callback);
#endif

        WICED_BT_TRACE("Bluetooth Enabled (%s)\n",
                ((WICED_BT_SUCCESS == p_event_data->enabled.status) ? "success" : "failure"));

        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            /* Bluetooth is enabled */
            wiced_bt_dev_read_local_addr(bda);
            WICED_BT_TRACE("Local Bluetooth Address: [%B]\n", bda);

            /* Perform application-specific initialization */
            ble_co2_sensnet_app_init();
        }
        break;
    case BTM_DISABLED_EVT:
        /* Bluetooth Controller and Host Stack Disabled */
        WICED_BT_TRACE("Bluetooth Disabled\n");
        break;
    case BTM_SECURITY_REQUEST_EVT:
        /* Security Request */
        WICED_BT_TRACE("Security Request\n");
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;
    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        /* Request for Pairing IO Capabilities (BLE) */
        WICED_BT_TRACE("BLE Pairing IO Capabilities Request\n");
        /* No IO Capabilities on this Platform */
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_BOND|BTM_LE_AUTH_REQ_MITM;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys = 0;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
        break;
    case BTM_PAIRING_COMPLETE_EVT:
        /* Pairing is Complete */
        p_ble_info = &p_event_data->pairing_complete.pairing_complete_info.ble;
        WICED_BT_TRACE("Pairing Complete %d.\n", p_ble_info->reason);
        break;
    case BTM_ENCRYPTION_STATUS_EVT:
        /* Encryption Status Change */
        WICED_BT_TRACE("Encryption Status event: bd ( %B ) res %d\n", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        break;
    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* Paired Device Link Keys Request */
        WICED_BT_TRACE("Paired Device Link Request Keys Event\n");
        /* Device/app-specific TODO: HANDLE PAIRED DEVICE LINK REQUEST KEY - retrieve from NVRAM, etc */
#if 0
        if (ble_co2_sensnet_read_link_keys( &p_event_data->paired_device_link_keys_request ))
        {
            WICED_BT_TRACE("Key Retrieval Success\n");
        }
        else
#endif
        /* Until key retrieval implemented above, just fail the request - will cause re-pairing */
        {
            WICED_BT_TRACE("Key Retrieval Failure\n");
            status = WICED_BT_ERROR;
        }
        break;
    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        /* Advertisement State Changed */
        p_adv_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE("Advertisement State Change: %d\n", *p_adv_mode);
        if ( BTM_BLE_ADVERT_OFF == *p_adv_mode )
        {
            ble_co2_sensnet_advertisement_stopped();
        }
        break;
    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        /* Pairing request, TODO: handle confirmation of numeric compare here if desired */
        WICED_BT_TRACE("numeric_value: %d\n", p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
        break;
    default:
        WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x (%d)\n", event, event);
        break;
    }

    return status;
}

/* Get a Value */
wiced_bt_gatt_status_t ble_co2_sensnet_get_value( uint16_t attr_handle, uint16_t conn_id, uint8_t *p_val, uint16_t max_len, uint16_t *p_len )
{
    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < ble_co2_sensnet_gatt_db_ext_attr_tbl_size; i++)
    {
        if (ble_co2_sensnet_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            isHandleInTable = WICED_TRUE;
            // Detected a matching handle in the external lookup table
            if (ble_co2_sensnet_gatt_db_ext_attr_tbl[i].cur_len <= max_len)
            {
                // Value fits within the supplied buffer; copy over the value
                *p_len = ble_co2_sensnet_gatt_db_ext_attr_tbl[i].cur_len;
                memcpy(p_val, ble_co2_sensnet_gatt_db_ext_attr_tbl[i].p_data, ble_co2_sensnet_gatt_db_ext_attr_tbl[i].cur_len);
                res = WICED_BT_GATT_SUCCESS;

                // TODO: Add code for any action required when this attribute is read
                switch ( attr_handle )
                {
                case HDLC_GENERIC_ACCESS_DEVICE_NAME_VALUE:
                    break;
                case HDLC_GENERIC_ACCESS_APPEARANCE_VALUE:
                    break;
                case HDLC_XENSIV_MEASUREMENT_CO2_DATA_VALUE:
                    break;
                case HDLD_XENSIV_MEASUREMENT_CO2_DATA_CLIENT_CONFIGURATION:
                    break;
                case HDLC_XENSIV_MEASUREMENT_PRESS_DATA_VALUE:
                    break;
                case HDLD_XENSIV_MEASUREMENT_PRESS_DATA_CLIENT_CONFIGURATION:
                    break;
                case HDLC_XENSIV_MEASUREMENT_TEMP_DATA_VALUE:
                    break;
                case HDLD_XENSIV_MEASUREMENT_TEMP_DATA_CLIENT_CONFIGURATION:
                    break;
                case HDLC_XENSIV_MEASUREMENT_HUM_DATA_VALUE:
                    break;
                case HDLD_XENSIV_MEASUREMENT_HUM_DATA_CLIENT_CONFIGURATION:
                    break;
                case HDLC_XENSIV_CONFIG_SAMPLE_RATE_VALUE:
                    break;
                case HDLD_XENSIV_CONFIG_SAMPLE_RATE_CLIENT_CONFIGURATION:
                    break;
                case HDLC_XENSIV_CONFIG_PRESSURE_COMPENSATION_VALUE:
                    break;
                case HDLD_XENSIV_CONFIG_PRESSURE_COMPENSATION_CLIENT_CONFIGURATION:
                    break;
                case HDLC_XENSIV_CONFIG_ALARM_THRESHOLD_VALUE:
                    break;
                case HDLD_XENSIV_CONFIG_ALARM_THRESHOLD_CLIENT_CONFIGURATION:
                    break;
                case HDLC_XENSIV_CONFIG_OFFSET_COMPENSATION_VALUE:
                    break;
                case HDLD_XENSIV_CONFIG_OFFSET_COMPENSATION_CLIENT_CONFIGURATION:
                    break;
                }
            }
            else
            {
                // Value to read will not fit within the buffer
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        // TODO: Add code to read value using handles not contained within external lookup table
        // This can apply when the option is enabled to not generate initial value arrays.
        // If the value for the current handle is successfully read then set the result using:
        // res = WICED_BT_GATT_SUCCESS;
        switch ( attr_handle )
        {
        default:
            // The read operation was not performed for the indicated handle
            WICED_BT_TRACE("Read Request to Invalid Handle: 0x%x\n", attr_handle);
            res = WICED_BT_GATT_READ_NOT_PERMIT;
            break;
        }
    }

    return res;
}

/* Set a Value */
wiced_bt_gatt_status_t ble_co2_sensnet_set_value( uint16_t attr_handle, uint16_t conn_id, uint8_t *p_val, uint16_t len )
{
    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < ble_co2_sensnet_gatt_db_ext_attr_tbl_size; i++)
    {
        if (ble_co2_sensnet_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            isHandleInTable = WICED_TRUE;
            // Verify that size constraints have been met
            validLen = (ble_co2_sensnet_gatt_db_ext_attr_tbl[i].max_len >= len);
            if (validLen)
            {
                // Value fits within the supplied buffer; copy over the value
                ble_co2_sensnet_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(ble_co2_sensnet_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                res = WICED_BT_GATT_SUCCESS;

                #ifdef DEBUG_MODE
                    char Buffer[255] = {0};
                #endif

                // TODO: Add code for any action required when this attribute is written
                // For example you may need to write the value into NVRAM if it needs to be persistent
                switch ( attr_handle )
                {
                case HDLD_XENSIV_MEASUREMENT_CO2_DATA_CLIENT_CONFIGURATION:
                    break;
                case HDLD_XENSIV_MEASUREMENT_PRESS_DATA_CLIENT_CONFIGURATION:
                    break;
                case HDLD_XENSIV_MEASUREMENT_TEMP_DATA_CLIENT_CONFIGURATION:
                    break;
                case HDLC_XENSIV_CONFIG_SAMPLE_RATE_VALUE:
                        wiced_stop_timer(&Measurement_Timer);
                        PASCO2_Config.sampleTime = ((ble_co2_sensnet_xensiv_config_sample_rate[1] << 8) + ble_co2_sensnet_xensiv_config_sample_rate[0]);
                        wiced_start_timer(&Measurement_Timer, PASCO2_Config.sampleTime);

                        #ifdef DEBUG_MODE
                            snprintf(Buffer, sizeof(Buffer), "SampleRate Changed! - %d\r\n", PASCO2_Config.sampleTime);
                            wiced_hal_puart_print(Buffer);
                        #endif
                    break;
                case HDLD_XENSIV_CONFIG_SAMPLE_RATE_CLIENT_CONFIGURATION:
                    break;
                case HDLC_XENSIV_CONFIG_PRESSURE_COMPENSATION_VALUE:
                        PASCO2_Config.pressureCompensation =    (ble_co2_sensnet_xensiv_config_pressure_compensation[3] << 24) +
                                                                (ble_co2_sensnet_xensiv_config_pressure_compensation[2] << 16) +
                                                                (ble_co2_sensnet_xensiv_config_pressure_compensation[1] << 8) +
                                                                ble_co2_sensnet_xensiv_config_pressure_compensation[0];
                        pasco2_setPressureCompensation(&PASCO2_Structure, PASCO2_Config.pressureCompensation);

                        #ifdef DEBUG_MODE
                            snprintf(Buffer, sizeof(Buffer), "pressureCompensation Changed! - %d\r\n", PASCO2_Config.pressureCompensation);
                            wiced_hal_puart_print(Buffer);
                        #endif
                    break;
                case HDLD_XENSIV_CONFIG_PRESSURE_COMPENSATION_CLIENT_CONFIGURATION:
                    break;
                case HDLC_XENSIV_CONFIG_ALARM_THRESHOLD_VALUE:
                        PASCO2_Config.threshold = (ble_co2_sensnet_xensiv_config_alarm_threshold[1] << 8) + ble_co2_sensnet_xensiv_config_alarm_threshold[0];;

                        #ifdef DEBUG_MODE
                            snprintf(Buffer, sizeof(Buffer), "threshold Changed! - %d\r\n", PASCO2_Config.threshold);
                            wiced_hal_puart_print(Buffer);
                        #endif
                    break;
                case HDLD_XENSIV_CONFIG_ALARM_THRESHOLD_CLIENT_CONFIGURATION:
                    break;
                case HDLC_XENSIV_CONFIG_OFFSET_COMPENSATION_VALUE:
                        PASCO2_Config.solderOffsetCompensation =    (ble_co2_sensnet_xensiv_config_offset_compensation[3] << 24) +
                                                                    (ble_co2_sensnet_xensiv_config_offset_compensation[2] << 16) +
                                                                    (ble_co2_sensnet_xensiv_config_offset_compensation[1] << 8) +
                                                                    ble_co2_sensnet_xensiv_config_offset_compensation[0];

                        #ifdef DEBUG_MODE
                            snprintf(Buffer, sizeof(Buffer), "solderOffsetCompensation Changed! - %d\r\n", PASCO2_Config.solderOffsetCompensation);
                            wiced_hal_puart_print(Buffer);
                        #endif
                    break;
                case HDLD_XENSIV_CONFIG_OFFSET_COMPENSATION_CLIENT_CONFIGURATION:
                    break;
                }

                wiced_hal_write_nvram(CO2_NVRAM_ID, sizeof(PASCO2Config_t), (uint8_t*) &PASCO2_Config, NULL);  // Write Configuration
            }
            else
            {
                // Value to write does not meet size constraints
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        // TODO: Add code to write value using handles not contained within external lookup table
        // This can apply when the option is enabled to not generate initial value arrays.
        // If the value for the current handle is successfully written then set the result using:
        // res = WICED_BT_GATT_SUCCESS;
        switch ( attr_handle )
        {
        default:
            // The write operation was not performed for the indicated handle
            WICED_BT_TRACE("Write Request to Invalid Handle: 0x%x\n", attr_handle);
            res = WICED_BT_GATT_WRITE_NOT_PERMIT;
            break;
        }
    }

    return res;
}

/* Handles Write Requests received from Client device */
wiced_bt_gatt_status_t ble_co2_sensnet_write_handler( wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Write Request */
    status = ble_co2_sensnet_set_value(p_write_req->handle, conn_id, p_write_req->p_val, p_write_req->val_len);

    return status;
}

/* Handles Read Requests received from Client device */
wiced_bt_gatt_status_t ble_co2_sensnet_read_handler( wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Read Request */
    status = ble_co2_sensnet_get_value(p_read_req->handle, conn_id, p_read_req->p_val, *p_read_req->p_val_len, p_read_req->p_val_len);

    return status;
}

/* GATT Connection Status Callback */
wiced_bt_gatt_status_t ble_co2_sensnet_connect_callback( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

          if ( NULL != p_conn_status )
          {
              if ( p_conn_status->connected )
              {
                  // Device has connected
                  Connection_ID = p_conn_status->conn_id;
                  wiced_hal_gpio_set_pin_output(LED_BLUE, 0);
              }
              else
              {
                  // Device has disconnected
                  Connection_ID = 0;
                  wiced_hal_gpio_set_pin_output(LED_BLUE, 1);

                  /* restart the advertisements */
                  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
              }
              status = WICED_BT_GATT_SUCCESS;
          }

          return status;
}

/* GATT Server Event Callback */
wiced_bt_gatt_status_t ble_co2_sensnet_server_callback( uint16_t conn_id, wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    switch ( type )
    {
    case GATTS_REQ_TYPE_READ:
        status = ble_co2_sensnet_read_handler( &p_data->read_req, conn_id );
        break;
    case GATTS_REQ_TYPE_WRITE:
        status = ble_co2_sensnet_write_handler( &p_data->write_req, conn_id );
        break;
    }

    return status;
}

/* GATT Event Handler */
wiced_bt_gatt_status_t ble_co2_sensnet_event_handler( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_connection_status_t *p_conn_status = NULL;
    wiced_bt_gatt_attribute_request_t *p_attr_req = NULL;

    switch ( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = ble_co2_sensnet_connect_callback( &p_event_data->connection_status );
        break;
    case GATT_ATTRIBUTE_REQUEST_EVT:
        p_attr_req = &p_event_data->attribute_request;
        status = ble_co2_sensnet_server_callback( p_attr_req->conn_id, p_attr_req->request_type, &p_attr_req->data );
        break;
    default:
        status = WICED_BT_GATT_SUCCESS;
        break;
    }

    return status;
}


/* Handle Command Received over Transport */
uint32_t hci_control_process_rx_cmd( uint8_t* p_data, uint32_t len )
{
    uint8_t status = 0;
    uint8_t cmd_status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t opcode = 0;
    uint8_t* p_payload_data = NULL;
    uint8_t payload_length = 0;

    WICED_BT_TRACE("hci_control_process_rx_cmd : Data Length '%d'\n", len);

    // At least 4 bytes are expected in WICED Header
    if ((NULL == p_data) || (len < 4))
    {
        WICED_BT_TRACE("Invalid Parameters\n");
        status = HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    else
    {
        // Extract OpCode and Payload Length from little-endian byte array
        opcode = LITTLE_ENDIAN_BYTE_ARRAY_TO_UINT16(p_data);
        payload_length = LITTLE_ENDIAN_BYTE_ARRAY_TO_UINT16(&p_data[sizeof(uint16_t)]);
        p_payload_data = &p_data[sizeof(uint16_t)*2];

        // TODO : Process received HCI Command based on its Control Group
        // (see 'hci_control_api.h' for additional details)
        switch ( HCI_CONTROL_GROUP(opcode) )
        {
        default:
            // HCI Control Group was not handled
            cmd_status = HCI_CONTROL_STATUS_UNKNOWN_GROUP;
            wiced_transport_send_data(HCI_CONTROL_EVENT_COMMAND_STATUS, &cmd_status, sizeof(cmd_status));
            break;
        }
    }

    // When operating in WICED_TRANSPORT_UART_HCI_MODE or WICED_TRANSPORT_SPI,
    // application has to free buffer in which data was received
    wiced_transport_free_buffer( p_data );
    p_data = NULL;

    return status;
}

#ifdef HCI_TRACE_OVER_TRANSPORT
/* Handle Sending of Trace over the Transport */
void ble_co2_sensnet_trace_callback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    wiced_transport_send_hci_trace( transport_pool, type, length, p_data );
}
#endif
