#include "BLE_CO2_CMD.h"

/*
    CHANGES TO BLE_CO2_SensNet.c
        * Add       #include "../_Libraries/BLE_CO2_CMD.h"
        * Insert    "application_start" and "IRQ_Measurement" Functions found below
        * Adjust    "ble_co2_sensnet_connect_callback" as found below

    CHANGES TO wiced_bt_cfg.c
        * Change BLE Advertisement Settings high and low_duty_durations to 0 (infinite)
        * Change .rpa_refresh_timeout to WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE
*/

/*******************************************************************************************************************************
 * void application_start(void)
 ******************************************************************************************************************************/
/*  void application_start(void) {
        wiced_result_t result;
        wiced_bt_device_address_t   bda;

        /* Initialize the transport configuration
        wiced_transport_init( &transport_cfg );
        /* Initialize Transport Buffer Pool
        transport_pool = wiced_transport_create_buffer_pool ( TRANS_UART_BUFFER_SIZE, TRANS_UART_BUFFER_COUNT );

        wiced_hal_gpio_set_pin_output(LED_GREEN, 1);        // Turn OFF Green Wing Board LED
        wiced_hal_gpio_set_pin_output(LED_ORANGE, 1);       // Turn OFF Orange Wing Board LED
        wiced_hal_gpio_set_pin_output(LED_RED, 1);          // Turn OFF Red Wing Board LED
        wiced_hal_gpio_set_pin_output(LED_BLUE, 1);         // Turn OFF Green Wing Board LED

        /* Set Debug UART as WICED_ROUTE_DEBUG_TO_PUART to see debug traces on Peripheral UART (PUART)
        wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
        Initialize_PUART();

        #ifdef DEBUG_MODE
            wiced_hal_puart_print("PAS CO2 Initialized!\r\n");
            wiced_hal_puart_print("APPLICATION_START!\r\n");
        #endif

        if (wiced_hal_read_nvram(CO2_NVRAM_ID, sizeof(PASCO2Config_t), (uint8_t*) &PASCO2_Config, &result)) {     // On Successful Configuration Read (Config already existing)
            if (PASCO2_Config.magicKey != MAGIC_KEY) {                                              // Check Version of Configuration
                PASCO2_Config.magicKey                  = MAGIC_KEY;
                PASCO2_Config.pressureCompensation      = 950;
                PASCO2_Config.threshold                 = 1000;
                PASCO2_Config.sampleTime                = 2000;
                PASCO2_Config.solderOffsetCompensation  = 0;
                wiced_hal_write_nvram(CO2_NVRAM_ID, sizeof(PASCO2Config_t), (uint8_t*) &PASCO2_Config, &result);  // Write Configuration if Wrong One was Present
            }
        } else {        // On Unsuccessful Configuration Read (No Config existing)
            PASCO2_Config.magicKey                  = MAGIC_KEY;
            PASCO2_Config.pressureCompensation      = 950;
            PASCO2_Config.threshold                 = 1000;
            PASCO2_Config.sampleTime                = 2000;
            PASCO2_Config.solderOffsetCompensation  = 0;
            wiced_hal_write_nvram(CO2_NVRAM_ID, sizeof(PASCO2Config_t), (uint8_t*) &PASCO2_Config, &result);      // Write Configuration for the First Time
        }

        /* Write Initial Values into GATT DB
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

        BLE_InitPASCO2(&PASCO2_Structure, CO2_I2C_ADDRESS, 950);
        BLE_InitDPS368(&DPS368_Structure, DPS368_I2C_ADDRESS);
        BLE_InitSHTC3(&SHTC3_Structure, SHTC3_I2C_ADDRESS);
        // BLE_InitDHT22(&DHT22_Structure);

        wiced_init_timer(&Measurement_Timer, &IRQ_Measurement, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER);
        wiced_start_timer(&Measurement_Timer, CO2_IRQ_MILLISECONDS_TIME);

        /* Initialize Bluetooth Controller and Host Stack
        wiced_bt_stack_init(ble_co2_sensnet_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);

        if (wiced_hal_read_nvram(SPP_NVRAM_ID+5, sizeof(wiced_bt_device_address_t), bda, &result) != sizeof(wiced_bt_device_address_t)) {
            unsigned char MAC_Buffer[6];
            for( uint idx = 0; idx < sizeof(MAC_Buffer)/sizeof(MAC_Buffer[0]); ++idx ) {
                MAC_Buffer[idx]  = HEX_To_Digit( BLE_ADDRESS_SETTING[     3 * idx ] ) << 4;
                MAC_Buffer[idx] |= HEX_To_Digit( BLE_ADDRESS_SETTING[ 1 + 3 * idx ] );
            }

            bda[0] = MAC_Buffer[0];          /* Valid static random address should have 2 most significant bits set to 1
            bda[1] = MAC_Buffer[1];
            bda[2] = MAC_Buffer[2];
            bda[3] = MAC_Buffer[3];
            bda[4] = MAC_Buffer[4];
            bda[5] = MAC_Buffer[5];

            wiced_hal_write_nvram(SPP_NVRAM_ID+5, sizeof(wiced_bt_device_address_t), bda, &result);
        }

        wiced_bt_set_local_bdaddr(bda , BLE_ADDR_PUBLIC);
    }
 */

/*******************************************************************************************************************************
 * void IRQ_Measurement (uint32_t arg)
 ******************************************************************************************************************************/
/*  void IRQ_Measurement (uint32_t arg) {
        Current_State = STATE_ACTIVE;
        wiced_hal_gpio_set_pin_output(LED_BLUE, ~wiced_hal_gpio_get_pin_output(LED_BLUE));

        int16_t CO2_Value = 0;
        // float DHT22_Temperature, DHT22_Humidity = 0;
        float SHTC3_Temperature, SHTC3_Temperature_Prec, SHTC3_Humidity, SHTC3_Humidity_Prec = 0;

        BLE_ReadDPS368(&DPS368_Structure);
        BLE_ReadPASCO2(&PASCO2_Structure, &CO2_Value, DPS368_Structure.pressure);
        BLE_ReadSHTC3(&SHTC3_Structure, &SHTC3_Temperature, &SHTC3_Temperature_Prec, &SHTC3_Humidity, &SHTC3_Humidity_Prec);
        // BLE_ReadDHT22(&DHT22_Structure, &DHT22_Temperature, &DHT22_Humidity);

        ble_co2_sensnet_xensiv_measurement_co2_data[0] = CO2_Value & 0xFF;       // Low Byte
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

        wiced_bt_gatt_send_notification(connection_id, HDLC_XENSIV_MEASUREMENT_CO2_DATA_VALUE, sizeof(ble_co2_sensnet_xensiv_measurement_co2_data), ble_co2_sensnet_xensiv_measurement_co2_data );
        wiced_bt_gatt_send_notification(connection_id, HDLC_XENSIV_MEASUREMENT_TEMP_DATA_VALUE, sizeof(ble_co2_sensnet_xensiv_measurement_temp_data), ble_co2_sensnet_xensiv_measurement_temp_data );
        wiced_bt_gatt_send_notification(connection_id, HDLC_XENSIV_MEASUREMENT_PRESS_DATA_VALUE, sizeof(ble_co2_sensnet_xensiv_measurement_press_data), ble_co2_sensnet_xensiv_measurement_press_data );
        wiced_bt_gatt_send_notification(connection_id, HDLC_XENSIV_MEASUREMENT_HUM_DATA_VALUE, sizeof(ble_co2_sensnet_xensiv_measurement_hum_data), ble_co2_sensnet_xensiv_measurement_hum_data );

        if (CO2_Value != 0) {
            if (CO2_Value <= CO2_MED_THRESHOLD) {
                wiced_hal_gpio_set_pin_output(LED_GREEN, 0);        // Turn ON Green Wing Board LED
                wiced_hal_gpio_set_pin_output(LED_ORANGE, 1);       // Turn OFF Orange Wing Board LED
                wiced_hal_gpio_set_pin_output(LED_RED, 1);          // Turn OFF Red Wing Board LED
            } else if(CO2_Value <= CO2_HIGH_THRESHOLD) {
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
 */

/*******************************************************************************************************************************
 * GATT Connection Status Callback
 ******************************************************************************************************************************/
 /* GATT Connection Status Callback
wiced_bt_gatt_status_t ble_co2_sensnet_connect_callback( wiced_bt_gatt_connection_status_t *p_conn_status )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

          if ( NULL != p_conn_status )
          {
              if ( p_conn_status->connected )
              {
                  // Device has connected
                  connection_id = p_conn_status->conn_id;
                  wiced_hal_gpio_set_pin_output(LED_BLUE, 0);
              }
              else
              {
                  // Device has disconnected
                  connection_id = 0;
                  wiced_hal_gpio_set_pin_output(LED_BLUE, 1);

                  /* restart the advertisements
                  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
              }
              status = WICED_BT_GATT_SUCCESS;
          }

          return status;
}
 */


/*******************************************************************
 * Additional Functions
 ******************************************************************/
unsigned char HEX_To_Digit( char ch ) {
    if (( '0' <= ch ) && ( ch <= '9' ))
        ch -= '0';
    else {
        if (( 'a' <= ch ) && ( ch <= 'f' ))
            ch += 10 - 'a';
        else {
            if (( 'A' <= ch ) && ( ch <= 'F' ) )
                ch += 10 - 'A';
            else
                ch = 16;
        }
    }
    return ch;
}

/*******************************************************************
 * PUART Functions
 ******************************************************************/
void Initialize_PUART( void ) {
    wiced_hal_puart_init();
    wiced_hal_puart_flow_off( );                // call wiced_hal_puart_flow_on() to turn on flow control
    wiced_hal_puart_enable_tx( );               // call wiced_hal_puart_disable_tx() to disable transmit capability
    wiced_hal_puart_set_baudrate( 256000 );     // Enable to change puart baud rate. eg: 9600, 19200, 38200
}

/*******************************************************************
 * PAS CO2 Functions
 ******************************************************************/
void BLE_InitPASCO2(PASCO2_t* co2, uint16_t co2_address, uint16_t PressCompensation) {
    pasco2_init(co2, co2_address);
    wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );

    pasco2_reset(co2, PASCO2_SRTRG_SOFT_RESET);
    wiced_rtos_delay_milliseconds(500, ALLOW_THREAD_TO_SLEEP );

    if (PressCompensation) {
        pasco2_setPressureCompensation(co2, (int) (PressCompensation / 100));
        wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );
    }
}

/*******************************************************************
 *
 ******************************************************************/
void BLE_ReadPASCO2(PASCO2_t* co2, int16_t *CO2_Value, uint16_t PressCompensation) {
    #ifdef DEBUG_MODE
        char Buffer[255] = {0};
        snprintf(Buffer, sizeof(Buffer), "PAS CO2 Errors: %i | %i | %i\r\n",    (int) pasco2_isCommunicationError(co2),
                                                                                (int) pasco2_isTemperatureOutOfRange(co2),
                                                                                (int) pasco2_isVdd12OutOfRange(co2));
        wiced_hal_puart_print(Buffer);
        wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );

        pasco2_clearTemperatureOutOfRange(co2);
        wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );

        pasco2_clearVdd12OutOfRange(co2);
        wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );

        pasco2_clearCommunicationError(co2);
        wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );
    #endif

    if (pasco2_isDataReady(co2))
        *CO2_Value = pasco2_getCo2Concentration(co2);
    else
        *CO2_Value = 0;

    #ifdef DEBUG_MODE
        snprintf(Buffer, sizeof(Buffer), "PAS CO2: CO2 Concentration: %i ppm\r\n", (int16_t) *CO2_Value);
        wiced_hal_puart_print(Buffer);
    #endif

    /* Update Pressure Compensation for XENSIV PAS CO2 */
    if (PressCompensation)
        pasco2_setPressureCompensation(co2, (int) (PressCompensation / 100));

    /* Launch Single CO2 Measurement */
    pasco2_setOperationMode(co2, PASCO2_OP_MODE_SINGLE);
}

/*******************************************************************
 * DPS368 Functions
 ******************************************************************/
void BLE_InitDPS368(DPS368_t* dps368, uint16_t dps368_address) {
    DPS368_init((DPS368_t*) dps368, dps368_address);
    wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );
}

/*******************************************************************
 *
 ******************************************************************/
DPS368_meas_state_t BLE_ReadDPS368(DPS368_t* dps368) {
    DPS368_meas_state_t Status = DPS368_data_ready(dps368);

    #ifdef DEBUG_MODE
        char Buffer[255] = {0};
        float Temp_Precision = dps368->temperature - (int) dps368->temperature;
        Temp_Precision *= 1000;

        snprintf(Buffer, sizeof(Buffer), "DPS368: Temp: %.2i.%.3i C       |       Press: %i\r\n", (int) dps368->temperature, (int) Temp_Precision, (int) dps368->pressure);
        wiced_hal_puart_print(Buffer);
    #endif

    return Status;
}

/*******************************************************************
 * DHT22 Functions
 ******************************************************************/
void BLE_InitDHT22(DHT* dht22) {
    DHTbegin(dht22);    // Initialize DHT22 Pin
    wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );
}

/*******************************************************************
 *
 ******************************************************************/
void BLE_ReadDHT22(DHT* dht22, float *DHT_Temp, float *DHT_Hum) {
    if(DHTread(dht22)){      // Get Temperature and Humidity - Trigger a Sensor package and read it
        *DHT_Temp = DHTreadTemperature(dht22, 1, 0);   // Read Temperature
        *DHT_Hum = DHTreadHumidity(dht22, 1);        // Read Humidity
        wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );

        #ifdef DEBUG_MODE
            char Buffer[255] = {0};

            float DHT_Temp_Precision = *DHT_Temp - (int) *DHT_Temp;
            DHT_Temp_Precision *= 1000;
            float DHT_Hum_Precision = *DHT_Hum - (int) *DHT_Hum;
            DHT_Hum_Precision *= 1000;

            snprintf(Buffer, sizeof(Buffer), "DHT22:  Temp: %.2i.%.3i C       |       Hum: %.2i.%.3i\r\n",    (int) *DHT_Temp, (int) DHT_Temp_Precision,
                                                                                                              (int) *DHT_Hum, (int) DHT_Hum_Precision);
            wiced_hal_puart_print(Buffer);
        #endif
    } else {
        *DHT_Temp = 0;
        *DHT_Hum = 0;
    }
}

/*******************************************************************
 * SHTC3 Functions
 ******************************************************************/
void BLE_InitSHTC3(SHTC3_t* shtc3, uint16_t shtc3_address) {
    SHTC3_Init(shtc3, shtc3_address, 0);
    wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );

    SHTC3_Sleep(shtc3);
    wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );

    SHTC3_Wakeup(shtc3);
    wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );
}

/*******************************************************************
 *
 ******************************************************************/
SHTC3_state_t BLE_ReadSHTC3(SHTC3_t* shtc3, float *SHTC3_Temp, float *SHTC3_Temp_Prec, float *SHTC3_Hum, float *SHTC3_Hum_Prec) {
    SHTC3_state_t Status = SHTC3_GetTempAndHumi(shtc3, SHTC3_Temp, SHTC3_Hum);

    if (*SHTC3_Temp <= 0)
        *SHTC3_Temp = 0;

    *SHTC3_Temp_Prec = *SHTC3_Temp - (int) *SHTC3_Temp;
    *SHTC3_Temp_Prec *= 1000;
    *SHTC3_Hum_Prec = *SHTC3_Hum - (int) *SHTC3_Hum;
    *SHTC3_Hum_Prec *= 1000;

    #ifdef DEBUG_MODE
        char Buffer[255] = {0};
        snprintf(Buffer, sizeof(Buffer), "SHTC3:  Temp: %.2i.%.3i C       |       Hum: %.2i.%.3i\r\n",    (int) *SHTC3_Temp, (int) *SHTC3_Temp_Prec,
                                                                                                          (int) *SHTC3_Hum,  (int) *SHTC3_Hum_Prec);
        wiced_hal_puart_print(Buffer);
        wiced_rtos_delay_milliseconds(50, ALLOW_THREAD_TO_SLEEP );
    #endif

    return Status;
}
