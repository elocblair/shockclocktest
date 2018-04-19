#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "our_service.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"
#include "nrf_drv_gpiote.h"



#define CS_ADXL2_PIN  8 /**< SPI CS Pin.*/
#define CS_ADXL1_PIN  17
#define CS_ADXL1_PIN_EVAL 3
#define CS_ADXL2_PIN_EVAL 8
#define CS_FRAM       4
#define CS_FRAM_EVAL  7


#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. 
																							if not enabled, the server's database cannot be
																								changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT               0                                          /**<number of central links used by the application. 
																							When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**<number of peripheral links used by the application. 
																							When changing this number remember to adjust the RAM settings*/

//#define DEVICE_NAME                      "OurCharacteristic"                      /**< Name of device. Will be included in the advertising data. */
#define DEVICE_NAME                      "JohnCougarMellenc"
#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval 
																							(in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       300                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(16, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(40, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification)
																							to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call 
																							(30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump,
 																							can be used to identify stack location on stack unwind. */
#define ADXL1  0
#define FRAM   1
#define ADXL2  2
#define SDCARD 3

static dm_application_instance_t        m_app_handle;                               /**< Application identifier allocated by device manager */

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
double vector;
uint8_t mainADXLid = 0;
uint8_t antADXLid = 0;
/****************************
*
* additional global variables	
*
****************************/	
uint8_t testANT;
uint8_t cs_flag = 0;  
// FRAM register variables 
int FRAMregister = 0;
int FRAMreadRegister = 0;
int registerAtStop = 0;
uint8_t FRAMfull = 0;
static volatile bool spi_xfer_done; 

// spi read from adxl buffers
uint8_t buffTx[1] = {0xF2}; // reading static values from the sensor NOT acc values
uint8_t* addrBuffTx = &buffTx[0];
uint8_t i = 0;
uint8_t writeFlag = 0;
double greatestVal = 0;
double greatestVal2 = 0;
uint8_t greatestSample[20];
int isInt = 0;
static uint8_t ADXLdataBuffer[196] = {0};
static uint8_t ADXLdataBuffer2[196] = {0};
nrf_drv_gpiote_pin_t main_bosch_address_pin = 2;
nrf_drv_gpiote_pin_t green_led = 15;
nrf_drv_gpiote_pin_t yellow_led = 20;
nrf_drv_gpiote_pin_t red_led = 19;
nrf_drv_gpiote_pin_t boschIntEval = 6;
nrf_drv_gpiote_pin_t boschInt = 3;// spi configuration structure


bool blockUpdate = false;

// service structure for our application
ble_os_t m_our_service;

// app_timer id variable and timer interval
APP_TIMER_DEF(m_our_char_timer_id);
APP_TIMER_DEF(data_dump_id);
#define OUR_CHAR_TIMER_INTERVAL     APP_TIMER_TICKS(10, APP_TIMER_PRESCALER) /* 10 ms intervals, 
																			      2 packets per connection interval*/
#define DATA_DUMP_TIMER_INTERVAL     APP_TIMER_TICKS(25, APP_TIMER_PRESCALER)

static ble_uuid_t       m_adv_uuids[] = {{BLE_UUID_OUR_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}}; 
 
bool init_i2c(void);
void initBosch();
uint8_t* readEuler();
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/****************************
*
*   spi event handler   
*		 called after contents of spi buffer have been transferred
*		 sets chip select pin high 
*		 allows the application to continue past __WFE(); 
*												   wait for event
*
****************************/

/****************************
*
*  bosch i2c gyroscope read
*
****************************/
uint8_t* readGyro(){
	uint8_t* dataPtr;
	if(!blockUpdate){
		dataPtr = readEuler();
	}	
	return dataPtr;
}	

	
void int_fxn(){

	/*uint8_t boschIntCheck[2];
	boschIntCheck[0] = 0x37;
	nrf_drv_twi_tx(&m_twi_master,BOSCH_ADDRESS,&boschIntCheck[0],1,false);
	nrf_drv_twi_rx(&m_twi_master,BOSCH_ADDRESS,&boschIntCheck[1],1);
	SEGGER_RTT_printf(0,"any or no motion = %d\n",boschIntCheck[1]);
	uint8_t boschIntReset[2] = {0x3F,0x40};
	nrf_drv_twi_tx(&m_twi_master, BOSCH_ADDRESS, &boschIntReset[0],2,false);	
	if(boschIntCheck[1] >= 128){
		blockUpdate = true;
		nrf_drv_gpiote_out_set(15); //green off 
		SEGGER_RTT_WriteString(0, "no motion\n");
		boschIntCheck[0] = 0x3E;
		boschIntCheck[1] = 0x01;
		nrf_drv_twi_tx(&m_twi_master, BOSCH_ADDRESS, &boschIntCheck[0],2,false);
		SEGGER_RTT_printf(0,"should be 01 %d\n",boschIntCheck[1]);
	}
	else if(boschIntCheck[1] >= 64 && boschIntCheck[1] < 128){
		blockUpdate = false;
		nrf_drv_gpiote_out_clear(15); //green on
		SEGGER_RTT_WriteString(0,"any motion\n");
		boschIntCheck[0] = 0x3E;
		boschIntCheck[1] = 0x00;
		nrf_drv_twi_tx(&m_twi_master, BOSCH_ADDRESS, &boschIntCheck[0],2,false);
		nrf_drv_twi_tx(&m_twi_master, BOSCH_ADDRESS, &boschIntCheck[0],1,false);
		nrf_drv_twi_rx(&m_twi_master, BOSCH_ADDRESS, &boschIntCheck[0],1);
		SEGGER_RTT_printf(0,"should be 00 %d\n",boschIntCheck[0]);
		ble_advertising_start(BLE_ADV_MODE_FAST);
	}*/
}
/****************************
*
*   timeout event   
*		 read entire buffer on adxl (32 samples)
*		 write buffer to FRAM
*		 read FRAM (check)
*		 reenable writes to FRAM
*		 read gyro bno055
*		 update characteristic for notify
*
****************************/
static void timer_timeout_handler(void * p_context)
{
	if(!blockUpdate){
		uint8_t* gyroData = readGyro();
		our_termperature_characteristic_update(&m_our_service, gyroData);
	}
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    //Initiate timer
	app_timer_create(&m_our_char_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    // initialize the services used by the application.
    our_service_init(&m_our_service);    
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
*/
static void application_timers_start(void)
{
	app_timer_start(m_our_char_timer_id, OUR_CHAR_TIMER_INTERVAL, NULL);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    //blockUpdate = true;
	SEGGER_RTT_WriteString(0,"NRF_INVALID_STATE\n");
	uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    // Call ble_our_service_on_ble_evt() to do housekeeping of ble connections related to our service and characteristic
	ble_our_service_on_ble_evt(&m_our_service, p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into ble_advertising_init().
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    // Create a scan response packet and include the list of UUIDs 

    ble_advdata_t srdata;
    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    srdata.uuids_complete.p_uuids = m_adv_uuids;

    err_code = ble_advertising_init(&advdata, &srdata, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}






void initializePin(nrf_drv_gpiote_out_config_t pinConfig, nrf_drv_gpiote_pin_t pin, bool pinStatus){
	ret_code_t err_code;
	err_code = nrf_drv_gpiote_out_init(pin,&pinConfig);
	if(err_code != 0){
		SEGGER_RTT_printf(0, "error initializing pin %d\n", pin);
	}
	if(pinStatus){
		nrf_drv_gpiote_out_set(pin);
	}
	else{
		nrf_drv_gpiote_out_clear(pin);
	}
}

int main(void)
{   
	/****************************
	*
	*  initialize gpiote driver
	*		init spi chip select pins
	*
	****************************/
	
	ret_code_t err_code;
	if(!nrf_drv_gpiote_is_init())
	{
		err_code = nrf_drv_gpiote_init();
	}
	if (err_code == 8){
		SEGGER_RTT_WriteString(0,"NRF_INVALID_STATE\n");
	}
 
	nrf_drv_gpiote_out_config_t outputGPIO = {
		NRF_GPIOTE_POLARITY_HITOLO,
		NRF_GPIOTE_INITIAL_VALUE_HIGH,
		false,
	};
	initializePin(outputGPIO,CS_ADXL2_PIN_EVAL, true);

	initializePin(outputGPIO,CS_ADXL1_PIN_EVAL, true);

	initializePin(outputGPIO,CS_FRAM_EVAL,true);

	initializePin(outputGPIO,main_bosch_address_pin,true);

	initializePin(outputGPIO,green_led,false);

	initializePin(outputGPIO,red_led,true);

	initializePin(outputGPIO,yellow_led,true);

	nrf_drv_gpiote_in_config_t interrupt = {
		NRF_GPIOTE_POLARITY_LOTOHI,
		NRF_GPIO_PIN_PULLDOWN,
		true,
		true,
	};
	
	err_code = nrf_drv_gpiote_in_init(boschIntEval, &interrupt, int_fxn);
	if(err_code != 0){
		SEGGER_RTT_WriteString(0,"interrupt pin not initialized\n");
	}
	nrf_drv_gpiote_in_event_enable(boschIntEval, true);
	
	
	/****************************
	*
	* spi init 
	*
	****************************/
    

/***********************************************************************************
*
*  Initialize i2c (twi)
*		bosch bno055
*
***********************************************************************************/

	bool i2cInitialized = init_i2c();
	if (!i2cInitialized){
		SEGGER_RTT_WriteString(0, "i2c not initialized");
	}
	initBosch();
    bool erase_bonds;

    // Initialize.
    timers_init();
    //buttons_leds_init(&erase_bonds);
    ble_stack_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;;)
    {
        power_manage();
    }
}

