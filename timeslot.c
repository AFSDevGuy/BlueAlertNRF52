#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "timeslot.h"
#include "ibeacon.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

/**Constants for timeslot API
*/

#define RADIO_DEFAULT_ADDRESS           (0x8E89BED6)
static uint32_t         m_alt_aa = RADIO_DEFAULT_ADDRESS;
#if 0
static const uint8_t g_ibeacon_pkt[] = {
  /* iBeacon packet */
    26, 0xFF, 0x4C, 0x00, 0x02, 0x15,
        0x3B, 0x0C, 0x44, 0xC6, 0x55, 0xA8, 0xF9, 0x55,
        0x32, 0xEB, 0x6A, 0xB2, 0x65, 0x42, 0xFE, 0x1A,
        0x00, 0x00,
        0x00, 0x00,
        0xC5
};
#endif

#define APP_BEACON_INFO_LENGTH          0x17                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                        /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                        /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH+1] =                    /**< Information advertised by the Beacon. */
{
    APP_BEACON_INFO_LENGTH,
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};

#if 0
typedef struct {
  uint32_t channel;
  uint32_t frequency;
} TMapping ;

static const TMapping g_advertisment[] = {
  {37,  2},
  {38, 26},
  {39, 80},
};
#define ADVERTISMENT_CHANNELS ((int)(sizeof(g_advertisment)/sizeof(g_advertisment[0])))
#endif

static nrf_radio_request_t  m_timeslot_request;
static uint32_t             m_slot_length;

static nrf_radio_signal_callback_return_param_t signal_callback_return_param;

static bool                 m_is_in_callback            = false; /** Code is in callback-context. */
static bool                 m_is_in_timeslot            = false; /** A timeslot is currently in progress. */
static bool                 m_end_timer_triggered       = false; /** The timeslot end timer has been triggered, and the timeslot is about to end. */


void send_packet() {
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
  NRF_RADIO->PACKETPTR = (uint32_t) m_beacon_info;
  NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
  NRF_RADIO->PREFIX0 |= (((m_alt_aa >> 24) << 8) & 0x0000FF00);
  NRF_RADIO->BASE1    = ((m_alt_aa <<  8) & 0xFFFFFF00);

      //NRF_RADIO->TXADDRESS = p_evt->access_address;
      NRF_RADIO->TXPOWER  = RADIO_TXPOWER_TXPOWER_Pos4dBm; // 0dbm

      // send the packet:
      NRF_RADIO->EVENTS_READY = 0U;
      NRF_RADIO->TASKS_TXEN   = 1;

      while (NRF_RADIO->EVENTS_READY == 0U)
      {
          // wait
      }
      NRF_RADIO->EVENTS_END  = 0U;
      NRF_RADIO->TASKS_START = 1U;

      while (NRF_RADIO->EVENTS_END == 0U)
      {
          // wait
      }

      uint32_t err_code = bsp_indication_set(BSP_INDICATE_SENT_OK);
      NRF_LOG_INFO("The packet was sent");
      APP_ERROR_CHECK(err_code);

      NRF_RADIO->EVENTS_DISABLED = 0U;
      // Disable radio
      NRF_RADIO->TASKS_DISABLE = 1U;

      while (NRF_RADIO->EVENTS_DISABLED == 0U)
      {
          // wait
      }
}


void radio_init()
{
    /* Reset all states in the radio peripheral */
    NRF_RADIO->POWER            = ((RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk);
    NRF_RADIO->POWER            = ((RADIO_POWER_POWER_Enabled  << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk);

    /* Set radio configuration parameters */
    NRF_RADIO->TXPOWER      = ((RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos) & RADIO_TXPOWER_TXPOWER_Msk);
    NRF_RADIO->MODE       = ((RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos) & RADIO_MODE_MODE_Msk);

    NRF_RADIO->FREQUENCY      = 2;          // Frequency bin 2, 2402MHz, channel 37.
    NRF_RADIO->DATAWHITEIV      = 37;         // NOTE: This value needs to correspond to the frequency being used


    /* Configure Access Address  */
    NRF_RADIO->PREFIX0  = ((RADIO_DEFAULT_ADDRESS >> 24) & 0x000000FF);
    NRF_RADIO->BASE0    = ((RADIO_DEFAULT_ADDRESS <<  8) & 0xFFFFFF00);
    NRF_RADIO->PREFIX0 |= (((m_alt_aa >> 24) << 8) & 0x0000FF00);
    NRF_RADIO->BASE1    = ((m_alt_aa <<  8) & 0xFFFFFF00);
    NRF_RADIO->TXADDRESS    = 0x00;         // Use logical address 0 (prefix0 + base0) = 0x8E89BED6 when transmitting
    NRF_RADIO->RXADDRESSES  = 0x01;       // Enable reception on logical address 0 (PREFIX0 + BASE0)

    /* PCNF-> Packet Configuration. Now we need to configure the sizes S0, S1 and length field to match the datapacket format of the advertisement packets. */
    NRF_RADIO->PCNF0 =  (
                          (((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk)    // length of S0 field in bytes 0-1.
                        | (((2UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk)    // length of S1 field in bits 0-8.
                        | (((6UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)    // length of length field in bits 0-8.
                      );

    /* Packet configuration */
    NRF_RADIO->PCNF1 =  (
                          (((37UL)                          << RADIO_PCNF1_MAXLEN_Pos)  & RADIO_PCNF1_MAXLEN_Msk)   // maximum length of payload in bytes [0-255]
                        | (((0UL)                           << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk)  // expand the payload with N bytes in addition to LENGTH [0-255]
                        | (((3UL)                           << RADIO_PCNF1_BALEN_Pos)   & RADIO_PCNF1_BALEN_Msk)    // base address length in number of bytes.
                        | (((RADIO_PCNF1_ENDIAN_Little)     << RADIO_PCNF1_ENDIAN_Pos)  & RADIO_PCNF1_ENDIAN_Msk)   // endianess of the S0, LENGTH, S1 and PAYLOAD fields.
                        | (((RADIO_PCNF1_WHITEEN_Enabled)   << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)  // enable packet whitening
                      );

    /* CRC config */
    NRF_RADIO->CRCPOLY = ((0x00065B << RADIO_CRCPOLY_CRCPOLY_Pos) & RADIO_CRCPOLY_CRCPOLY_Msk);    // CRC polynomial function
    NRF_RADIO->CRCCNF = (((RADIO_CRCCNF_SKIPADDR_Skip) << RADIO_CRCCNF_SKIPADDR_Pos) & RADIO_CRCCNF_SKIPADDR_Msk)
                      | (((RADIO_CRCCNF_LEN_Three)      << RADIO_CRCCNF_LEN_Pos)       & RADIO_CRCCNF_LEN_Msk);

    NRF_RADIO->CRCINIT = ((0x555555 << RADIO_CRCINIT_CRCINIT_Pos) & RADIO_CRCINIT_CRCINIT_Msk);    // Initial value of CRC
    /* Lock interframe spacing, so that the radio won't send too soon / start RX too early */
    NRF_RADIO->TIFS = 148;

    NRF_RADIO->EVENTS_END = 0;

    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

}
static void timeslot_end(void)
{
 //   radio_disable();
 //   timer_on_ts_end(timeslot_end_time_get());
    m_is_in_timeslot = false;
    m_is_in_callback = false;
    m_end_timer_triggered = false;

#ifdef NRF52
    NRF_TIMER0->TASKS_STOP = 0;
    NRF_TIMER0->TASKS_SHUTDOWN = 1;
    for (uint32_t i = 0; i < 6; i++)
    {
        NRF_TIMER0->EVENTS_COMPARE[i] = 0;
        NRF_TIMER0->CC[i] = 0;
    }
    NRF_TIMER0->INTENCLR = 0xFFFFFFFF;
    NVIC_ClearPendingIRQ(TIMER0_IRQn);
#endif
}

uint32_t timeslot_init(nrf_clock_lf_cfg_t lfclksrc)
{

    m_is_in_callback = false;
    return NRF_SUCCESS;
}

/**@brief Request next timeslot event in earliest configuration
 */
uint32_t request_next_event_earliest(void)
{
    m_slot_length                                  = 15000;
    m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us   = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us  = 1000000;
    return sd_radio_request(&m_timeslot_request);
}


/**@brief Configure next timeslot event in earliest configuration
 */
void configure_next_event_earliest(void)
{
    m_slot_length                                  = 15000;
    m_timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_NORMAL;
    m_timeslot_request.params.earliest.length_us   = m_slot_length;
    m_timeslot_request.params.earliest.timeout_us  = 1000000;
}


/**@brief Configure next timeslot event in normal configuration
 */
void configure_next_event_normal(void)
{
    m_slot_length                                 = 15000;
    m_timeslot_request.request_type               = NRF_RADIO_REQ_TYPE_NORMAL;
    m_timeslot_request.params.normal.hfclk        = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_timeslot_request.params.normal.priority     = NRF_RADIO_PRIORITY_HIGH;
    m_timeslot_request.params.normal.distance_us  = 1000000;
    m_timeslot_request.params.normal.length_us    = m_slot_length;
}


/**@brief Timeslot signal handler
 */
void nrf_evt_signal_handler(uint32_t evt_id, void *p_context)
{
    uint32_t err_code;
    
    switch (evt_id)
    {
        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            //No implementation needed
            break;
        case NRF_EVT_RADIO_SESSION_IDLE:
            //No implementation needed
            break;
        case NRF_EVT_RADIO_SESSION_CLOSED:
            //No implementation needed, session ended
            break;
        case NRF_EVT_RADIO_BLOCKED:
            //Fall through
        case NRF_EVT_RADIO_CANCELED:
            err_code = request_next_event_earliest();
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}


/**@brief Timeslot event handler
 */
nrf_radio_signal_callback_return_param_t * radio_callback(uint8_t signal_type)
{
    switch(signal_type)
    {
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
            //Start of the timeslot - set up timer interrupt
            signal_callback_return_param.params.request.p_next = NULL;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
            
            NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
            NRF_TIMER0->CC[0] = m_slot_length - 1000;
            NVIC_EnableIRQ(TIMER0_IRQn);   
            
            nrf_gpio_pin_toggle(20); //Toggle LED4
            radio_init();
            send_packet();
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
            signal_callback_return_param.params.request.p_next = NULL;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
            //send_packet();
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
            //Timer interrupt - do graceful shutdown - schedule next timeslot
            timeslot_end();
            configure_next_event_normal();
            signal_callback_return_param.params.request.p_next = &m_timeslot_request;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
            break;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
            //No implementation needed
            break;
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
            //Try scheduling a new timeslot
            configure_next_event_earliest();
            signal_callback_return_param.params.request.p_next = &m_timeslot_request;
            signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
            break;
        default:
            //No implementation needed
            break;
    }
    return (&signal_callback_return_param);
}



/**@brief Function for initializing the timeslot API.
 */
uint32_t timeslot_sd_init(void)
{
    uint32_t err_code;
    
    err_code = sd_radio_session_open(radio_callback);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = request_next_event_earliest();
    if (err_code != NRF_SUCCESS)
    {
        (void)sd_radio_session_close();
        return err_code;
    }
    NRF_SDH_SOC_OBSERVER(m_nrf_soc_observer, 0, nrf_evt_signal_handler, NULL);
    return NRF_SUCCESS;
}

