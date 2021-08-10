/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "em_common.h"
#include "sl_app_log.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "sl_simple_led_instances.h"
#include "sl_simple_button_instances.h"
#include "nvm3.h"
#include "nvm3_hal_flash.h"
#include "spidrv.h"
#include "sl_spidrv_instances.h"


#define SPI_HANDLE    sl_spidrv_exp_handle
static volatile bool transfer_complete = true;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

extern nvm3_Handle_t *nvm3_defaultHandle;
static uint8_t customAdv[] = { 0x02, 0x01, 0x06, // Len=2 Status Value(0x06)
    0x05, 0xFF, //Length(5), Manufacturer-Specific Data (type 0xFF)
    0xFF, 0x02, // SiLab Manufactures Code
    0x00, 0xFF, // Counter button press
    0x03, 0x08, 0x55, 0x32}; // Tag Name U2


#define QUUPPA_TAG_ID		0x66, 0x55, 0x44, 0x33, 0x22, 0x11
#define QUUPPA_DF_PATTERN	0x67, 0xf7, 0xdb, 0x34, 0xc4, 0x03, 0x8e, 0x5c, 0x0b, 0xaa, 0x97, 0x30, 0x56, 0xe6

// Quuppa Direction Finding (DF) Packet
static uint8_t quuppaDFPacket[] = {
  0x02,           // AD Len=2           0
  0x01,           // AD Type Flags      1
  0x02,           // Flag Data          2
  0x1b,           // AD data Length     3
  0xff,           // AD Type            4
  0xc7,           // Company ID LSB     5
  0x00,           // Company ID MSB     6
  0x01,           // Quuppa Packet ID   7
  0x21,           // Quuppa Device Type 8
  0x19,           // Header, Carries information of Tag’s TX rate, TX power and ID type     9
  QUUPPA_TAG_ID,  // Quuppa Tag ID      10 11 12 13 14 15
  0x38,           // CRC-8              16
  QUUPPA_DF_PATTERN // Quuppa DF pattern
};

#define QUUPPA_CRC_INDEX                16
#define QUUPPA_CRC_START_INDEX          8
#define QUUPPA_CRC_END_INDEX            16
#define QUUPPA_DATA_START_INDEX         19

// Quuppa Data Packet
static uint8_t quuppaDataPacket[] = {
    0x02,         // AD Len=2           0
    0x01,         // AD Type Flags      1
    0x02,         // Flag Data          2
    0x1b,         // AD data Length     3
    0xff,         // AD Type            4
    0xc7,         // Company ID LSB     5
    0x00,         // Company ID MSB     6
    0xF0,         // Quuppa Packet ID   7
    0x19,         // Header, Carries information of Tag’s TX rate, TX power and ID type     8
    QUUPPA_TAG_ID,// Quuppa Tag ID      9 10 11 12 13 14
    0xFF,         // Developer specific data    15
    0x01,         // Developer ID (LSB)         16
    0x01,         // Developer ID (MSB)         17
    0x00, 0x01, 0x02, 0x03, // Payload          18 ... 
    0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B,
    0x0C
  };

// Uncomment this to build Tag with Quuppa advertisement
// #define QUUPPA_MODE

// Number of active connections.
static uint8_t connection_count = 0;

uint8_t g_lab4Connection;


// These variables need to be added and are used for the connection handle
// and the notification parameters to send Button data to the mobile app
uint8_t notification_data[1] = {0};
uint16_t notification_len = 0;
uint8_t notifyEnabled = false;

extern sl_simple_button_context_t simple_btn0_context;


#define SCAN_INTERVAL                 1600   // * 0.625ms = 1 sec
#define SCAN_WINDOW                   1600   // * 0.625ms = 1 sec
#define SCAN_PASSIVE                  0
#define SCAN_ACTIVE                   1

#define CONN_INTERVAL_MIN             6    // * 1.25 ms = 7.5 ms
#define CONN_INTERVAL_MAX             6    // * 1.25 ms = 7.5 ms
#define CONN_SLAVE_LATENCY            0    //no latency
#define CONN_TIMEOUT                  100  //1000ms
#define CONN_MIN_CE_LENGTH            0
#define CONN_MAX_CE_LENGTH            0xffff


void sl_button_on_change(const sl_button_t *handle)
{
  sl_simple_button_context_t *ctxt = ((sl_simple_button_context_t *)handle[0].context);
  if (ctxt->state) {
      ctxt->history += 1;
      sl_bt_external_signal(1);
  }

  sl_app_log("BTN %d\n",ctxt->state);

}

void print_data(uint8array *arr)
{
  for(int i = 0; i < arr->len; i++)
      app_log("%02x", arr->data[i]);
  app_log("\n");
}


/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/

// Callback fired when data is transmitted
void transfer_callback(SPIDRV_HandleData_t *handle,
                       Ecode_t transfer_status,
                       int items_transferred)
{
  (void)&handle;
  (void)items_transferred;

  // Post semaphore to signal to application
  // task that transfer is successful
  if (transfer_status == ECODE_EMDRV_SPIDRV_OK) {
    transfer_complete = true;
  }
}


/**************************************************************************//**
 * Quuppa CRC-8
 *****************************************************************************/
uint8_t crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    uint8_t i, j;
    uint8_t bitMask = 1 << 7;
    for (i = 0; i < len; i++) 
    {
      crc ^= data[i];
      for (j = 0; j < 8; j++) 
      {
        if ((crc & bitMask) != 0)
          crc = (uint8_t)((crc << 1) ^ 0x97);
        else
          crc <<= 1;
      }
    }
    crc = crc ^ 0x00;
    return crc;
}

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
//  Ecode_t ecode;
  uint32_t numberOfObjects;
  unsigned char data1[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
  unsigned char data2[] = { 11, 12, 13, 14, 15, 17, 18, 19, 20 };
  uint32_t objectType;
  size_t dataLen1;
  Ecode_t status;

  status = nvm3_open(nvm3_defaultHandle, nvm3_defaultInit);
  numberOfObjects = nvm3_countObjects(nvm3_defaultHandle);
  if (numberOfObjects < 2)
  {
    nvm3_writeData(nvm3_defaultHandle, 10, data1, sizeof(data1));
    nvm3_writeData(nvm3_defaultHandle, 11, data2, sizeof(data2));
  }
  nvm3_getObjectInfo(nvm3_defaultHandle, 11, &objectType, &dataLen1);
  if (objectType == NVM3_OBJECTTYPE_DATA)
  {
    nvm3_readData(nvm3_defaultHandle, 11, data1, dataLen1);
  }
  sl_app_log("Status %02X  %4x\n", status, numberOfObjects);

}

static uint32_t timer = 0;
static uint32_t time_stop_scanner = 0;
static uint32_t time_advertise = 0;






/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
  sl_status_t sc;
  Ecode_t ecode;
  static unsigned char data1[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
  static unsigned char data2[] = { 11, 12, 13, 14, 15, 17, 18, 19, 20 };
  sl_simple_button_poll_step(&simple_btn0_context);
  if ( simple_btn0_context.history )
  {
      if ( transfer_complete )
      {
          ecode = SPIDRV_MTransfer(SPI_HANDLE, data1, data2, 10, transfer_callback);
          EFM_ASSERT(ecode == ECODE_OK);
      }
//      sc = sl_bt_advertiser_start(
//        advertising_set_handle,
//        advertiser_general_discoverable,
//        advertiser_connectable_scannable);
//      sl_app_assert(sc == SL_STATUS_OK,
//                    "[E: 0x%04x] Failed to start advertising\n",
//                    (int)sc);

#ifdef QUUPPA_MODE
      static uint8_t counter = 0;
      // Update pauload section of the Quuppa Data Packet
      quuppaDataPacket[QUUPPA_DATA_START_INDEX] = counter++;

      // Send Quuppa Data packet
      sc = sl_bt_advertiser_set_data(advertising_set_handle,
                                       0,
                                       sizeof(quuppaDataPacket),
                                       quuppaDataPacket);
#else
      sc = sl_bt_advertiser_set_data(advertising_set_handle,
                                       0,
                                       sizeof(customAdv),
                                       customAdv);
#endif
      if ( sc == SL_STATUS_OK )
      {
          customAdv[5]++;
      }
      simple_btn0_context.history = 0;
  }

/*

  timer++;

  if ((time_stop_scanner > 0) && (time_stop_scanner <= timer))
  {
      time_stop_scanner = 0;
      sc = sl_bt_scanner_stop();
      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to stop scanner\n",
                 (int)sc);
      time_advertise = timer + 10;
      sl_app_log("Scanner Stopped\n");
  }


  if ((time_advertise > 0) && (time_advertise <= timer))
  {
      time_advertise = 0;
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_user_data, // advertiser_general_discoverable,
        advertiser_connectable_non_scannable); //advertiser_connectable_scannable
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc);
      sl_app_log("Advertising\n");
  }
*/

}



void start_scanner(void)
{
  sl_status_t sc;

  // Set passive scanning on 1Mb PHY
  sc = sl_bt_scanner_set_mode(gap_1m_phy, SCAN_PASSIVE);

  app_assert(sc == SL_STATUS_OK,
             "[E: 0x%04x] Failed to set scanner mode\n",
             (int)sc);

  // Set scan interval and scan window
  sc = sl_bt_scanner_set_timing(gap_1m_phy, SCAN_INTERVAL, SCAN_WINDOW);
  app_assert(sc == SL_STATUS_OK,
             "[E: 0x%04x] Failed to set scanner timing\n",
             (int)sc);


  // Set the default connection parameters for subsequent connections
  sc = sl_bt_connection_set_default_parameters(CONN_INTERVAL_MIN,
                                               CONN_INTERVAL_MAX,
                                               CONN_SLAVE_LATENCY,
                                               CONN_TIMEOUT,
                                               CONN_MIN_CE_LENGTH,
                                               CONN_MAX_CE_LENGTH);
  app_assert(sc == SL_STATUS_OK,
             "[E: 0x%04x] Failed to set parameters\n",
             (int)sc);

  // Set max MTU size
  uint16_t mtu = 250, mtu_out = 0; // 23 to 250
  sc = sl_bt_gatt_set_max_mtu(mtu, &mtu_out);
  app_assert(sc == SL_STATUS_OK,
             "[E: 0x%04x] Failed to set max MTU size\n",
             (int)sc);

  // Start scanning - looking for tags
  sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
  app_log("Start scanning\n");
  app_assert_status(sc);
}




void startADV(void)
{
  sl_status_t sc;
  sc = sl_bt_advertiser_start(
    advertising_set_handle,
    advertiser_user_data, // advertiser_general_discoverable,
    advertiser_connectable_non_scannable); //advertiser_connectable_scannable

  sl_app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to start advertising\n",
                (int)sc);
  sl_app_log("Advertising\n");

}



/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    /*
    case sl_bt_evt_advertiser_timeout_id:
      // Start scanner
      time_stop_scanner = timer + 2;
      start_scanner();
      sl_app_log("Scanner Started\n");
      break;
     */

    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Print boot message.
      sl_app_log("Bluetooth stack booted: v%d.%d.%d-b%d\n",
                 evt->data.evt_system_boot.major,
                 evt->data.evt_system_boot.minor,
                 evt->data.evt_system_boot.patch,
                 evt->data.evt_system_boot.build);

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to get Bluetooth address\n",
                    (int)sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to write attribute\n",
                    (int)sc);

      sl_app_log("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                 address_type ? "static random" : "public device",
                 address.addr[5],
                 address.addr[4],
                 address.addr[3],
                 address.addr[2],
                 address.addr[1],
                 address.addr[0]);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to create advertising set\n",
                    (int)sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        400, // min. adv. interval (milliseconds * 1.6), 1600 = 1 sec
        400, // max. adv. interval (milliseconds * 1.6), 1600 = 1 sec
        0,   // adv. duration
        0);  // max. num. adv. events
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to set advertising timing\n",
                    (int)sc);

#ifdef QUUPPA_MODE
      // Update Quuppa CRC Index
      quuppaDFPacket [QUUPPA_CRC_INDEX] = crc8(
            &quuppaDFPacket[QUUPPA_CRC_START_INDEX],
            QUUPPA_CRC_END_INDEX - QUUPPA_CRC_START_INDEX);

      // Start Quuppa DF adv
      sc = sl_bt_advertiser_set_data(advertising_set_handle,
                                     0,
                                     sizeof(quuppaDFPacket),
                                     quuppaDFPacket);
#else
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_set_data(advertising_set_handle,
                                     0,
                                     sizeof(customAdv),
                                     customAdv);
#endif
      startADV();

      break;


    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      sl_app_log("Connected: %d\n", evt->data.evt_connection_opened.connection);
      /*sl_app_log("Client address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                 evt->data.evt_connection_opened.address.addr[5],
                 evt->data.evt_connection_opened.address.addr[4],
                 evt->data.evt_connection_opened.address.addr[3],
                 evt->data.evt_connection_opened.address.addr[2],
                 evt->data.evt_connection_opened.address.addr[1],
                 evt->data.evt_connection_opened.address.addr[0]);
                 */
      connection_count++;

      // When sending notifications we need the connection handle.  Capture it here
      g_lab4Connection = evt->data.evt_connection_opened.connection;

      // Continue advertising if the stack allows further connections.
      /*
      if (connection_count < 1) { // SL_BT_CONFIG_MAX_CONNECTIONS
        sc = sl_bt_advertiser_start(
          advertising_set_handle,
          advertiser_user_data, //advertiser_general_discoverable,
          advertiser_connectable_scannable);
        sl_app_assert(sc == SL_STATUS_OK,
                      "[E: 0x%04x] Failed to start advertising\n",
                      (int)sc);
        sl_app_log("Continue advertising\n");
      }
      */
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      sl_app_log("Closed: %d, reason %d\n",
                 evt->data.evt_connection_closed.connection,
                 evt->data.evt_connection_closed.reason);

      connection_count--;

      startADV();
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    case sl_bt_evt_system_external_signal_id:
      if (notifyEnabled) {
          if (evt->data.evt_system_external_signal.extsignals == 1)  // 1 = BTN0
          {
              notification_data[0] = (uint8_t)simple_btn0_context.history;
              //simple_btn0_context.history = 0;

              // send number of button presses
              sc = sl_bt_gatt_server_send_characteristic_notification(
                      g_lab4Connection, gattdb_BTN, sizeof(notification_data),
                      notification_data, &notification_len);
          }
      }
      break;
    case  sl_bt_evt_gatt_server_characteristic_status_id:
      if ((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_BTN)
           && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01))
      {
          if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x00)
              notifyEnabled = false;
          else
              notifyEnabled = true;
      }
      break;



    case sl_bt_evt_gatt_server_user_write_request_id:
      app_log ("Write: %db, char: %d\n",
               evt->data.evt_gatt_server_attribute_value.value.len,
               evt->data.evt_gatt_server_user_write_request.characteristic);

      if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_LED) {

          sl_bt_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection,
            gattdb_LED, SL_STATUS_OK);

          app_log ("ACK sent\n", evt->data.evt_gatt_server_attribute_value.value.len);


         // Write user supplied value to LEDs.
         if (evt->data.evt_gatt_server_attribute_value.value.data[0]) {

             //This is the use of the Simple LED component
             sl_led_turn_on(&sl_led_led0);
         }
         else {
             //This is the use of the Simple LED component
             sl_led_turn_off(&sl_led_led0);
         }


         if (evt->data.evt_gatt_server_attribute_value.value.len > 1)
           {
             print_data(&evt->data.evt_gatt_server_attribute_value.value);
           }

       }
       break;


    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
