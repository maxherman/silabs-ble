/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"
//#include "em_device.h"
//#include "em_chip.h"
//#include "em_core.h"
#include "em_cmu.h"
//#include "em_emu.h"
#include "em_rtcc.h"
//#include "em_letimer.h"


#include "app_log.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "conn.h"
#include "sl_simple_timer.h"
#include "sl_simple_button.h"
#include "sl_simple_button_instances.h"

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;
stack_state_t stack_state = STACK_IDLE;
static uint8_t flip_index = 0;
static int32_t rtc_timer_freq = 32768;

uint8_t active_connections = 0;



/**************************************************************************//**
 * Application Init.
 *****************************************************************************/

SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  init_connection_list();

  app_log_time("Init done\n");
  app_log_time("@ SleepTimerFreq = %d\n", sl_sleeptimer_get_timer_frequency())

  rtc_timer_freq = sl_sleeptimer_get_timer_frequency();
}


uint32_t microtime(void)
{

  return (sl_sleeptimer_get_tick_count64()*1000) / rtc_timer_freq;
}

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

  static unsigned long print_time = 0;
  static unsigned long flip_time = 0;
  long seconds = (microtime() / 1000);

  static uint8_t flip_done = 0;

  // Print enumerated tags every second
  if (seconds >= print_time)
  {
      print_time = seconds + 1;
      //app_log_time("\n----------------------------------\n");
      //print_enumerated_tags();
  }

/*
  if ((seconds > flip_time) && (!flip_done) && (seconds > 5))
  {
      //flip_done = 1;
      flip_time = seconds + 5;

      flip();
      flip_index += 1;
  }
*/

  // Call flip state machine
  flip_process_action();

}




/*
 * Override function for button press that needs to be added.  This function is
 * called in interrupt context and uses the Bluetooth stack external signaling.
 */
void sl_button_on_change(const sl_button_t *handle)
{
  sl_simple_button_context_t *ctxt = ((sl_simple_button_context_t *)handle[0].context);

  if (ctxt->state)
  {
    app_log_time("BTN %d\n", ctxt->pin);
    if (ctxt->pin == 0)
    {
      flip();
      flip_index += 1;
    }
    else
    {
        print_enumerated_tags();
    }
  }

}



void write_data(conn_properties_t* conn)
{
  sl_status_t sc;

  uint8_t BLOCK_SIZE = 8;
  uint8_t data[BLOCK_SIZE];
  uint16_t sent = 0;

  stack_state = STACK_WRITING;

  for (int i = 0; i < BLOCK_SIZE; i++)
    data [i] = i;

  data [0] = flip_index % 2;

  conn->connection_state = TAG_WRITING;



#ifdef NEED_ACK
  app_log_time("#%d\tWrite %d bytes, handle %d\n", conn->index, BLOCK_SIZE, conn->connection_handle);
  sc = sl_bt_gatt_write_characteristic_value(
      conn->connection_handle,
      21, BLOCK_SIZE, data);

  if (sc != SL_STATUS_OK)
    app_log_time("[E: 0x%04x] Failed to write\n", (int)sc);
#else
  conn->flipped = 1;

  sc = sl_bt_gatt_write_characteristic_value_without_response(
      conn->connection_handle,
      21, BLOCK_SIZE, data, &sent);

  if (sc != SL_STATUS_OK)
    app_log_time("[E: 0x%04x] Failed to write\n", (int)sc);

  app_log_time("#%d\tUNRELIABLE write %d bytes, wrote %d, handle %d\n", conn->index, BLOCK_SIZE, sent, conn->connection_handle);

#endif


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
  conn_properties_t* conn;
  uint32_t mt = microtime();


  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Print boot message.
      app_log_time("Bluetooth stack booted: v%d.%d.%d-b%d\n",
                   evt->data.evt_system_boot.major,
                   evt->data.evt_system_boot.minor,
                   evt->data.evt_system_boot.patch,
                   evt->data.evt_system_boot.build);

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert_status(sc);

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
      app_assert_status(sc);

      app_log_time("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   address_type ? "static random" : "public device",
                   address.addr[5],
                   address.addr[4],
                   address.addr[3],
                   address.addr[2],
                   address.addr[1],
                   address.addr[0]);

#define SCAN_INTERVAL                 1600   // * 0.625ms = 1 sec
#define SCAN_WINDOW                   1600   // * 0.625ms = 1 sec
#define SCAN_PASSIVE                  0
#define SCAN_ACTIVE                   1

#define CONN_INTERVAL_MIN             6    // * 1.25 ms = 7.5 ms
#define CONN_INTERVAL_MAX             12    // * 1.25 ms = 7.5 ms
#define CONN_SLAVE_LATENCY            0    //no latency
#define CONN_TIMEOUT                  1000  //1000ms
#define CONN_MIN_CE_LENGTH            0
#define CONN_MAX_CE_LENGTH            0xffff


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
      uint16_t mtu = 200, mtu_out = 0; // 23 to 200
      sc = sl_bt_gatt_set_max_mtu(mtu, &mtu_out);
      app_assert(sc == SL_STATUS_OK,
                 "[E: 0x%04x] Failed to set max MTU size\n",
                 (int)sc);

      app_log_time("Max MTU = %d\n", mtu_out);


      // Start scanning - looking for tags
      sc = sl_bt_scanner_start(gap_1m_phy, scanner_discover_generic);
      app_log_time("Start scanning\n");
      app_assert_status(sc);


      break;



    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:

      conn = get_connection_by_address(&evt->data.evt_connection_opened.address);
      if (conn != NULL)
      {
        if (conn->connection_state == TAG_WAITING_CONNECTION)
        {

          conn->skip_adv = 1; // we need to skip next adv, as it's timing will be off
          conn->flip_connected = mt;
          conn->connection_state = TAG_CONNECTED;
          conn->connection_handle = evt->data.evt_connection_opened.connection;

          app_log_time("#%d\tConnected, handle %d, time_err %d\n",
                         conn->index,
                         conn->connection_handle,
                         mt - conn->expected_connection
                         );


          //write_data(conn);
          sc = sl_bt_connection_close(conn->connection_handle);

        }
        else if (conn->connection_state == TAG_TIMEOUT)
        {
            app_log_time("#%d\tConnection already timed out\n", conn->index);
            sl_bt_connection_close(conn->connection_handle);
        }

      }
      else // connected to unknown device
      {
        stack_state = STACK_IDLE;
        sl_bt_connection_close(evt->data.evt_connection_opened.connection);
      }



      break;



    // -------------------------------
    // This event indicates that an operation was completed (write/read/etc)
    case sl_bt_evt_gatt_procedure_completed_id:

      if (stack_state == STACK_WRITING)
      {
        conn = get_connection_by_handle(evt->data.evt_gatt_procedure_completed.connection);

        if (conn != NULL)
        {
            app_log_time("#%d\tCompleted handle %d, result %d\n",
                         conn->index,
                         conn->connection_handle,
                         evt->data.evt_gatt_procedure_completed.result);

          conn->flipped = 1;
          conn->connection_state = TAG_CONNECTED;

          app_log_time("#%d\tDone writing, handle %d, Disconnecting\n", conn->index, conn->connection_handle);

          stack_state = STACK_DISCONNECTING;
          sc = sl_bt_connection_close(conn->connection_handle);
          if (sc != SL_STATUS_OK)
            app_log_time("[E: 0x%04x] Failed to disconnect\n", (int)sc);

        }

      }
      else if (stack_state == STACK_READING)
      {
        app_log_time("\tDone reading block\n");
      }


      break;


    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      active_connections -= 1;
      stack_state = STACK_IDLE;

      conn = get_connection_by_handle(evt->data.evt_connection_closed.connection);
      if (conn != NULL)
      {
        app_log_time("#%d\tConnection closed: %d, total time %d\n", conn->index, evt->data.evt_connection_closed.connection, mt - conn->flip_start);

        conn->connection_state = TAG_NOT_CONNECTED;
        conn->connection_handle = CONNECTION_HANDLE_INVALID;
        conn->flip_complete = mt;
      }

      break;


    // -------------------------------
    case sl_bt_evt_scanner_scan_report_id:
        if (evt->data.evt_scanner_scan_report.data.len == 13)
        {
          // U2
          if ( (evt->data.evt_scanner_scan_report.data.data[11] == 'U') &&
                (evt->data.evt_scanner_scan_report.data.data[12] == '2'))
          {
            conn = get_connection_by_address(&evt->data.evt_scanner_scan_report.address);

            if (conn == NULL)
            {
              conn = add_connection_state(CONNECTION_HANDLE_INVALID,
                              &evt->data.evt_scanner_scan_report.address,
                              evt->data.evt_scanner_scan_report.address_type,
                              TAG_NOT_CONNECTED);
              if (conn)
              {
                app_log_time("#%d\tadded %02X:%02X:%02X:%02X:%02X:%02X\tLen: %d\n",
                  conn->index,
                  evt->data.evt_scanner_scan_report.address.addr[5],
                  evt->data.evt_scanner_scan_report.address.addr[4],
                  evt->data.evt_scanner_scan_report.address.addr[3],
                  evt->data.evt_scanner_scan_report.address.addr[2],
                  evt->data.evt_scanner_scan_report.address.addr[1],
                  evt->data.evt_scanner_scan_report.address.addr[0],
                  evt->data.evt_scanner_scan_report.data.len);
              }
            }



            if (conn != NULL)
            {
              update_connection_time (conn, &evt->data.evt_scanner_scan_report);
            }

          }
        }


      break;


    case sl_bt_evt_connection_parameters_id:
      app_log_time("H%d\tconn int=%d, lat=%d, sec=%d, tout=%d, txsize=%d\n",
                   evt->data.evt_connection_phy_status.connection, //conn->index,
                   evt->data.evt_connection_parameters.interval,
                   evt->data.evt_connection_parameters.latency,
                   evt->data.evt_connection_parameters.security_mode,
                   evt->data.evt_connection_parameters.timeout,
                   evt->data.evt_connection_parameters.txsize);
      break;

    case sl_bt_evt_connection_phy_status_id:
      app_log_time("H%d\tsl_bt_evt_connection_phy_status_id, phy=%d\n",
                   evt->data.evt_connection_phy_status.connection, //conn->index,
                   evt->data.evt_connection_phy_status.phy);
      break;
    case sl_bt_evt_connection_tx_power_id:
      app_log_time("H%d\tsl_bt_evt_connection_tx_power_id, power=%d\n",
                   evt->data.evt_connection_phy_status.connection, //conn->index,
                   evt->data.evt_connection_tx_power.power_level);
      break;
    case sl_bt_evt_gatt_mtu_exchanged_id:
      app_log_time("H%d\tsl_bt_evt_gatt_mtu_exchanged_id, mtu=%d\n",
                   evt->data.evt_connection_phy_status.connection, //conn->index,
                   evt->data.evt_gatt_mtu_exchanged.mtu);
      break;



    // -------------------------------
    // Default event handler.
    default:
      app_log_time("??? %x\n", SL_BT_MSG_ID(evt->header));
      break;

  }
}

