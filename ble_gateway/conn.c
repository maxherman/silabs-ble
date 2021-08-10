#include "em_common.h"
#include "app_log.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "conn.h"



static conn_properties_t conn_properties[AOA_MAX_TAGS];
static uint8_t active_connections_num;

static unsigned long flip_start = 0;
static unsigned long flip_end = 0;



void update_connection_time(conn_properties_t* conn, sl_bt_evt_scanner_scan_report_t *report)
{
  uint32_t mt = microtime();

  // If last ad is less then 50 ms - it's probably a reflection, ignore it
  if ((mt - conn->last_timestamp < 50) || (conn->skip_adv))
  {
    conn->skip_adv = 0;
    return;
  }


  uint16_t new_adv = mt - conn->last_timestamp;
  conn->last_timestamp = mt;


  // If new_adv 25% more then usual value, it means we missed adv,
  // and we need to skip this adv
  if (conn->adv_period > 0)
    if (new_adv > conn->adv_period + conn->adv_period / 4)
    {
        /*app_log_time("#%d\tSKIPPING [%d > %d]\n",
                     conn->index,
                     new_adv,
                     conn->adv_period + conn->adv_period / 4);
                     */
        return;
    }

  conn->adv[conn->adv_index] = new_adv;
  conn->adv_index++;
  conn->adv_index %= ADV_COUNT;

  uint8_t cnt = 0;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < ADV_COUNT; i++)
  {
      if (conn->adv[i] > 0)
      {
          sum += conn->adv[i];
          cnt++;
      }
  }

  conn->adv_period = sum / cnt;
/*
  app_log_time("%d\tADV\t%d\t%d\t%d\t%d\t%d\t%d\n",
               conn->index,
               conn->adv[0],
               conn->adv[1],
               conn->adv[2],
               conn->adv[3],
               conn->adv[4],
               conn->adv_period
               );
               */
}



void init_connection_list(void)
{
  uint8_t i;
  active_connections_num = 0;

  // Initialize connection state variables
  for (i = 0; i < AOA_MAX_TAGS; i++) {
    conn_properties[i].connection_handle = CONNECTION_HANDLE_INVALID;
  }
}


conn_properties_t* add_connection_state(uint16_t connection, bd_addr *address, uint8_t address_type, connection_state_t state)
{
  conn_properties_t* conn = NULL;

  // If there is place to store new connection
  if (active_connections_num < AOA_MAX_TAGS) {
    conn = &conn_properties[active_connections_num];

    // Store the connection handle, and the server address
    conn->connection_handle = connection;
    conn->address = *address;
    conn->address_type = address_type;
    conn->connection_state = state;
    conn->index = active_connections_num;

    conn->last_timestamp = microtime();
    conn->flipped = 0;
    conn->flip_complete = 0;
    conn->flip_connected = 0;
    conn->flip_start = 0;
    conn->timeouts = 0;

    for (uint8_t i = 0; i < ADV_COUNT; i++)
      conn->adv[i] = 0;
    conn->adv_index = 0;
    conn->adv_period = 0;
    conn->skip_adv = 0;

    // Entry is now valid
    active_connections_num++;
  }
  return conn;
}

conn_properties_t* get_connection_by_handle(uint16_t connection_handle)
{
  conn_properties_t* ret = NULL;
  // Find the connection state entry in the table corresponding to the connection handle
  for (uint8_t i = 0; i < active_connections_num; i++) {
    if (conn_properties[i].connection_handle == connection_handle) {
      // Return a pointer to the connection state entry
      ret = &conn_properties[i];
      break;
    }
  }
  // Return error if connection not found
  return ret;
}

conn_properties_t* get_connection_by_address(bd_addr* address)
{
  conn_properties_t* ret = NULL;
  // Find the connection state entry in the table corresponding to the connection address
  for (uint8_t i = 0; i < active_connections_num; i++) {
    if (0 == memcmp(address, &(conn_properties[i].address), sizeof(bd_addr))) {
      // Return a pointer to the connection state entry
      ret = &conn_properties[i];
      break;
    }
  }
  // Return error if connection not found
  return ret;
}

void print_enumerated_tags(void)
{
  uint32_t mt = microtime();

  app_log_time("Idx\tMAC Address\t\tHndl\tSta\tLast\tPeriod\n");

  for (int i = 0; i < AOA_MAX_TAGS; i++)
  {
      conn_properties_t *conn = &conn_properties[i];

      if (conn->connection_state == TAG_EMPTY)
        continue;

      app_log_time("#%d\t%02X:%02X:%02X:%02X:%02X:%02X\t%d\t%d\t%d\t%d\n",
              conn->index,
              conn->address.addr[5],
              conn->address.addr[4],
              conn->address.addr[3],
              conn->address.addr[2],
              conn->address.addr[1],
              conn->address.addr[0],
              conn->connection_handle,
              conn->connection_state,
              mt - conn->last_timestamp,
              conn->adv_period);

  }
}

void print_flip_stats(void)
{
  uint8_t i;

  app_log_time("\n--- FLIPPED %d tags in %d ms ---\n",
               active_connections_num,
               flip_end - flip_start);

  app_log_time("Idx\tStart\tAntErr\tWait\tDurat\tTout\n");

  for (i = 0; i < AOA_MAX_TAGS; i++)
  {
      conn_properties_t *conn = &conn_properties[i];

      if (conn->connection_state == TAG_EMPTY)
        continue;


      app_log_time("#%d\t%d\t%d\t%d\t%d\t%d\n",
              conn->index,
              conn->flip_start,
              conn->flip_connected - conn->expected_connection,
              conn->flip_connected - conn->flip_start,
              conn->flip_complete - conn->flip_connected,
              conn->timeouts);

  }
}






static uint8_t flipping = 0;
unsigned long connection_timeout = 0;
#define PRE_CONN        25     // ms
#define TIMEOUT_CONN    1000    // ms

uint8_t flip_in_progress (void)
{
  return flipping;
}

void flip(void)
{
  if (!flipping)
  {
    flipping = 1;
    print_enumerated_tags();
    app_log_time("Starting flip\n");
    flip_start = microtime();
  }
}




void flip_process_action(void)
{
  uint32_t mt = microtime();

  conn_properties_t* conn;
  int16_t index = -1;
  int16_t min_delta = -1;
  sl_status_t sc;




#ifndef NEED_ACK
  if (stack_state == STACK_WRITING)
  {
      for (int i = 0; i < AOA_MAX_TAGS; i++)
      {
          conn = &conn_properties[i];
          if (conn->connection_state == TAG_WRITING)
          {
              app_log_time("#%d\tClosing connection, handle %d\n", conn->index, conn->connection_handle);
              stack_state = STACK_DISCONNECTING;
              sc = sl_bt_connection_close(conn->connection_handle);
              if (sc != SL_STATUS_OK)
                app_log_time("[E: 0x%04x] Failed to disconnect\n", (int)sc);
          }
      }
  }
#endif






  // See if current connection timed out
  if ((connection_timeout > 0) && (mt > connection_timeout))
  {
      if (stack_state == STACK_CONNECTING)
      {
          // Find which tags is trying to connect
          for (int i = 0; i < AOA_MAX_TAGS; i++)
          {
              conn = &conn_properties[i];
              if (conn->connection_state == TAG_WAITING_CONNECTION)
              {
                  conn->flipped = 0;
                  conn->timeouts += 1;
                  conn->connection_state = TAG_NOT_CONNECTED;

                  sl_bt_connection_close(conn->connection_open_handle);

                  app_log_time("#%d\tTIMEOUT [%d > %d], closing handle %d\n",
                               conn->index,
                               connection_timeout,
                               mt,
                               conn->connection_open_handle);
                  break;
              }
          }
      }
      connection_timeout = 0;
  }


  if (!flipping)
    return;

  //if (stack_state != STACK_IDLE)
  //  return;

  if ((stack_state == STACK_CONNECTING) || (stack_state == STACK_DISCONNECTING) )
    return;

  if (active_connections > 0)
    return;


  // Find a tag that will adv next, and start the process
  index = -1;
  for (int i = 0; i < AOA_MAX_TAGS; i++)
  {
      conn = &conn_properties[i];

      // only look at the IDLE tags, that has not been flipped yet
      if ((conn->connection_state ==  TAG_NOT_CONNECTED) && (conn->flipped == 0))
      {
          // calculate time until next ad
          int16_t delta = conn->last_timestamp - mt - PRE_CONN;

          while (delta <= 0)
              delta += conn->adv_period ;
/*
          app_log_time("#%d\tDELTA\tlast = %d\tnext = %d\tdelta = %d\n",
                       conn->index,
                       conn->last_timestamp,
                       conn->last_timestamp+delta,
                       delta);
*/
          if ((delta < min_delta) || (min_delta == -1))
          {
              min_delta = delta;
              index = i;
          }

      }
  }



  if (index >= 0) // Need to start flip on another tag
  {
      uint8_t conn_handle;
      conn = &conn_properties[index];
      conn->expected_connection = mt + min_delta;
      connection_timeout = conn->expected_connection + TIMEOUT_CONN;

      stack_state = STACK_CONNECTING;
      conn->connection_state = TAG_WAITING_CONNECTION;
      conn->flip_start = mt;

      app_log_time("#%d\tConnecting, anticipate in %d, timeout %d\n",
                   index,
                   conn->expected_connection,
                   connection_timeout);

      active_connections += 1;
      //app_log_time("AC = %d\n", active_connections);

      sc = sl_bt_connection_open(
          conn->address,
          conn->address_type,
          gap_1m_phy,
          &conn->connection_open_handle);

      if (sc != SL_STATUS_OK)
          app_log_time("#%d\tERROR 0x%04x to open connection\n",index , (int)sc);
  }
  else // all tags are complete, print stats
  {
      flip_end = mt;
      app_log_time("Flip is complete\n");

      print_flip_stats();

      // Done flipping, all tags are flipped
      flipping = 0;
      for (int i = 0; i < AOA_MAX_TAGS; i++)
      {
          conn = &conn_properties[i];
          conn->flipped = 0;
          conn->timeouts = 0;
      }

  }

}



