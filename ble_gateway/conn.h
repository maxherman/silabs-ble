#include "em_common.h"


#ifndef CONN_H
#define CONN_H

#define CONNECTION_HANDLE_INVALID     (uint16_t)0xFFFFu
#define AOA_MAX_TAGS                  32



// Connection state, used only in connection oriented mode
typedef enum {
  TAG_EMPTY,
  TAG_NOT_CONNECTED,
  TAG_TIMEOUT,
  TAG_WAITING_CONNECTION,
  TAG_CONNECTED,

  TAG_WRITING,
  TAG_READING,
} connection_state_t;


#define ADV_COUNT   3


typedef struct {
  uint16_t index;

  uint32_t last_timestamp;
  uint16_t adv[ADV_COUNT];
  uint8_t adv_index;
  uint16_t adv_period;
  uint8_t skip_adv;

  uint8_t flipped;

  uint32_t expected_connection;
  uint32_t flip_start;
  uint32_t flip_connected;
  uint32_t flip_complete;
  uint8_t timeouts;

  uint16_t connection_handle;
  uint8_t connection_open_handle;
  bd_addr address;
  uint8_t address_type;
  connection_state_t connection_state;
} conn_properties_t;




void update_connection_time(conn_properties_t *conn, sl_bt_evt_scanner_scan_report_t *report);
void init_connection_list(void);
conn_properties_t* add_connection_state(uint16_t connection, bd_addr *address, uint8_t address_type, connection_state_t state);
conn_properties_t* get_connection_by_handle(uint16_t connection_handle);
conn_properties_t* get_connection_by_address(bd_addr* address);
void print_enumerated_tags(void);
void flip(void);
void flip_process_action(void);
uint8_t flip_in_progress (void);


#endif
