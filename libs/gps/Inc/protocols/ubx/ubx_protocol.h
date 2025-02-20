// protocols/ubx/ubx_protocol.h
#pragma once
#include "ubx_types.h"
#include "ubx_defs.h"
#include "ubx_messages.h"
#include "gps_types.h"
#include "ubx_packet_handler.h"
#include "stdint.h"
#include "i2c.h"
#include "logging.h"


gps_status_e ubx_send_command(ubx_protocol_t proto, bool ack_only);
gps_status_e ubx_free_frame(ubx_protocol_t* proto);
//TODO: I don't like this function name. It's not clear what it does
gps_status_e ubx_extract_pvt_data(ubx_protocol_t* proto, gps_data_type_e type, gps_data_t* data);
gps_status_e ubx_get_nav_status(ubx_protocol_t* proto);
gps_status_e ubx_configure_i2c(ubx_protocol_t* proto);
gps_status_e ubx_reset(ubx_protocol_t* proto);
