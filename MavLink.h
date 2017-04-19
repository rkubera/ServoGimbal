#ifndef _MavLink_H_
#define _MavLink_H_
#include "libraries/AutoSerial/AutoSerial.h"
void request_mavlink_rates();
void read_mavlink();
void getMavlinkRCRollPitch (uint16_t &rc_roll, uint16_t &rc_pitch, uint8_t mavlinkRollRCChannel, uint8_t mavlinkPitchRCChannel);

// we have separate helpers disabled to make it possible
// to select MAVLink 1.0 in the arduino GUI build
//#define MAVLINK_SEPARATE_HELPERS
#include "libraries/Mavlink/v1.0/ardupilotmega/version.h"

// this allows us to make mavlink_message_t much smaller
#define MAVLINK_MAX_PAYLOAD_LEN MAVLINK_MAX_DIALECT_PAYLOAD_SIZE

#define MAVLINK_COMM_NUM_BUFFERS 1
#include "libraries/Mavlink/v1.0/mavlink_types.h"

/// MAVLink stream used for HIL interaction
extern AutoSerial mavlinkSerial;

/// MAVLink stream used for ground control communication
//extern AutoSerial	*mavlink_comm_1_port;

/// MAVLink system definition
extern mavlink_system_t mavlink_system;

/// Send a byte to the nominated MAVLink channel
///
/// @param chan		Channel to send to
/// @param ch		Byte to send
///
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
		mavlinkSerial.write(ch);
}

/// Read a byte from the nominated MAVLink channel
///
/// @param chan		Channel to receive on
/// @returns		Byte read
///
static inline uint8_t comm_receive_ch(mavlink_channel_t chan)
{
    uint8_t data = 0;

    data = mavlinkSerial.read();
    return data;
}

/// Check for available data on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
static inline uint16_t comm_get_available(mavlink_channel_t chan)
{
    uint16_t bytes = 0;
    bytes = mavlinkSerial.available();
    return bytes;
}

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "libraries/Mavlink/v1.0/ardupilotmega/mavlink.h"

uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid);

#endif
