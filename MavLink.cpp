
#include "MavLink.h"
#include "libraries/Mavlink/v1.0/mavlink_types.h"
#include "libraries/Mavlink/v1.0/ardupilotmega/mavlink.h"
#include <Arduino.h>
#include "libraries/AutoSerial/AutoSerial.h"

/// MAVLink stream used for HIL interaction
extern AutoSerial mavlinkSerial;

mavlink_system_t mavlink_system = {12,1}; //modified
uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid) {
	if (sysid != mavlink_system.sysid) {
		return 1;
	}
	return 0;
}

static volatile uint16_t chan_raw [16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//MAVLink session control
static float        lastMAVBeat = 0;
static uint8_t      apm_mav_system;
static uint8_t      apm_mav_component;

#define ToRad(x) (x*0.01745329252)      // *pi/180
#define ToDeg(x) (x*57.2957795131)      // *180/pi

// true when we have received at least 1 MAVLink packet
static bool mavlink_active;

void getMavlinkRCRollPitch (uint16_t &rc_roll, uint16_t &rc_pitch, uint8_t mavlinkRollRCChannel, uint8_t mavlinkPitchRCChannel) {
	if (mavlinkPitchRCChannel>15 || mavlinkRollRCChannel>15) {
		rc_pitch = 0;
		rc_roll = 0;
		return;
	}
	rc_pitch = chan_raw[mavlinkPitchRCChannel-1];
	rc_roll = chan_raw[mavlinkRollRCChannel-1];
}

void request_mavlink_rates() {
	const int  maxStreams = 6;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_EXTENDED_STATUS,
        MAV_DATA_STREAM_RC_CHANNELS,
        MAV_DATA_STREAM_POSITION,
        MAV_DATA_STREAM_EXTRA1,
        MAV_DATA_STREAM_EXTRA2};
    const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x05, 0x02};
	/*
	const int  maxStreams = 1;
	const uint8_t MAVStreams[maxStreams] = {
			MAV_DATA_STREAM_RC_CHANNELS
	};
	const uint16_t MAVRates[maxStreams] = {0x10};
	*/
	for (int i=0; i < maxStreams; i++) {
		mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
				apm_mav_system, apm_mav_component,
				MAVStreams[i], MAVRates[i], 1);
	}
}

void read_mavlink() {
	mavlink_message_t msg;
	mavlink_status_t status;
	while(mavlinkSerial.available() > 0) {
		uint8_t c = mavlinkSerial.read();
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			lastMAVBeat = millis();
			mavlink_active = 1;

			switch(msg.msgid) {

			case MAVLINK_MSG_ID_ATTITUDE: {
				Serial.println (F("Attitude"));
			}
			break;
			case MAVLINK_MSG_ID_HEARTBEAT: {
				Serial.println (F("Heartbeat"));
			}
			break;

			case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
				Serial.println(F("Mam1!"));
				chan_raw[0] = mavlink_msg_rc_channels_raw_get_chan1_raw(&msg);
				chan_raw[1] = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
				chan_raw[2] = mavlink_msg_rc_channels_raw_get_chan3_raw(&msg);
				chan_raw[3] = mavlink_msg_rc_channels_raw_get_chan4_raw(&msg);
				chan_raw[4] = mavlink_msg_rc_channels_raw_get_chan5_raw(&msg);
				chan_raw[5] = mavlink_msg_rc_channels_raw_get_chan6_raw(&msg);
				chan_raw[6] = mavlink_msg_rc_channels_raw_get_chan7_raw(&msg);
				chan_raw[7] = mavlink_msg_rc_channels_raw_get_chan8_raw(&msg);
				for (uint8_t i=0; i<16; i++) {
					Serial.print (chan_raw[i]);
					Serial.print(F(" "));
				}
				Serial.println();
			}
			break;

			case MAVLINK_MSG_ID_RC_CHANNELS: {
				Serial.println(F("Mam2!"));
				chan_raw[0] = mavlink_msg_rc_channels_get_chan1_raw(&msg);
				chan_raw[1] = mavlink_msg_rc_channels_get_chan2_raw(&msg);
				chan_raw[2] = mavlink_msg_rc_channels_get_chan3_raw(&msg);
				chan_raw[4] = mavlink_msg_rc_channels_get_chan4_raw(&msg);
				chan_raw[5] = mavlink_msg_rc_channels_get_chan5_raw(&msg);
				chan_raw[6] = mavlink_msg_rc_channels_get_chan6_raw(&msg);
				chan_raw[7] = mavlink_msg_rc_channels_get_chan7_raw(&msg);
				chan_raw[8] = mavlink_msg_rc_channels_get_chan8_raw(&msg);
				chan_raw[9] = mavlink_msg_rc_channels_get_chan9_raw(&msg);
				chan_raw[10] = mavlink_msg_rc_channels_get_chan10_raw(&msg);
				chan_raw[11] = mavlink_msg_rc_channels_get_chan11_raw(&msg);
				chan_raw[12] = mavlink_msg_rc_channels_get_chan12_raw(&msg);
				chan_raw[13] = mavlink_msg_rc_channels_get_chan13_raw(&msg);
				chan_raw[14] = mavlink_msg_rc_channels_get_chan14_raw(&msg);
				chan_raw[15] = mavlink_msg_rc_channels_get_chan15_raw(&msg);
				for (uint8_t i=0; i<16; i++) {
					Serial.print (chan_raw[i]);
					Serial.print(F(" "));
				}
				Serial.println();
			}
			break;
			default:
				break;
			}
		}
	}
}
