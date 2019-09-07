#pragma once
// MESSAGE MISSION_HEIGHT_SETPOINT PACKING

#define MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT 315

MAVPACKED(
typedef struct __mavlink_mission_height_setpoint_t {
 uint64_t time_usec; /*< [us] Timestamp (synced to UNIX time or since system boot).*/
 float height; /*<  height setpoint*/
}) mavlink_mission_height_setpoint_t;

#define MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN 12
#define MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_MIN_LEN 12
#define MAVLINK_MSG_ID_315_LEN 12
#define MAVLINK_MSG_ID_315_MIN_LEN 12

#define MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_CRC 40
#define MAVLINK_MSG_ID_315_CRC 40



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MISSION_HEIGHT_SETPOINT { \
    315, \
    "MISSION_HEIGHT_SETPOINT", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mission_height_setpoint_t, time_usec) }, \
         { "height", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mission_height_setpoint_t, height) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MISSION_HEIGHT_SETPOINT { \
    "MISSION_HEIGHT_SETPOINT", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mission_height_setpoint_t, time_usec) }, \
         { "height", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mission_height_setpoint_t, height) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_height_setpoint message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param height  height setpoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_height_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, height);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN);
#else
    mavlink_mission_height_setpoint_t packet;
    packet.time_usec = time_usec;
    packet.height = height;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_CRC);
}

/**
 * @brief Pack a mission_height_setpoint message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param height  height setpoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_height_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, height);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN);
#else
    mavlink_mission_height_setpoint_t packet;
    packet.time_usec = time_usec;
    packet.height = height;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_CRC);
}

/**
 * @brief Encode a mission_height_setpoint struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_height_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_height_setpoint_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_height_setpoint_t* mission_height_setpoint)
{
    return mavlink_msg_mission_height_setpoint_pack(system_id, component_id, msg, mission_height_setpoint->time_usec, mission_height_setpoint->height);
}

/**
 * @brief Encode a mission_height_setpoint struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_height_setpoint C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_height_setpoint_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_height_setpoint_t* mission_height_setpoint)
{
    return mavlink_msg_mission_height_setpoint_pack_chan(system_id, component_id, chan, msg, mission_height_setpoint->time_usec, mission_height_setpoint->height);
}

/**
 * @brief Send a mission_height_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param height  height setpoint
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_height_setpoint_send(mavlink_channel_t chan, uint64_t time_usec, float height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, height);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT, buf, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_CRC);
#else
    mavlink_mission_height_setpoint_t packet;
    packet.time_usec = time_usec;
    packet.height = height;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT, (const char *)&packet, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_CRC);
#endif
}

/**
 * @brief Send a mission_height_setpoint message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_height_setpoint_send_struct(mavlink_channel_t chan, const mavlink_mission_height_setpoint_t* mission_height_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_height_setpoint_send(chan, mission_height_setpoint->time_usec, mission_height_setpoint->height);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT, (const char *)mission_height_setpoint, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_CRC);
#endif
}

#if MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_height_setpoint_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, height);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT, buf, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_CRC);
#else
    mavlink_mission_height_setpoint_t *packet = (mavlink_mission_height_setpoint_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->height = height;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT, (const char *)packet, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_HEIGHT_SETPOINT UNPACKING


/**
 * @brief Get field time_usec from mission_height_setpoint message
 *
 * @return [us] Timestamp (synced to UNIX time or since system boot).
 */
static inline uint64_t mavlink_msg_mission_height_setpoint_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field height from mission_height_setpoint message
 *
 * @return  height setpoint
 */
static inline float mavlink_msg_mission_height_setpoint_get_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a mission_height_setpoint message into a struct
 *
 * @param msg The message to decode
 * @param mission_height_setpoint C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_height_setpoint_decode(const mavlink_message_t* msg, mavlink_mission_height_setpoint_t* mission_height_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mission_height_setpoint->time_usec = mavlink_msg_mission_height_setpoint_get_time_usec(msg);
    mission_height_setpoint->height = mavlink_msg_mission_height_setpoint_get_height(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN? msg->len : MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN;
        memset(mission_height_setpoint, 0, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN);
    memcpy(mission_height_setpoint, _MAV_PAYLOAD(msg), len);
#endif
}
