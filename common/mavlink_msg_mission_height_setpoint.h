#pragma once
// MESSAGE MISSION_HEIGHT_SETPOINT PACKING

#define MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT 315

MAVPACKED(
typedef struct __mavlink_mission_height_setpoint_t {
 uint64_t time_usec; /*< [us] Timestamp (synced to UNIX time or since system boot).*/
 float height; /*<  Height setpoint to be given to MAV*/
 float yaw; /*<  Fixed Yaw to be given to MAV*/
 uint16_t avoidance_flag; /*<  Flag which denotes whether avoidance will run or not*/
}) mavlink_mission_height_setpoint_t;

#define MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN 18
#define MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_MIN_LEN 18
#define MAVLINK_MSG_ID_315_LEN 18
#define MAVLINK_MSG_ID_315_MIN_LEN 18

#define MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_CRC 248
#define MAVLINK_MSG_ID_315_CRC 248



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MISSION_HEIGHT_SETPOINT { \
    315, \
    "MISSION_HEIGHT_SETPOINT", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mission_height_setpoint_t, time_usec) }, \
         { "height", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mission_height_setpoint_t, height) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mission_height_setpoint_t, yaw) }, \
         { "avoidance_flag", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_mission_height_setpoint_t, avoidance_flag) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MISSION_HEIGHT_SETPOINT { \
    "MISSION_HEIGHT_SETPOINT", \
    4, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_mission_height_setpoint_t, time_usec) }, \
         { "height", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_mission_height_setpoint_t, height) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_mission_height_setpoint_t, yaw) }, \
         { "avoidance_flag", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_mission_height_setpoint_t, avoidance_flag) }, \
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
 * @param height  Height setpoint to be given to MAV
 * @param yaw  Fixed Yaw to be given to MAV
 * @param avoidance_flag  Flag which denotes whether avoidance will run or not
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_height_setpoint_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float height, float yaw, uint16_t avoidance_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, height);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint16_t(buf, 16, avoidance_flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN);
#else
    mavlink_mission_height_setpoint_t packet;
    packet.time_usec = time_usec;
    packet.height = height;
    packet.yaw = yaw;
    packet.avoidance_flag = avoidance_flag;

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
 * @param height  Height setpoint to be given to MAV
 * @param yaw  Fixed Yaw to be given to MAV
 * @param avoidance_flag  Flag which denotes whether avoidance will run or not
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_height_setpoint_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float height,float yaw,uint16_t avoidance_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, height);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint16_t(buf, 16, avoidance_flag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN);
#else
    mavlink_mission_height_setpoint_t packet;
    packet.time_usec = time_usec;
    packet.height = height;
    packet.yaw = yaw;
    packet.avoidance_flag = avoidance_flag;

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
    return mavlink_msg_mission_height_setpoint_pack(system_id, component_id, msg, mission_height_setpoint->time_usec, mission_height_setpoint->height, mission_height_setpoint->yaw, mission_height_setpoint->avoidance_flag);
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
    return mavlink_msg_mission_height_setpoint_pack_chan(system_id, component_id, chan, msg, mission_height_setpoint->time_usec, mission_height_setpoint->height, mission_height_setpoint->yaw, mission_height_setpoint->avoidance_flag);
}

/**
 * @brief Send a mission_height_setpoint message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param height  Height setpoint to be given to MAV
 * @param yaw  Fixed Yaw to be given to MAV
 * @param avoidance_flag  Flag which denotes whether avoidance will run or not
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_height_setpoint_send(mavlink_channel_t chan, uint64_t time_usec, float height, float yaw, uint16_t avoidance_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, height);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint16_t(buf, 16, avoidance_flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT, buf, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_CRC);
#else
    mavlink_mission_height_setpoint_t packet;
    packet.time_usec = time_usec;
    packet.height = height;
    packet.yaw = yaw;
    packet.avoidance_flag = avoidance_flag;

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
    mavlink_msg_mission_height_setpoint_send(chan, mission_height_setpoint->time_usec, mission_height_setpoint->height, mission_height_setpoint->yaw, mission_height_setpoint->avoidance_flag);
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
static inline void mavlink_msg_mission_height_setpoint_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float height, float yaw, uint16_t avoidance_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, height);
    _mav_put_float(buf, 12, yaw);
    _mav_put_uint16_t(buf, 16, avoidance_flag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT, buf, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_MIN_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_CRC);
#else
    mavlink_mission_height_setpoint_t *packet = (mavlink_mission_height_setpoint_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->height = height;
    packet->yaw = yaw;
    packet->avoidance_flag = avoidance_flag;

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
 * @return  Height setpoint to be given to MAV
 */
static inline float mavlink_msg_mission_height_setpoint_get_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from mission_height_setpoint message
 *
 * @return  Fixed Yaw to be given to MAV
 */
static inline float mavlink_msg_mission_height_setpoint_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field avoidance_flag from mission_height_setpoint message
 *
 * @return  Flag which denotes whether avoidance will run or not
 */
static inline uint16_t mavlink_msg_mission_height_setpoint_get_avoidance_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  16);
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
    mission_height_setpoint->yaw = mavlink_msg_mission_height_setpoint_get_yaw(msg);
    mission_height_setpoint->avoidance_flag = mavlink_msg_mission_height_setpoint_get_avoidance_flag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN? msg->len : MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN;
        memset(mission_height_setpoint, 0, MAVLINK_MSG_ID_MISSION_HEIGHT_SETPOINT_LEN);
    memcpy(mission_height_setpoint, _MAV_PAYLOAD(msg), len);
#endif
}
