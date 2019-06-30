#pragma once
// MESSAGE GRIPPER_SERVO PACKING

#define MAVLINK_MSG_ID_GRIPPER_SERVO 229

MAVPACKED(
typedef struct __mavlink_gripper_servo_t {
 uint64_t time_usec; /*< [us] Timestamp (synced to UNIX time or since system boot).*/
 float servo_setpoint; /*<  actuator controls*/
}) mavlink_gripper_servo_t;

#define MAVLINK_MSG_ID_GRIPPER_SERVO_LEN 12
#define MAVLINK_MSG_ID_GRIPPER_SERVO_MIN_LEN 12
#define MAVLINK_MSG_ID_229_LEN 12
#define MAVLINK_MSG_ID_229_MIN_LEN 12

#define MAVLINK_MSG_ID_GRIPPER_SERVO_CRC 175
#define MAVLINK_MSG_ID_229_CRC 175



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GRIPPER_SERVO { \
    229, \
    "GRIPPER_SERVO", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gripper_servo_t, time_usec) }, \
         { "servo_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gripper_servo_t, servo_setpoint) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GRIPPER_SERVO { \
    "GRIPPER_SERVO", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gripper_servo_t, time_usec) }, \
         { "servo_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gripper_servo_t, servo_setpoint) }, \
         } \
}
#endif

/**
 * @brief Pack a gripper_servo message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param servo_setpoint  actuator controls
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gripper_servo_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float servo_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GRIPPER_SERVO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, servo_setpoint);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GRIPPER_SERVO_LEN);
#else
    mavlink_gripper_servo_t packet;
    packet.time_usec = time_usec;
    packet.servo_setpoint = servo_setpoint;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GRIPPER_SERVO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GRIPPER_SERVO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GRIPPER_SERVO_MIN_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_CRC);
}

/**
 * @brief Pack a gripper_servo message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param servo_setpoint  actuator controls
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gripper_servo_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float servo_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GRIPPER_SERVO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, servo_setpoint);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GRIPPER_SERVO_LEN);
#else
    mavlink_gripper_servo_t packet;
    packet.time_usec = time_usec;
    packet.servo_setpoint = servo_setpoint;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GRIPPER_SERVO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GRIPPER_SERVO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GRIPPER_SERVO_MIN_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_CRC);
}

/**
 * @brief Encode a gripper_servo struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gripper_servo C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gripper_servo_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gripper_servo_t* gripper_servo)
{
    return mavlink_msg_gripper_servo_pack(system_id, component_id, msg, gripper_servo->time_usec, gripper_servo->servo_setpoint);
}

/**
 * @brief Encode a gripper_servo struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gripper_servo C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gripper_servo_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gripper_servo_t* gripper_servo)
{
    return mavlink_msg_gripper_servo_pack_chan(system_id, component_id, chan, msg, gripper_servo->time_usec, gripper_servo->servo_setpoint);
}

/**
 * @brief Send a gripper_servo message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (synced to UNIX time or since system boot).
 * @param servo_setpoint  actuator controls
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gripper_servo_send(mavlink_channel_t chan, uint64_t time_usec, float servo_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GRIPPER_SERVO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, servo_setpoint);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GRIPPER_SERVO, buf, MAVLINK_MSG_ID_GRIPPER_SERVO_MIN_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_CRC);
#else
    mavlink_gripper_servo_t packet;
    packet.time_usec = time_usec;
    packet.servo_setpoint = servo_setpoint;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GRIPPER_SERVO, (const char *)&packet, MAVLINK_MSG_ID_GRIPPER_SERVO_MIN_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_CRC);
#endif
}

/**
 * @brief Send a gripper_servo message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gripper_servo_send_struct(mavlink_channel_t chan, const mavlink_gripper_servo_t* gripper_servo)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gripper_servo_send(chan, gripper_servo->time_usec, gripper_servo->servo_setpoint);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GRIPPER_SERVO, (const char *)gripper_servo, MAVLINK_MSG_ID_GRIPPER_SERVO_MIN_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_CRC);
#endif
}

#if MAVLINK_MSG_ID_GRIPPER_SERVO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gripper_servo_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float servo_setpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, servo_setpoint);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GRIPPER_SERVO, buf, MAVLINK_MSG_ID_GRIPPER_SERVO_MIN_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_CRC);
#else
    mavlink_gripper_servo_t *packet = (mavlink_gripper_servo_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->servo_setpoint = servo_setpoint;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GRIPPER_SERVO, (const char *)packet, MAVLINK_MSG_ID_GRIPPER_SERVO_MIN_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_LEN, MAVLINK_MSG_ID_GRIPPER_SERVO_CRC);
#endif
}
#endif

#endif

// MESSAGE GRIPPER_SERVO UNPACKING


/**
 * @brief Get field time_usec from gripper_servo message
 *
 * @return [us] Timestamp (synced to UNIX time or since system boot).
 */
static inline uint64_t mavlink_msg_gripper_servo_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field servo_setpoint from gripper_servo message
 *
 * @return  actuator controls
 */
static inline float mavlink_msg_gripper_servo_get_servo_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a gripper_servo message into a struct
 *
 * @param msg The message to decode
 * @param gripper_servo C-struct to decode the message contents into
 */
static inline void mavlink_msg_gripper_servo_decode(const mavlink_message_t* msg, mavlink_gripper_servo_t* gripper_servo)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gripper_servo->time_usec = mavlink_msg_gripper_servo_get_time_usec(msg);
    gripper_servo->servo_setpoint = mavlink_msg_gripper_servo_get_servo_setpoint(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GRIPPER_SERVO_LEN? msg->len : MAVLINK_MSG_ID_GRIPPER_SERVO_LEN;
        memset(gripper_servo, 0, MAVLINK_MSG_ID_GRIPPER_SERVO_LEN);
    memcpy(gripper_servo, _MAV_PAYLOAD(msg), len);
#endif
}
