#pragma once
// MESSAGE LADGNSS_TRANSMIT_END PACKING

#define MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END 3

MAVPACKED(
typedef struct __mavlink_ladgnss_transmit_end_t {
 uint8_t reserved; /*< Reserved*/
}) mavlink_ladgnss_transmit_end_t;

#define MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN 1
#define MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_MIN_LEN 1
#define MAVLINK_MSG_ID_3_LEN 1
#define MAVLINK_MSG_ID_3_MIN_LEN 1

#define MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_CRC 2
#define MAVLINK_MSG_ID_3_CRC 2



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LADGNSS_TRANSMIT_END { \
    3, \
    "LADGNSS_TRANSMIT_END", \
    1, \
    {  { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_ladgnss_transmit_end_t, reserved) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LADGNSS_TRANSMIT_END { \
    "LADGNSS_TRANSMIT_END", \
    1, \
    {  { "reserved", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_ladgnss_transmit_end_t, reserved) }, \
         } \
}
#endif

/**
 * @brief Pack a ladgnss_transmit_end message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param reserved Reserved
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ladgnss_transmit_end_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN];
    _mav_put_uint8_t(buf, 0, reserved);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN);
#else
    mavlink_ladgnss_transmit_end_t packet;
    packet.reserved = reserved;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_CRC);
}

/**
 * @brief Pack a ladgnss_transmit_end message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param reserved Reserved
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ladgnss_transmit_end_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN];
    _mav_put_uint8_t(buf, 0, reserved);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN);
#else
    mavlink_ladgnss_transmit_end_t packet;
    packet.reserved = reserved;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_CRC);
}

/**
 * @brief Encode a ladgnss_transmit_end struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ladgnss_transmit_end C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ladgnss_transmit_end_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ladgnss_transmit_end_t* ladgnss_transmit_end)
{
    return mavlink_msg_ladgnss_transmit_end_pack(system_id, component_id, msg, ladgnss_transmit_end->reserved);
}

/**
 * @brief Encode a ladgnss_transmit_end struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ladgnss_transmit_end C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ladgnss_transmit_end_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ladgnss_transmit_end_t* ladgnss_transmit_end)
{
    return mavlink_msg_ladgnss_transmit_end_pack_chan(system_id, component_id, chan, msg, ladgnss_transmit_end->reserved);
}

/**
 * @brief Send a ladgnss_transmit_end message
 * @param chan MAVLink channel to send the message
 *
 * @param reserved Reserved
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ladgnss_transmit_end_send(mavlink_channel_t chan, uint8_t reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN];
    _mav_put_uint8_t(buf, 0, reserved);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END, buf, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_CRC);
#else
    mavlink_ladgnss_transmit_end_t packet;
    packet.reserved = reserved;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END, (const char *)&packet, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_CRC);
#endif
}

/**
 * @brief Send a ladgnss_transmit_end message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ladgnss_transmit_end_send_struct(mavlink_channel_t chan, const mavlink_ladgnss_transmit_end_t* ladgnss_transmit_end)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ladgnss_transmit_end_send(chan, ladgnss_transmit_end->reserved);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END, (const char *)ladgnss_transmit_end, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_CRC);
#endif
}

#if MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ladgnss_transmit_end_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t reserved)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, reserved);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END, buf, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_CRC);
#else
    mavlink_ladgnss_transmit_end_t *packet = (mavlink_ladgnss_transmit_end_t *)msgbuf;
    packet->reserved = reserved;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END, (const char *)packet, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_CRC);
#endif
}
#endif

#endif

// MESSAGE LADGNSS_TRANSMIT_END UNPACKING


/**
 * @brief Get field reserved from ladgnss_transmit_end message
 *
 * @return Reserved
 */
static inline uint8_t mavlink_msg_ladgnss_transmit_end_get_reserved(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a ladgnss_transmit_end message into a struct
 *
 * @param msg The message to decode
 * @param ladgnss_transmit_end C-struct to decode the message contents into
 */
static inline void mavlink_msg_ladgnss_transmit_end_decode(const mavlink_message_t* msg, mavlink_ladgnss_transmit_end_t* ladgnss_transmit_end)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ladgnss_transmit_end->reserved = mavlink_msg_ladgnss_transmit_end_get_reserved(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN? msg->len : MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN;
        memset(ladgnss_transmit_end, 0, MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_LEN);
    memcpy(ladgnss_transmit_end, _MAV_PAYLOAD(msg), len);
#endif
}
