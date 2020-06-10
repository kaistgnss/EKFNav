#pragma once
// MESSAGE LADGNSS_MEASUREMENT_BLOCK PACKING

#define MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK 2

MAVPACKED(
typedef struct __mavlink_ladgnss_measurement_block_t {
 int16_t prc_100s; /*< Pseudorange correction (PRC) 100s*/
 int16_t rrc_100s; /*< Pseudorange rate correction (RRC) 100s*/
 int16_t prc_30s; /*< Pseudorange correction (PRC) 30s*/
 int16_t rrc_30s; /*< Pseudorange rate correction (RRC) 30s*/
 uint8_t ranging_source_id; /*< Ranging source ID*/
 uint32_t iod; /*< Issue of data (IOD)*/
 uint8_t sigma_pr_gnd_100s; /*< Sigma_pr_gnd (100s)*/
 uint8_t sigma_pr_gnd_30s; /*< Sigma_pr_gnd (30s)*/
 int8_t b1; /*< B1*/
 int8_t b2; /*< B2*/
 int8_t b3; /*< B3*/
 int8_t b4; /*< B4*/
}) mavlink_ladgnss_measurement_block_t;

#define MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN 16
#define MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_MIN_LEN 16
#define MAVLINK_MSG_ID_2_LEN 16
#define MAVLINK_MSG_ID_2_MIN_LEN 16

#define MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_CRC 124
#define MAVLINK_MSG_ID_2_CRC 124



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LADGNSS_MEASUREMENT_BLOCK { \
    2, \
    "LADGNSS_MEASUREMENT_BLOCK", \
    12, \
    {  { "prc_100s", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_ladgnss_measurement_block_t, prc_100s) }, \
         { "rrc_100s", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_ladgnss_measurement_block_t, rrc_100s) }, \
         { "prc_30s", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_ladgnss_measurement_block_t, prc_30s) }, \
         { "rrc_30s", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_ladgnss_measurement_block_t, rrc_30s) }, \
         { "ranging_source_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_ladgnss_measurement_block_t, ranging_source_id) }, \
         { "iod", NULL, MAVLINK_TYPE_UINT32_T, 0, 9, offsetof(mavlink_ladgnss_measurement_block_t, iod) }, \
         { "sigma_pr_gnd_100s", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_ladgnss_measurement_block_t, sigma_pr_gnd_100s) }, \
         { "sigma_pr_gnd_30s", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_ladgnss_measurement_block_t, sigma_pr_gnd_30s) }, \
         { "b1", NULL, MAVLINK_TYPE_INT8_T, 0, 12, offsetof(mavlink_ladgnss_measurement_block_t, b1) }, \
         { "b2", NULL, MAVLINK_TYPE_INT8_T, 0, 13, offsetof(mavlink_ladgnss_measurement_block_t, b2) }, \
         { "b3", NULL, MAVLINK_TYPE_INT8_T, 0, 14, offsetof(mavlink_ladgnss_measurement_block_t, b3) }, \
         { "b4", NULL, MAVLINK_TYPE_INT8_T, 0, 15, offsetof(mavlink_ladgnss_measurement_block_t, b4) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LADGNSS_MEASUREMENT_BLOCK { \
    "LADGNSS_MEASUREMENT_BLOCK", \
    12, \
    {  { "prc_100s", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_ladgnss_measurement_block_t, prc_100s) }, \
         { "rrc_100s", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_ladgnss_measurement_block_t, rrc_100s) }, \
         { "prc_30s", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_ladgnss_measurement_block_t, prc_30s) }, \
         { "rrc_30s", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_ladgnss_measurement_block_t, rrc_30s) }, \
         { "ranging_source_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_ladgnss_measurement_block_t, ranging_source_id) }, \
         { "iod", NULL, MAVLINK_TYPE_UINT32_T, 0, 9, offsetof(mavlink_ladgnss_measurement_block_t, iod) }, \
         { "sigma_pr_gnd_100s", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_ladgnss_measurement_block_t, sigma_pr_gnd_100s) }, \
         { "sigma_pr_gnd_30s", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_ladgnss_measurement_block_t, sigma_pr_gnd_30s) }, \
         { "b1", NULL, MAVLINK_TYPE_INT8_T, 0, 12, offsetof(mavlink_ladgnss_measurement_block_t, b1) }, \
         { "b2", NULL, MAVLINK_TYPE_INT8_T, 0, 13, offsetof(mavlink_ladgnss_measurement_block_t, b2) }, \
         { "b3", NULL, MAVLINK_TYPE_INT8_T, 0, 14, offsetof(mavlink_ladgnss_measurement_block_t, b3) }, \
         { "b4", NULL, MAVLINK_TYPE_INT8_T, 0, 15, offsetof(mavlink_ladgnss_measurement_block_t, b4) }, \
         } \
}
#endif

/**
 * @brief Pack a ladgnss_measurement_block message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ranging_source_id Ranging source ID
 * @param iod Issue of data (IOD)
 * @param prc_100s Pseudorange correction (PRC) 100s
 * @param rrc_100s Pseudorange rate correction (RRC) 100s
 * @param prc_30s Pseudorange correction (PRC) 30s
 * @param rrc_30s Pseudorange rate correction (RRC) 30s
 * @param sigma_pr_gnd_100s Sigma_pr_gnd (100s)
 * @param sigma_pr_gnd_30s Sigma_pr_gnd (30s)
 * @param b1 B1
 * @param b2 B2
 * @param b3 B3
 * @param b4 B4
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ladgnss_measurement_block_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t ranging_source_id, uint32_t iod, int16_t prc_100s, int16_t rrc_100s, int16_t prc_30s, int16_t rrc_30s, uint8_t sigma_pr_gnd_100s, uint8_t sigma_pr_gnd_30s, int8_t b1, int8_t b2, int8_t b3, int8_t b4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN];
    _mav_put_int16_t(buf, 0, prc_100s);
    _mav_put_int16_t(buf, 2, rrc_100s);
    _mav_put_int16_t(buf, 4, prc_30s);
    _mav_put_int16_t(buf, 6, rrc_30s);
    _mav_put_uint8_t(buf, 8, ranging_source_id);
    _mav_put_uint32_t(buf, 9, iod);
    _mav_put_uint8_t(buf, 10, sigma_pr_gnd_100s);
    _mav_put_uint8_t(buf, 11, sigma_pr_gnd_30s);
    _mav_put_int8_t(buf, 12, b1);
    _mav_put_int8_t(buf, 13, b2);
    _mav_put_int8_t(buf, 14, b3);
    _mav_put_int8_t(buf, 15, b4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN);
#else
    mavlink_ladgnss_measurement_block_t packet;
    packet.prc_100s = prc_100s;
    packet.rrc_100s = rrc_100s;
    packet.prc_30s = prc_30s;
    packet.rrc_30s = rrc_30s;
    packet.ranging_source_id = ranging_source_id;
    packet.iod = iod;
    packet.sigma_pr_gnd_100s = sigma_pr_gnd_100s;
    packet.sigma_pr_gnd_30s = sigma_pr_gnd_30s;
    packet.b1 = b1;
    packet.b2 = b2;
    packet.b3 = b3;
    packet.b4 = b4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_CRC);
}

/**
 * @brief Pack a ladgnss_measurement_block message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ranging_source_id Ranging source ID
 * @param iod Issue of data (IOD)
 * @param prc_100s Pseudorange correction (PRC) 100s
 * @param rrc_100s Pseudorange rate correction (RRC) 100s
 * @param prc_30s Pseudorange correction (PRC) 30s
 * @param rrc_30s Pseudorange rate correction (RRC) 30s
 * @param sigma_pr_gnd_100s Sigma_pr_gnd (100s)
 * @param sigma_pr_gnd_30s Sigma_pr_gnd (30s)
 * @param b1 B1
 * @param b2 B2
 * @param b3 B3
 * @param b4 B4
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ladgnss_measurement_block_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t ranging_source_id,uint32_t iod,int16_t prc_100s,int16_t rrc_100s,int16_t prc_30s,int16_t rrc_30s,uint8_t sigma_pr_gnd_100s,uint8_t sigma_pr_gnd_30s,int8_t b1,int8_t b2,int8_t b3,int8_t b4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN];
    _mav_put_int16_t(buf, 0, prc_100s);
    _mav_put_int16_t(buf, 2, rrc_100s);
    _mav_put_int16_t(buf, 4, prc_30s);
    _mav_put_int16_t(buf, 6, rrc_30s);
    _mav_put_uint8_t(buf, 8, ranging_source_id);
    _mav_put_uint32_t(buf, 9, iod);
    _mav_put_uint8_t(buf, 10, sigma_pr_gnd_100s);
    _mav_put_uint8_t(buf, 11, sigma_pr_gnd_30s);
    _mav_put_int8_t(buf, 12, b1);
    _mav_put_int8_t(buf, 13, b2);
    _mav_put_int8_t(buf, 14, b3);
    _mav_put_int8_t(buf, 15, b4);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN);
#else
    mavlink_ladgnss_measurement_block_t packet;
    packet.prc_100s = prc_100s;
    packet.rrc_100s = rrc_100s;
    packet.prc_30s = prc_30s;
    packet.rrc_30s = rrc_30s;
    packet.ranging_source_id = ranging_source_id;
    packet.iod = iod;
    packet.sigma_pr_gnd_100s = sigma_pr_gnd_100s;
    packet.sigma_pr_gnd_30s = sigma_pr_gnd_30s;
    packet.b1 = b1;
    packet.b2 = b2;
    packet.b3 = b3;
    packet.b4 = b4;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_CRC);
}

/**
 * @brief Encode a ladgnss_measurement_block struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ladgnss_measurement_block C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ladgnss_measurement_block_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ladgnss_measurement_block_t* ladgnss_measurement_block)
{
    return mavlink_msg_ladgnss_measurement_block_pack(system_id, component_id, msg, ladgnss_measurement_block->ranging_source_id, ladgnss_measurement_block->iod, ladgnss_measurement_block->prc_100s, ladgnss_measurement_block->rrc_100s, ladgnss_measurement_block->prc_30s, ladgnss_measurement_block->rrc_30s, ladgnss_measurement_block->sigma_pr_gnd_100s, ladgnss_measurement_block->sigma_pr_gnd_30s, ladgnss_measurement_block->b1, ladgnss_measurement_block->b2, ladgnss_measurement_block->b3, ladgnss_measurement_block->b4);
}

/**
 * @brief Encode a ladgnss_measurement_block struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ladgnss_measurement_block C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ladgnss_measurement_block_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ladgnss_measurement_block_t* ladgnss_measurement_block)
{
    return mavlink_msg_ladgnss_measurement_block_pack_chan(system_id, component_id, chan, msg, ladgnss_measurement_block->ranging_source_id, ladgnss_measurement_block->iod, ladgnss_measurement_block->prc_100s, ladgnss_measurement_block->rrc_100s, ladgnss_measurement_block->prc_30s, ladgnss_measurement_block->rrc_30s, ladgnss_measurement_block->sigma_pr_gnd_100s, ladgnss_measurement_block->sigma_pr_gnd_30s, ladgnss_measurement_block->b1, ladgnss_measurement_block->b2, ladgnss_measurement_block->b3, ladgnss_measurement_block->b4);
}

/**
 * @brief Send a ladgnss_measurement_block message
 * @param chan MAVLink channel to send the message
 *
 * @param ranging_source_id Ranging source ID
 * @param iod Issue of data (IOD)
 * @param prc_100s Pseudorange correction (PRC) 100s
 * @param rrc_100s Pseudorange rate correction (RRC) 100s
 * @param prc_30s Pseudorange correction (PRC) 30s
 * @param rrc_30s Pseudorange rate correction (RRC) 30s
 * @param sigma_pr_gnd_100s Sigma_pr_gnd (100s)
 * @param sigma_pr_gnd_30s Sigma_pr_gnd (30s)
 * @param b1 B1
 * @param b2 B2
 * @param b3 B3
 * @param b4 B4
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ladgnss_measurement_block_send(mavlink_channel_t chan, uint8_t ranging_source_id, uint32_t iod, int16_t prc_100s, int16_t rrc_100s, int16_t prc_30s, int16_t rrc_30s, uint8_t sigma_pr_gnd_100s, uint8_t sigma_pr_gnd_30s, int8_t b1, int8_t b2, int8_t b3, int8_t b4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN];
    _mav_put_int16_t(buf, 0, prc_100s);
    _mav_put_int16_t(buf, 2, rrc_100s);
    _mav_put_int16_t(buf, 4, prc_30s);
    _mav_put_int16_t(buf, 6, rrc_30s);
    _mav_put_uint8_t(buf, 8, ranging_source_id);
    _mav_put_uint32_t(buf, 9, iod);
    _mav_put_uint8_t(buf, 10, sigma_pr_gnd_100s);
    _mav_put_uint8_t(buf, 11, sigma_pr_gnd_30s);
    _mav_put_int8_t(buf, 12, b1);
    _mav_put_int8_t(buf, 13, b2);
    _mav_put_int8_t(buf, 14, b3);
    _mav_put_int8_t(buf, 15, b4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK, buf, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_CRC);
#else
    mavlink_ladgnss_measurement_block_t packet;
    packet.prc_100s = prc_100s;
    packet.rrc_100s = rrc_100s;
    packet.prc_30s = prc_30s;
    packet.rrc_30s = rrc_30s;
    packet.ranging_source_id = ranging_source_id;
    packet.iod = iod;
    packet.sigma_pr_gnd_100s = sigma_pr_gnd_100s;
    packet.sigma_pr_gnd_30s = sigma_pr_gnd_30s;
    packet.b1 = b1;
    packet.b2 = b2;
    packet.b3 = b3;
    packet.b4 = b4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK, (const char *)&packet, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_CRC);
#endif
}

/**
 * @brief Send a ladgnss_measurement_block message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ladgnss_measurement_block_send_struct(mavlink_channel_t chan, const mavlink_ladgnss_measurement_block_t* ladgnss_measurement_block)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ladgnss_measurement_block_send(chan, ladgnss_measurement_block->ranging_source_id, ladgnss_measurement_block->iod, ladgnss_measurement_block->prc_100s, ladgnss_measurement_block->rrc_100s, ladgnss_measurement_block->prc_30s, ladgnss_measurement_block->rrc_30s, ladgnss_measurement_block->sigma_pr_gnd_100s, ladgnss_measurement_block->sigma_pr_gnd_30s, ladgnss_measurement_block->b1, ladgnss_measurement_block->b2, ladgnss_measurement_block->b3, ladgnss_measurement_block->b4);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK, (const char *)ladgnss_measurement_block, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_CRC);
#endif
}

#if MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ladgnss_measurement_block_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t ranging_source_id, uint32_t iod, int16_t prc_100s, int16_t rrc_100s, int16_t prc_30s, int16_t rrc_30s, uint8_t sigma_pr_gnd_100s, uint8_t sigma_pr_gnd_30s, int8_t b1, int8_t b2, int8_t b3, int8_t b4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, prc_100s);
    _mav_put_int16_t(buf, 2, rrc_100s);
    _mav_put_int16_t(buf, 4, prc_30s);
    _mav_put_int16_t(buf, 6, rrc_30s);
    _mav_put_uint8_t(buf, 8, ranging_source_id);
    _mav_put_uint32_t(buf, 9, iod);
    _mav_put_uint8_t(buf, 10, sigma_pr_gnd_100s);
    _mav_put_uint8_t(buf, 11, sigma_pr_gnd_30s);
    _mav_put_int8_t(buf, 12, b1);
    _mav_put_int8_t(buf, 13, b2);
    _mav_put_int8_t(buf, 14, b3);
    _mav_put_int8_t(buf, 15, b4);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK, buf, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_CRC);
#else
    mavlink_ladgnss_measurement_block_t *packet = (mavlink_ladgnss_measurement_block_t *)msgbuf;
    packet->prc_100s = prc_100s;
    packet->rrc_100s = rrc_100s;
    packet->prc_30s = prc_30s;
    packet->rrc_30s = rrc_30s;
    packet->ranging_source_id = ranging_source_id;
    packet->iod = iod;
    packet->sigma_pr_gnd_100s = sigma_pr_gnd_100s;
    packet->sigma_pr_gnd_30s = sigma_pr_gnd_30s;
    packet->b1 = b1;
    packet->b2 = b2;
    packet->b3 = b3;
    packet->b4 = b4;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK, (const char *)packet, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_CRC);
#endif
}
#endif

#endif

// MESSAGE LADGNSS_MEASUREMENT_BLOCK UNPACKING


/**
 * @brief Get field ranging_source_id from ladgnss_measurement_block message
 *
 * @return Ranging source ID
 */
static inline uint8_t mavlink_msg_ladgnss_measurement_block_get_ranging_source_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field iod from ladgnss_measurement_block message
 *
 * @return Issue of data (IOD)
 */
static inline uint32_t mavlink_msg_ladgnss_measurement_block_get_iod(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  9);
}

/**
 * @brief Get field prc_100s from ladgnss_measurement_block message
 *
 * @return Pseudorange correction (PRC) 100s
 */
static inline int16_t mavlink_msg_ladgnss_measurement_block_get_prc_100s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field rrc_100s from ladgnss_measurement_block message
 *
 * @return Pseudorange rate correction (RRC) 100s
 */
static inline int16_t mavlink_msg_ladgnss_measurement_block_get_rrc_100s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field prc_30s from ladgnss_measurement_block message
 *
 * @return Pseudorange correction (PRC) 30s
 */
static inline int16_t mavlink_msg_ladgnss_measurement_block_get_prc_30s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field rrc_30s from ladgnss_measurement_block message
 *
 * @return Pseudorange rate correction (RRC) 30s
 */
static inline int16_t mavlink_msg_ladgnss_measurement_block_get_rrc_30s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field sigma_pr_gnd_100s from ladgnss_measurement_block message
 *
 * @return Sigma_pr_gnd (100s)
 */
static inline uint8_t mavlink_msg_ladgnss_measurement_block_get_sigma_pr_gnd_100s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field sigma_pr_gnd_30s from ladgnss_measurement_block message
 *
 * @return Sigma_pr_gnd (30s)
 */
static inline uint8_t mavlink_msg_ladgnss_measurement_block_get_sigma_pr_gnd_30s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field b1 from ladgnss_measurement_block message
 *
 * @return B1
 */
static inline int8_t mavlink_msg_ladgnss_measurement_block_get_b1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  12);
}

/**
 * @brief Get field b2 from ladgnss_measurement_block message
 *
 * @return B2
 */
static inline int8_t mavlink_msg_ladgnss_measurement_block_get_b2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  13);
}

/**
 * @brief Get field b3 from ladgnss_measurement_block message
 *
 * @return B3
 */
static inline int8_t mavlink_msg_ladgnss_measurement_block_get_b3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  14);
}

/**
 * @brief Get field b4 from ladgnss_measurement_block message
 *
 * @return B4
 */
static inline int8_t mavlink_msg_ladgnss_measurement_block_get_b4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  15);
}

/**
 * @brief Decode a ladgnss_measurement_block message into a struct
 *
 * @param msg The message to decode
 * @param ladgnss_measurement_block C-struct to decode the message contents into
 */
static inline void mavlink_msg_ladgnss_measurement_block_decode(const mavlink_message_t* msg, mavlink_ladgnss_measurement_block_t* ladgnss_measurement_block)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ladgnss_measurement_block->prc_100s = mavlink_msg_ladgnss_measurement_block_get_prc_100s(msg);
    ladgnss_measurement_block->rrc_100s = mavlink_msg_ladgnss_measurement_block_get_rrc_100s(msg);
    ladgnss_measurement_block->prc_30s = mavlink_msg_ladgnss_measurement_block_get_prc_30s(msg);
    ladgnss_measurement_block->rrc_30s = mavlink_msg_ladgnss_measurement_block_get_rrc_30s(msg);
    ladgnss_measurement_block->ranging_source_id = mavlink_msg_ladgnss_measurement_block_get_ranging_source_id(msg);
    ladgnss_measurement_block->iod = mavlink_msg_ladgnss_measurement_block_get_iod(msg);
    ladgnss_measurement_block->sigma_pr_gnd_100s = mavlink_msg_ladgnss_measurement_block_get_sigma_pr_gnd_100s(msg);
    ladgnss_measurement_block->sigma_pr_gnd_30s = mavlink_msg_ladgnss_measurement_block_get_sigma_pr_gnd_30s(msg);
    ladgnss_measurement_block->b1 = mavlink_msg_ladgnss_measurement_block_get_b1(msg);
    ladgnss_measurement_block->b2 = mavlink_msg_ladgnss_measurement_block_get_b2(msg);
    ladgnss_measurement_block->b3 = mavlink_msg_ladgnss_measurement_block_get_b3(msg);
    ladgnss_measurement_block->b4 = mavlink_msg_ladgnss_measurement_block_get_b4(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN? msg->len : MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN;
        memset(ladgnss_measurement_block, 0, MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_LEN);
    memcpy(ladgnss_measurement_block, _MAV_PAYLOAD(msg), len);
#endif
}
