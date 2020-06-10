#pragma once
// MESSAGE LADGNSS_COMMON PACKING

#define MAVLINK_MSG_ID_LADGNSS_COMMON 1

MAVPACKED(
typedef struct __mavlink_ladgnss_common_t {
 float latitude; /*< Latitude*/
 float longitude; /*< Longitude*/
 float reference_point_height; /*< Reference point height*/
 float modified_z_count; /*< Modified Z-count (14 bits)*/
 uint8_t additional_msg_flag; /*< Additional message flag (2 bits)*/
 uint8_t n_measurements; /*< Number of measurements (5 bits)*/
 uint8_t measurement_type; /*< Measurement type (3 bits)*/
 float eph_decorrelation_parama; /*< Ephemeris decorrelation parameter*/
 uint8_t eph_crc; /*< Ephemeris CRC*/
 uint8_t source_availability_duration; /*< Source availability duration*/
 float sigma_vert_iono_gradient; /*< Sigma_vert_iono_gradient*/
 uint8_t refrectivity_index; /*< Refrectivity index*/
 uint8_t scale_height; /*< Scale height*/
 uint8_t refrectivity_uncertainty; /*< Refractivity uncertainty*/
}) mavlink_ladgnss_common_t;

#define MAVLINK_MSG_ID_LADGNSS_COMMON_LEN 32
#define MAVLINK_MSG_ID_LADGNSS_COMMON_MIN_LEN 32
#define MAVLINK_MSG_ID_1_LEN 32
#define MAVLINK_MSG_ID_1_MIN_LEN 32

#define MAVLINK_MSG_ID_LADGNSS_COMMON_CRC 320
#define MAVLINK_MSG_ID_1_CRC 320



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LADGNSS_COMMON { \
    1, \
    "LADGNSS_COMMON", \
    14, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ladgnss_common_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ladgnss_common_t, longitude) }, \
         { "reference_point_height", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ladgnss_common_t, reference_point_height) }, \
         { "modified_z_count", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ladgnss_common_t, modified_z_count) }, \
         { "additional_msg_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_ladgnss_common_t, additional_msg_flag) }, \
         { "n_measurements", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_ladgnss_common_t, n_measurements) }, \
         { "measurement_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_ladgnss_common_t, measurement_type) }, \
         { "eph_decorrelation_parama", NULL, MAVLINK_TYPE_FLOAT, 0, 19, offsetof(mavlink_ladgnss_common_t, eph_decorrelation_parama) }, \
         { "eph_crc", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_ladgnss_common_t, eph_crc) }, \
         { "source_availability_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_ladgnss_common_t, source_availability_duration) }, \
         { "sigma_vert_iono_gradient", NULL, MAVLINK_TYPE_FLOAT, 0, 25, offsetof(mavlink_ladgnss_common_t, sigma_vert_iono_gradient) }, \
         { "refrectivity_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_ladgnss_common_t, refrectivity_index) }, \
         { "scale_height", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_ladgnss_common_t, scale_height) }, \
         { "refrectivity_uncertainty", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_ladgnss_common_t, refrectivity_uncertainty) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LADGNSS_COMMON { \
    "LADGNSS_COMMON", \
    14, \
    {  { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ladgnss_common_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ladgnss_common_t, longitude) }, \
         { "reference_point_height", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_ladgnss_common_t, reference_point_height) }, \
         { "modified_z_count", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_ladgnss_common_t, modified_z_count) }, \
         { "additional_msg_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_ladgnss_common_t, additional_msg_flag) }, \
         { "n_measurements", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_ladgnss_common_t, n_measurements) }, \
         { "measurement_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_ladgnss_common_t, measurement_type) }, \
         { "eph_decorrelation_parama", NULL, MAVLINK_TYPE_FLOAT, 0, 19, offsetof(mavlink_ladgnss_common_t, eph_decorrelation_parama) }, \
         { "eph_crc", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_ladgnss_common_t, eph_crc) }, \
         { "source_availability_duration", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_ladgnss_common_t, source_availability_duration) }, \
         { "sigma_vert_iono_gradient", NULL, MAVLINK_TYPE_FLOAT, 0, 25, offsetof(mavlink_ladgnss_common_t, sigma_vert_iono_gradient) }, \
         { "refrectivity_index", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_ladgnss_common_t, refrectivity_index) }, \
         { "scale_height", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_ladgnss_common_t, scale_height) }, \
         { "refrectivity_uncertainty", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_ladgnss_common_t, refrectivity_uncertainty) }, \
         } \
}
#endif

/**
 * @brief Pack a ladgnss_common message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param modified_z_count Modified Z-count (14 bits)
 * @param additional_msg_flag Additional message flag (2 bits)
 * @param n_measurements Number of measurements (5 bits)
 * @param measurement_type Measurement type (3 bits)
 * @param eph_decorrelation_parama Ephemeris decorrelation parameter
 * @param eph_crc Ephemeris CRC
 * @param source_availability_duration Source availability duration
 * @param sigma_vert_iono_gradient Sigma_vert_iono_gradient
 * @param refrectivity_index Refrectivity index
 * @param scale_height Scale height
 * @param refrectivity_uncertainty Refractivity uncertainty
 * @param latitude Latitude
 * @param longitude Longitude
 * @param reference_point_height Reference point height
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ladgnss_common_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float modified_z_count, uint8_t additional_msg_flag, uint8_t n_measurements,
							   uint8_t measurement_type, float eph_decorrelation_parama, uint8_t eph_crc,
							   uint8_t source_availability_duration, float sigma_vert_iono_gradient,
							   uint8_t refrectivity_index, uint8_t scale_height, uint8_t refrectivity_uncertainty,
							   float latitude, float longitude, float reference_point_height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LADGNSS_COMMON_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, reference_point_height);
    _mav_put_float(buf, 12, modified_z_count);
    _mav_put_uint8_t(buf, 16, additional_msg_flag);
    _mav_put_uint8_t(buf, 17, n_measurements);
    _mav_put_uint8_t(buf, 18, measurement_type);
    _mav_put_float(buf, 19, eph_decorrelation_parama);
    _mav_put_uint8_t(buf, 23, eph_crc);
    _mav_put_uint8_t(buf, 24, source_availability_duration);
    _mav_put_float(buf, 25, sigma_vert_iono_gradient);
    _mav_put_uint8_t(buf, 29, refrectivity_index);
    _mav_put_uint8_t(buf, 30, scale_height);
    _mav_put_uint8_t(buf, 31, refrectivity_uncertainty);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LADGNSS_COMMON_LEN);
#else
    mavlink_ladgnss_common_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.reference_point_height = reference_point_height;
    packet.modified_z_count = modified_z_count;
    packet.additional_msg_flag = additional_msg_flag;
    packet.n_measurements = n_measurements;
    packet.measurement_type = measurement_type;
    packet.eph_decorrelation_parama = eph_decorrelation_parama;
    packet.eph_crc = eph_crc;
    packet.source_availability_duration = source_availability_duration;
    packet.sigma_vert_iono_gradient = sigma_vert_iono_gradient;
    packet.refrectivity_index = refrectivity_index;
    packet.scale_height = scale_height;
    packet.refrectivity_uncertainty = refrectivity_uncertainty;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LADGNSS_COMMON_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LADGNSS_COMMON;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LADGNSS_COMMON_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_CRC);
}

/**
 * @brief Pack a ladgnss_common message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param modified_z_count Modified Z-count (14 bits)
 * @param additional_msg_flag Additional message flag (2 bits)
 * @param n_measurements Number of measurements (5 bits)
 * @param measurement_type Measurement type (3 bits)
 * @param eph_decorrelation_parama Ephemeris decorrelation parameter
 * @param eph_crc Ephemeris CRC
 * @param source_availability_duration Source availability duration
 * @param sigma_vert_iono_gradient Sigma_vert_iono_gradient
 * @param refrectivity_index Refrectivity index
 * @param scale_height Scale height
 * @param refrectivity_uncertainty Refractivity uncertainty
 * @param latitude Latitude
 * @param longitude Longitude
 * @param reference_point_height Reference point height
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ladgnss_common_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float modified_z_count,uint8_t additional_msg_flag,uint8_t n_measurements,
								   uint8_t measurement_type,float eph_decorrelation_parama,uint8_t eph_crc,
								   uint8_t source_availability_duration,float sigma_vert_iono_gradient,uint8_t refrectivity_index,
								   uint8_t scale_height,uint8_t refrectivity_uncertainty,
								   float latitude,float longitude,float reference_point_height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LADGNSS_COMMON_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, reference_point_height);
    _mav_put_float(buf, 12, modified_z_count);
    _mav_put_uint8_t(buf, 16, additional_msg_flag);
    _mav_put_uint8_t(buf, 17, n_measurements);
    _mav_put_uint8_t(buf, 18, measurement_type);
    _mav_put_float(buf, 19, eph_decorrelation_parama);
    _mav_put_uint8_t(buf, 23, eph_crc);
    _mav_put_uint8_t(buf, 24, source_availability_duration);
    _mav_put_float(buf, 25, sigma_vert_iono_gradient);
    _mav_put_uint8_t(buf, 29, refrectivity_index);
    _mav_put_uint8_t(buf, 30, scale_height);
    _mav_put_uint8_t(buf, 31, refrectivity_uncertainty);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LADGNSS_COMMON_LEN);
#else
    mavlink_ladgnss_common_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.reference_point_height = reference_point_height;
    packet.modified_z_count = modified_z_count;
    packet.additional_msg_flag = additional_msg_flag;
    packet.n_measurements = n_measurements;
    packet.measurement_type = measurement_type;
    packet.eph_decorrelation_parama = eph_decorrelation_parama;
    packet.eph_crc = eph_crc;
    packet.source_availability_duration = source_availability_duration;
    packet.sigma_vert_iono_gradient = sigma_vert_iono_gradient;
    packet.refrectivity_index = refrectivity_index;
    packet.scale_height = scale_height;
    packet.refrectivity_uncertainty = refrectivity_uncertainty;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LADGNSS_COMMON_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_LADGNSS_COMMON;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LADGNSS_COMMON_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_CRC);
}

/**
 * @brief Encode a ladgnss_common struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ladgnss_common C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ladgnss_common_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ladgnss_common_t* ladgnss_common)
{
    return mavlink_msg_ladgnss_common_pack(system_id, component_id, msg, ladgnss_common->modified_z_count, ladgnss_common->additional_msg_flag, ladgnss_common->n_measurements, ladgnss_common->measurement_type, ladgnss_common->eph_decorrelation_parama, ladgnss_common->eph_crc, ladgnss_common->source_availability_duration, ladgnss_common->sigma_vert_iono_gradient, ladgnss_common->refrectivity_index, ladgnss_common->scale_height, ladgnss_common->refrectivity_uncertainty, ladgnss_common->latitude, ladgnss_common->longitude, ladgnss_common->reference_point_height);
}

/**
 * @brief Encode a ladgnss_common struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ladgnss_common C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ladgnss_common_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ladgnss_common_t* ladgnss_common)
{
    return mavlink_msg_ladgnss_common_pack_chan(system_id, component_id, chan, msg, ladgnss_common->modified_z_count, ladgnss_common->additional_msg_flag, ladgnss_common->n_measurements, ladgnss_common->measurement_type, ladgnss_common->eph_decorrelation_parama, ladgnss_common->eph_crc, ladgnss_common->source_availability_duration, ladgnss_common->sigma_vert_iono_gradient, ladgnss_common->refrectivity_index, ladgnss_common->scale_height, ladgnss_common->refrectivity_uncertainty, ladgnss_common->latitude, ladgnss_common->longitude, ladgnss_common->reference_point_height);
}

/**
 * @brief Send a ladgnss_common message
 * @param chan MAVLink channel to send the message
 *
 * @param modified_z_count Modified Z-count (14 bits)
 * @param additional_msg_flag Additional message flag (2 bits)
 * @param n_measurements Number of measurements (5 bits)
 * @param measurement_type Measurement type (3 bits)
 * @param eph_decorrelation_parama Ephemeris decorrelation parameter
 * @param eph_crc Ephemeris CRC
 * @param source_availability_duration Source availability duration
 * @param sigma_vert_iono_gradient Sigma_vert_iono_gradient
 * @param refrectivity_index Refrectivity index
 * @param scale_height Scale height
 * @param refrectivity_uncertainty Refractivity uncertainty
 * @param latitude Latitude
 * @param longitude Longitude
 * @param reference_point_height Reference point height
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ladgnss_common_send(mavlink_channel_t chan,
		float modified_z_count, uint8_t additional_msg_flag, uint8_t n_measurements, uint8_t measurement_type,
		float eph_decorrelation_parama, uint8_t eph_crc, uint8_t source_availability_duration,
		float sigma_vert_iono_gradient, uint8_t refrectivity_index, uint8_t scale_height, uint8_t refrectivity_uncertainty,
		float latitude, float longitude, float reference_point_height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_LADGNSS_COMMON_LEN];
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, reference_point_height);
    _mav_put_float(buf, 12, modified_z_count);
    _mav_put_uint8_t(buf, 16, additional_msg_flag);
    _mav_put_uint8_t(buf, 17, n_measurements);
    _mav_put_uint8_t(buf, 18, measurement_type);
    _mav_put_float(buf, 19, eph_decorrelation_parama);
    _mav_put_uint8_t(buf, 23, eph_crc);
    _mav_put_uint8_t(buf, 24, source_availability_duration);
    _mav_put_float(buf, 25, sigma_vert_iono_gradient);
    _mav_put_uint8_t(buf, 29, refrectivity_index);
    _mav_put_uint8_t(buf, 30, scale_height);
    _mav_put_uint8_t(buf, 31, refrectivity_uncertainty);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_COMMON, buf, MAVLINK_MSG_ID_LADGNSS_COMMON_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_CRC);
#else
    mavlink_ladgnss_common_t packet;
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.reference_point_height = reference_point_height;
    packet.modified_z_count = modified_z_count;
    packet.additional_msg_flag = additional_msg_flag;
    packet.n_measurements = n_measurements;
    packet.measurement_type = measurement_type;
    packet.eph_decorrelation_parama = eph_decorrelation_parama;
    packet.eph_crc = eph_crc;
    packet.source_availability_duration = source_availability_duration;
    packet.sigma_vert_iono_gradient = sigma_vert_iono_gradient;
    packet.refrectivity_index = refrectivity_index;
    packet.scale_height = scale_height;
    packet.refrectivity_uncertainty = refrectivity_uncertainty;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_COMMON, (const char *)&packet, MAVLINK_MSG_ID_LADGNSS_COMMON_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_CRC);
#endif
}

/**
 * @brief Send a ladgnss_common message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ladgnss_common_send_struct(mavlink_channel_t chan, const mavlink_ladgnss_common_t* ladgnss_common)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ladgnss_common_send(chan, ladgnss_common->modified_z_count, ladgnss_common->additional_msg_flag, ladgnss_common->n_measurements, ladgnss_common->measurement_type, ladgnss_common->eph_decorrelation_parama, ladgnss_common->eph_crc, ladgnss_common->source_availability_duration, ladgnss_common->sigma_vert_iono_gradient, ladgnss_common->refrectivity_index, ladgnss_common->scale_height, ladgnss_common->refrectivity_uncertainty, ladgnss_common->latitude, ladgnss_common->longitude, ladgnss_common->reference_point_height);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_COMMON, (const char *)ladgnss_common, MAVLINK_MSG_ID_LADGNSS_COMMON_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_CRC);
#endif
}

#if MAVLINK_MSG_ID_LADGNSS_COMMON_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ladgnss_common_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,
		float modified_z_count, uint8_t additional_msg_flag, uint8_t n_measurements, uint8_t measurement_type,
		float eph_decorrelation_parama, uint8_t eph_crc, uint8_t source_availability_duration,
		float sigma_vert_iono_gradient, uint8_t refrectivity_index, uint8_t scale_height, uint8_t refrectivity_uncertainty,
		float latitude, float longitude, float reference_point_height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, latitude);
    _mav_put_float(buf, 4, longitude);
    _mav_put_float(buf, 8, reference_point_height);
    _mav_put_float(buf, 12, modified_z_count);
    _mav_put_uint8_t(buf, 16, additional_msg_flag);
    _mav_put_uint8_t(buf, 17, n_measurements);
    _mav_put_uint8_t(buf, 18, measurement_type);
    _mav_put_float(buf, 19, eph_decorrelation_parama);
    _mav_put_uint8_t(buf, 23, eph_crc);
    _mav_put_uint8_t(buf, 24, source_availability_duration);
    _mav_put_float(buf, 25, sigma_vert_iono_gradient);
    _mav_put_uint8_t(buf, 29, refrectivity_index);
    _mav_put_uint8_t(buf, 30, scale_height);
    _mav_put_uint8_t(buf, 31, refrectivity_uncertainty);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_COMMON, buf, MAVLINK_MSG_ID_LADGNSS_COMMON_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_CRC);
#else
    mavlink_ladgnss_common_t *packet = (mavlink_ladgnss_common_t *)msgbuf;
    packet->latitude = latitude;
    packet->longitude = longitude;
    packet->reference_point_height = reference_point_height;
    packet->modified_z_count = modified_z_count;
    packet->additional_msg_flag = additional_msg_flag;
    packet->n_measurements = n_measurements;
    packet->measurement_type = measurement_type;
    packet->eph_decorrelation_parama = eph_decorrelation_parama;
    packet->eph_crc = eph_crc;
    packet->source_availability_duration = source_availability_duration;
    packet->sigma_vert_iono_gradient = sigma_vert_iono_gradient;
    packet->refrectivity_index = refrectivity_index;
    packet->scale_height = scale_height;
    packet->refrectivity_uncertainty = refrectivity_uncertainty;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LADGNSS_COMMON, (const char *)packet, MAVLINK_MSG_ID_LADGNSS_COMMON_MIN_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_LEN, MAVLINK_MSG_ID_LADGNSS_COMMON_CRC);
#endif
}
#endif

#endif

// MESSAGE LADGNSS_COMMON UNPACKING


/**
 * @brief Get field modified_z_count from ladgnss_common message
 *
 * @return Modified Z-count (14 bits)
 */
static inline float mavlink_msg_ladgnss_common_get_modified_z_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field additional_msg_flag from ladgnss_common message
 *
 * @return Additional message flag (2 bits)
 */
static inline uint8_t mavlink_msg_ladgnss_common_get_additional_msg_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field n_measurements from ladgnss_common message
 *
 * @return Number of measurements (5 bits)
 */
static inline uint8_t mavlink_msg_ladgnss_common_get_n_measurements(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field measurement_type from ladgnss_common message
 *
 * @return Measurement type (3 bits)
 */
static inline uint8_t mavlink_msg_ladgnss_common_get_measurement_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field eph_decorrelation_parama from ladgnss_common message
 *
 * @return Ephemeris decorrelation parameter
 */
static inline float mavlink_msg_ladgnss_common_get_eph_decorrelation_parama(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  19);
}

/**
 * @brief Get field eph_crc from ladgnss_common message
 *
 * @return Ephemeris CRC
 */
static inline uint8_t mavlink_msg_ladgnss_common_get_eph_crc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field source_availability_duration from ladgnss_common message
 *
 * @return Source availability duration
 */
static inline uint8_t mavlink_msg_ladgnss_common_get_source_availability_duration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field sigma_vert_iono_gradient from ladgnss_common message
 *
 * @return Sigma_vert_iono_gradient
 */
static inline float mavlink_msg_ladgnss_common_get_sigma_vert_iono_gradient(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  25);
}

/**
 * @brief Get field refrectivity_index from ladgnss_common message
 *
 * @return Refrectivity index
 */
static inline uint8_t mavlink_msg_ladgnss_common_get_refrectivity_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field scale_height from ladgnss_common message
 *
 * @return Scale height
 */
static inline uint8_t mavlink_msg_ladgnss_common_get_scale_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field refrectivity_uncertainty from ladgnss_common message
 *
 * @return Refractivity uncertainty
 */
static inline uint8_t mavlink_msg_ladgnss_common_get_refrectivity_uncertainty(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field latitude from ladgnss_common message
 *
 * @return Latitude
 */
static inline float mavlink_msg_ladgnss_common_get_latitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field longitude from ladgnss_common message
 *
 * @return Longitude
 */
static inline float mavlink_msg_ladgnss_common_get_longitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field reference_point_height from ladgnss_common message
 *
 * @return Reference point height
 */
static inline float mavlink_msg_ladgnss_common_get_reference_point_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a ladgnss_common message into a struct
 *
 * @param msg The message to decode
 * @param ladgnss_common C-struct to decode the message contents into
 */
static inline void mavlink_msg_ladgnss_common_decode(const mavlink_message_t* msg, mavlink_ladgnss_common_t* ladgnss_common)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ladgnss_common->latitude = mavlink_msg_ladgnss_common_get_latitude(msg);
    ladgnss_common->longitude = mavlink_msg_ladgnss_common_get_longitude(msg);
    ladgnss_common->reference_point_height = mavlink_msg_ladgnss_common_get_reference_point_height(msg);
    ladgnss_common->modified_z_count = mavlink_msg_ladgnss_common_get_modified_z_count(msg);
    ladgnss_common->additional_msg_flag = mavlink_msg_ladgnss_common_get_additional_msg_flag(msg);
    ladgnss_common->n_measurements = mavlink_msg_ladgnss_common_get_n_measurements(msg);
    ladgnss_common->measurement_type = mavlink_msg_ladgnss_common_get_measurement_type(msg);
    ladgnss_common->eph_decorrelation_parama = mavlink_msg_ladgnss_common_get_eph_decorrelation_parama(msg);
    ladgnss_common->eph_crc = mavlink_msg_ladgnss_common_get_eph_crc(msg);
    ladgnss_common->source_availability_duration = mavlink_msg_ladgnss_common_get_source_availability_duration(msg);
    ladgnss_common->sigma_vert_iono_gradient = mavlink_msg_ladgnss_common_get_sigma_vert_iono_gradient(msg);
    ladgnss_common->refrectivity_index = mavlink_msg_ladgnss_common_get_refrectivity_index(msg);
    ladgnss_common->scale_height = mavlink_msg_ladgnss_common_get_scale_height(msg);
    ladgnss_common->refrectivity_uncertainty = mavlink_msg_ladgnss_common_get_refrectivity_uncertainty(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LADGNSS_COMMON_LEN? msg->len : MAVLINK_MSG_ID_LADGNSS_COMMON_LEN;
        memset(ladgnss_common, 0, MAVLINK_MSG_ID_LADGNSS_COMMON_LEN);
    memcpy(ladgnss_common, _MAV_PAYLOAD(msg), len);
#endif
}
