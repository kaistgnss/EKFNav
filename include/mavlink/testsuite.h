/** @file
 *    @brief MAVLink comm protocol testsuite generated from ladgnss.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef LADGNSS_TESTSUITE_H
#define LADGNSS_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_ladgnss(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_ladgnss(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_ladgnss_common(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LADGNSS_COMMON >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ladgnss_common_t packet_in = {
        17.0,45.0,963497880,963498088,963498296,18275,199,10,77,144,211,22,89,156
    };
    mavlink_ladgnss_common_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.eph_decorrelation_parama = packet_in.eph_decorrelation_parama;
        packet1.sigma_vert_iono_gradient = packet_in.sigma_vert_iono_gradient;
        packet1.latitude = packet_in.latitude;
        packet1.longitude = packet_in.longitude;
        packet1.reference_point_height = packet_in.reference_point_height;
        packet1.modified_z_count = packet_in.modified_z_count;
        packet1.additional_msg_flag = packet_in.additional_msg_flag;
        packet1.n_measurements = packet_in.n_measurements;
        packet1.measurement_type = packet_in.measurement_type;
        packet1.eph_crc = packet_in.eph_crc;
        packet1.source_availability_duration = packet_in.source_availability_duration;
        packet1.refrectivity_index = packet_in.refrectivity_index;
        packet1.scale_height = packet_in.scale_height;
        packet1.refrectivity_uncertainty = packet_in.refrectivity_uncertainty;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LADGNSS_COMMON_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LADGNSS_COMMON_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ladgnss_common_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ladgnss_common_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ladgnss_common_pack(system_id, component_id, &msg , packet1.modified_z_count , packet1.additional_msg_flag , packet1.n_measurements , packet1.measurement_type , packet1.eph_decorrelation_parama , packet1.eph_crc , packet1.source_availability_duration , packet1.sigma_vert_iono_gradient , packet1.refrectivity_index , packet1.scale_height , packet1.refrectivity_uncertainty , packet1.latitude , packet1.longitude , packet1.reference_point_height );
    mavlink_msg_ladgnss_common_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ladgnss_common_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.modified_z_count , packet1.additional_msg_flag , packet1.n_measurements , packet1.measurement_type , packet1.eph_decorrelation_parama , packet1.eph_crc , packet1.source_availability_duration , packet1.sigma_vert_iono_gradient , packet1.refrectivity_index , packet1.scale_height , packet1.refrectivity_uncertainty , packet1.latitude , packet1.longitude , packet1.reference_point_height );
    mavlink_msg_ladgnss_common_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ladgnss_common_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ladgnss_common_send(MAVLINK_COMM_1 , packet1.modified_z_count , packet1.additional_msg_flag , packet1.n_measurements , packet1.measurement_type , packet1.eph_decorrelation_parama , packet1.eph_crc , packet1.source_availability_duration , packet1.sigma_vert_iono_gradient , packet1.refrectivity_index , packet1.scale_height , packet1.refrectivity_uncertainty , packet1.latitude , packet1.longitude , packet1.reference_point_height );
    mavlink_msg_ladgnss_common_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ladgnss_measurement_block(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ladgnss_measurement_block_t packet_in = {
        17235,17339,17443,17547,29,96,163,230,41,108,175,242
    };
    mavlink_ladgnss_measurement_block_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.prc_100s = packet_in.prc_100s;
        packet1.rrc_100s = packet_in.rrc_100s;
        packet1.prc_30s = packet_in.prc_30s;
        packet1.rrc_30s = packet_in.rrc_30s;
        packet1.ranging_source_id = packet_in.ranging_source_id;
        packet1.iod = packet_in.iod;
        packet1.sigma_pr_gnd_100s = packet_in.sigma_pr_gnd_100s;
        packet1.sigma_pr_gnd_30s = packet_in.sigma_pr_gnd_30s;
        packet1.b1 = packet_in.b1;
        packet1.b2 = packet_in.b2;
        packet1.b3 = packet_in.b3;
        packet1.b4 = packet_in.b4;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LADGNSS_MEASUREMENT_BLOCK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ladgnss_measurement_block_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ladgnss_measurement_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ladgnss_measurement_block_pack(system_id, component_id, &msg , packet1.ranging_source_id , packet1.iod , packet1.prc_100s , packet1.rrc_100s , packet1.prc_30s , packet1.rrc_30s , packet1.sigma_pr_gnd_100s , packet1.sigma_pr_gnd_30s , packet1.b1 , packet1.b2 , packet1.b3 , packet1.b4 );
    mavlink_msg_ladgnss_measurement_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ladgnss_measurement_block_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ranging_source_id , packet1.iod , packet1.prc_100s , packet1.rrc_100s , packet1.prc_30s , packet1.rrc_30s , packet1.sigma_pr_gnd_100s , packet1.sigma_pr_gnd_30s , packet1.b1 , packet1.b2 , packet1.b3 , packet1.b4 );
    mavlink_msg_ladgnss_measurement_block_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ladgnss_measurement_block_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ladgnss_measurement_block_send(MAVLINK_COMM_1 , packet1.ranging_source_id , packet1.iod , packet1.prc_100s , packet1.rrc_100s , packet1.prc_30s , packet1.rrc_30s , packet1.sigma_pr_gnd_100s , packet1.sigma_pr_gnd_30s , packet1.b1 , packet1.b2 , packet1.b3 , packet1.b4 );
    mavlink_msg_ladgnss_measurement_block_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ladgnss_transmit_end(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_ladgnss_transmit_end_t packet_in = {
        5
    };
    mavlink_ladgnss_transmit_end_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.reserved = packet_in.reserved;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LADGNSS_TRANSMIT_END_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ladgnss_transmit_end_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_ladgnss_transmit_end_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ladgnss_transmit_end_pack(system_id, component_id, &msg , packet1.reserved );
    mavlink_msg_ladgnss_transmit_end_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ladgnss_transmit_end_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.reserved );
    mavlink_msg_ladgnss_transmit_end_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_ladgnss_transmit_end_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_ladgnss_transmit_end_send(MAVLINK_COMM_1 , packet1.reserved );
    mavlink_msg_ladgnss_transmit_end_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ladgnss(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_ladgnss_common(system_id, component_id, last_msg);
    mavlink_test_ladgnss_measurement_block(system_id, component_id, last_msg);
    mavlink_test_ladgnss_transmit_end(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // LADGNSS_TESTSUITE_H
