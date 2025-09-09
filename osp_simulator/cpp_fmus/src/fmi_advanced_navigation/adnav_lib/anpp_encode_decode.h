/******************************************************************************/
/*                                                                            */
/*                Advanced Navigation Packet Protocol Library                 */
/*              C Language Dynamic GNSS Compass SDK, Version 6.0              */
/*                    Copyright 2022, Advanced Navigation                     */
/*                                                                            */
/******************************************************************************/
/*
 * Copyright (C) 2022 Advanced Navigation
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

// NOLINTBEGIN

#ifndef ANPP_ENCODE_DECODE_H_
#define ANPP_ENCODE_DECODE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "an_packet_protocol.h"

#define MAXIMUM_DETAILED_SATELLITES 32
#define MAXIMUM_PACKET_PERIODS 50

#define START_SYSTEM_PACKETS 0
#define START_STATE_PACKETS 20
#define START_CONFIGURATION_PACKETS 180

typedef enum {
    packet_id_acknowledge,
    packet_id_request,
    packet_id_boot_mode,
    packet_id_device_information,
    packet_id_restore_factory_settings,
    packet_id_reset,
    packet_id_6_reserved,
    packet_id_file_transfer_request,
    packet_id_file_transfer_acknowledge,
    packet_id_file_transfer,
    packet_id_serial_port_passthrough,
    packet_id_ip_configuration,
    packet_id_12_reserved,
    packet_id_extended_device_information,
    packet_id_subcomponent_information,
    end_system_packets,

    packet_id_system_state = START_STATE_PACKETS,
    packet_id_unix_time,
    packet_id_formatted_time,
    packet_id_status,
    packet_id_position_standard_deviation,
    packet_id_velocity_standard_deviation,
    packet_id_euler_orientation_standard_deviation,
    packet_id_quaternion_orientation_standard_deviation,
    packet_id_raw_sensors,
    packet_id_raw_gnss,
    packet_id_satellites,
    packet_id_satellites_detailed,
    packet_id_geodetic_position,
    packet_id_ecef_position,
    packet_id_utm_position,
    packet_id_ned_velocity,
    packet_id_body_velocity,
    packet_id_acceleration,
    packet_id_body_acceleration,
    packet_id_euler_orientation,
    packet_id_quaternion_orientation,
    packet_id_dcm_orientation,
    packet_id_angular_velocity,
    packet_id_angular_acceleration,
    packet_id_external_position_velocity,
    packet_id_external_position,
    packet_id_external_velocity,
    packet_id_external_body_velocity,
    packet_id_external_heading,
    packet_id_running_time,
    packet_id_local_magnetics,
    packet_id_odometer_state,
    packet_id_external_time,
    packet_id_external_depth,
    packet_id_geoid_height,
    packet_id_rtcm_corrections,
    packet_id_56_reserved,
    packet_id_wind,
    packet_id_heave,
    packet_id_59_reserved,
    packet_id_raw_satellite_data,
    packet_id_raw_satellite_ephemeris,
    packet_id_62_reserved,
    packet_id_63_reserved,
    packet_id_64_reserved,
    packet_id_65_reserved,
    packet_id_66_reserved,
    packet_id_external_odometer,
    packet_id_external_air_data,
    packet_id_gnss_receiver_information,
    packet_id_raw_dvl_data,
    packet_id_north_seeking_status,
    packet_id_gimbal_state,
    packet_id_automotive,
    packet_id_74_reserved,
    packet_id_external_magnetometers,
    packet_id_76_reserved,
    packet_id_77_reserved,
    packet_id_78_reserved,
    packet_id_79_reserved,
    packet_id_basestation,
    packet_id_81_reserved,
    packet_id_82_reserved,
    packet_id_zero_angular_velocity,
    packet_id_extended_satellites,
    packet_id_sensor_temperatures,
    packet_id_system_temperature,
    packet_id_quantum_sensor,
    end_state_packets,

    packet_id_packet_timer_period = START_CONFIGURATION_PACKETS,
    packet_id_packet_periods,
    packet_id_baud_rates,
    packet_id_183_reserved,
    packet_id_sensor_ranges,
    packet_id_installation_alignment,
    packet_id_filter_options,
    packet_id_187_reserved,
    packet_id_gpio_configuration,
    packet_id_magnetic_calibration_values,
    packet_id_magnetic_calibration_configuration,
    packet_id_magnetic_calibration_status,
    packet_id_odometer_configuration,
    packet_id_zero_alignment,
    packet_id_reference_offsets,
    packet_id_gpio_output_configuration,
    packet_id_dual_antenna_configuration,
    packet_id_gnss_configuration,
    packet_id_user_data,
    packet_id_gpio_input_configuration,
    packet_id_200_reserved,
    packet_id_201_reserved,
    packet_id_ip_dataports_configuration,
    packet_id_can_configuration,
    end_configuration_packets
} packet_id_e;

typedef enum {
    gnss_fix_none,
    gnss_fix_2d,
    gnss_fix_3d,
    gnss_fix_sbas,
    gnss_fix_differential,
    gnss_fix_omnistar,
    gnss_fix_rtk_float,
    gnss_fix_rtk_fixed
} gnss_fix_type_e;

typedef struct
{
    union {
        uint16_t r;
        struct
        {
            uint16_t system_failure : 1;
            uint16_t accelerometer_sensor_failure : 1;
            uint16_t gyroscope_sensor_failure : 1;
            uint16_t magnetometer_sensor_failure : 1;
            uint16_t pressure_sensor_failure : 1;
            uint16_t gnss_failure : 1;
            uint16_t accelerometer_over_range : 1;
            uint16_t gyroscope_over_range : 1;
            uint16_t magnetometer_over_range : 1;
            uint16_t pressure_over_range : 1;
            uint16_t minimum_temperature_alarm : 1;
            uint16_t maximum_temperature_alarm : 1;
            uint16_t low_voltage_alarm : 1;
            uint16_t high_voltage_alarm : 1;
            uint16_t gnss_antenna_fault : 1;
            uint16_t serial_port_overflow_alarm : 1;
        } b;
    } system_status;
    union {
        uint16_t r;
        struct
        {
            uint16_t orientation_filter_initialised : 1;
            uint16_t ins_filter_initialised : 1;
            uint16_t heading_initialised : 1;
            uint16_t utc_time_initialised : 1;
            uint16_t gnss_fix_type : 3;
            uint16_t event1_flag : 1;
            uint16_t event2_flag : 1;
            uint16_t internal_gnss_enabled : 1;
            uint16_t dual_antenna_heading_active : 1;
            uint16_t velocity_heading_enabled : 1;
            uint16_t atmospheric_altitude_enabled : 1;
            uint16_t external_position_active : 1;
            uint16_t external_velocity_active : 1;
            uint16_t external_heading_active : 1;
        } b;
    } filter_status;
    uint32_t unix_time_seconds;
    uint32_t microseconds;
    double latitude;
    double longitude;
    double height;
    float velocity[3];
    float body_acceleration[3];
    float g_force;
    float orientation[3];
    float angular_velocity[3];
    float standard_deviation[3];
} system_state_packet_t;

typedef struct
{
    float hdop;
    float vdop;
    uint8_t gps_satellites;
    uint8_t glonass_satellites;
    uint8_t beidou_satellites;
    uint8_t galileo_satellites;
    uint8_t sbas_satellites;
} satellites_packet_t;

typedef struct
{
    float velocity[3];
} body_velocity_packet_t;

an_packet_t* encode_system_state_packet(system_state_packet_t* system_state_packet);
an_packet_t* encode_satellites_packet(satellites_packet_t* satellites_packet);
an_packet_t* encode_body_velocity_packet(body_velocity_packet_t* body_velocity_packet);

#ifdef __cplusplus
}
#endif

#endif

// NOLINTEND