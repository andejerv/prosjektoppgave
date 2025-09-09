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

#include "anpp_encode_decode.h"

#include <string.h>

#include "an_packet_protocol.h"

/*
 * This file contains functions to decode and encode packets
 *
 * Decode functions take an an_packet_t and turn it into a type specific
 * to that packet so that the fields can be conveniently accessed. Decode
 * functions return 0 for success and 1 for failure. Decode functions are
 * used when receiving packets.
 *
 * Example decode
 *
 * an_packet_t an_packet
 * acknowledge_packet_t acknowledge_packet
 * ...
 * decode_acknowledge_packet(&acknowledge_packet, &an_packet);
 * printf("acknowledge id %d with result %d\n", acknowledge_packet.packet_id, acknowledge_packet.acknowledge_result);
 *
 * Encode functions take a type specific structure and turn it into an
 * an_packet_t. Encode functions are used when sending packets. Don't
 * forget to free the returned packet with an_packet_free().
 *
 * Example encode
 *
 * an_packet_t* an_packet;
 * boot_mode_packet_t boot_mode_packet;
 * ...
 * boot_mode_packet.boot_mode = boot_mode_bootloader;
 * an_packet = encode_boot_mode_packet(&boot_mode_packet);
 * serial_port_transmit(an_packet_pointer(an_packet), an_packet_size(an_packet));
 * an_packet_free(&an_packet);
 *
 */

an_packet_t* encode_system_state_packet(system_state_packet_t* system_state_packet)
{
    an_packet_t* an_packet = an_packet_allocate(100, packet_id_system_state);
    if (an_packet != NULL) {
        memcpy(&an_packet->data[0], &system_state_packet->system_status, sizeof(uint16_t));
        memcpy(&an_packet->data[2], &system_state_packet->filter_status, sizeof(uint16_t));
        memcpy(&an_packet->data[4], &system_state_packet->unix_time_seconds, sizeof(uint32_t));
        memcpy(&an_packet->data[8], &system_state_packet->microseconds, sizeof(uint32_t));
        memcpy(&an_packet->data[12], &system_state_packet->latitude, sizeof(double));
        memcpy(&an_packet->data[20], &system_state_packet->longitude, sizeof(double));
        memcpy(&an_packet->data[28], &system_state_packet->height, sizeof(double));
        memcpy(&an_packet->data[36], &system_state_packet->velocity[0], 3 * sizeof(float));
        memcpy(&an_packet->data[48], &system_state_packet->body_acceleration[0], 3 * sizeof(float));
        memcpy(&an_packet->data[60], &system_state_packet->g_force, sizeof(float));
        memcpy(&an_packet->data[64], &system_state_packet->orientation[0], 3 * sizeof(float));
        memcpy(&an_packet->data[76], &system_state_packet->angular_velocity[0], 3 * sizeof(float));
        memcpy(&an_packet->data[88], &system_state_packet->standard_deviation[0], 3 * sizeof(float));
    }
    return an_packet;
}

an_packet_t* encode_satellites_packet(satellites_packet_t* satellites_packet)
{
    an_packet_t* an_packet = an_packet_allocate(13, packet_id_satellites);
    if (an_packet != NULL) {
        memcpy(&an_packet->data[0], &satellites_packet->hdop, sizeof(float));
        memcpy(&an_packet->data[4], &satellites_packet->vdop, sizeof(float));
        memcpy(&an_packet->data[8], &satellites_packet->gps_satellites, 5 * sizeof(uint8_t));
    }
    return an_packet;
}

an_packet_t* encode_body_velocity_packet(body_velocity_packet_t* body_velocity_packet)
{
    an_packet_t* an_packet = an_packet_allocate(12, packet_id_body_velocity);
    if (an_packet != NULL) {
        memcpy(&an_packet->data[0], &body_velocity_packet->velocity, 3 * sizeof(float));
    }
    return an_packet;
}
// NOLINTEND