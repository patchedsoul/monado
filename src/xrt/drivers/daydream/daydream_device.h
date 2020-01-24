// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to Daydream driver code.
 * @author Pete Black <pete.black@collabora.com>
 * @ingroup drv_daydream
 */

#pragma once

#include "math/m_api.h"
#include "xrt/xrt_device.h"
#include "os/os_threading.h"
#include "os/os_ble.h"

#ifdef __cplusplus
extern "C" {
#endif


/*!
 * A parsed sample of accel and gyro.
 */
struct daydream_parsed_sample
{
    struct xrt_vec3_i32 accel;
    struct xrt_vec3_i32 gyro;
    struct xrt_vec3_i32 mag;
};

/*!
 * A parsed input packet.
 */
struct button_data {
    uint8_t volup :1 ;
    uint8_t voldn :1 ;
    uint8_t app :1 ;
    uint8_t home :1 ;
    uint8_t touchpad :1 ;
};


struct daydream_parsed_input
{
    struct button_data buttons;
    int timestamp;
    uint16_t timestamp_last;
    struct xrt_vec2_i32 touchpad;
    struct daydream_parsed_sample sample;

};

struct daydream_device
{
    struct xrt_device base;
    struct os_ble_device *ble;
    struct os_thread_helper oth;
    char mac[128];

    struct
    {
        //! Lock for last and fusion.
        struct os_mutex lock;

        //! Last sensor read.
        struct daydream_parsed_input last;

        struct
        {
            struct xrt_quat rot;
            struct xrt_vec3 rotvec;
            struct imu_fusion *fusion;
            struct
            {
                struct xrt_vec3 accel;
                struct xrt_vec3 gyro;
                struct xrt_vec3 mag;

            } variance;
        } fusion;
    };

    struct
    {

        struct
        {
            struct xrt_vec3 factor;
            struct xrt_vec3 bias;
        } accel;

        struct
        {
            struct xrt_vec3 factor;
            struct xrt_vec3 bias;
        } gyro;

        struct
        {
            struct xrt_vec3 factor;
            struct xrt_vec3 bias;
        } mag;


    } calibration;

    struct
    {
        //! Last adjusted accelerator value.
        struct xrt_vec3 accel;
        //! Last adjusted gyro value.
        struct xrt_vec3 gyro;
        struct xrt_vec3 mag;

    } read;

    bool print_spew;
    bool print_debug;

    struct
    {
        bool control;
        bool calibration;
        bool last_frame;
        bool fusion;
    } gui;
};


struct daydream_device *
daydream_device_create(
                 bool print_spew,
                 bool print_debug);


static void
daydream_device_destroy(struct xrt_device *xdev);

#define DAYDREAM_SPEW(c, ...)                                                        \
    do {                                                                   \
        if (c->print_spew) {                                           \
            fprintf(stderr, "%s - ", __func__);                    \
            fprintf(stderr, __VA_ARGS__);                          \
            fprintf(stderr, "\n");                                 \
        }                                                              \
    } while (false)

#define DAYDREAM_DEBUG(c, ...)                                                       \
    do {                                                                   \
        if (c->print_debug) {                                          \
            fprintf(stderr, "%s - ", __func__);                    \
            fprintf(stderr, __VA_ARGS__);                          \
            fprintf(stderr, "\n");                                 \
        }                                                              \
    } while (false)

#define DAYDREAM_ERROR(c, ...)                                                       \
    do {                                                                   \
        fprintf(stderr, "%s - ", __func__);                            \
        fprintf(stderr, __VA_ARGS__);                                  \
        fprintf(stderr, "\n");                                         \
    } while (false)


#ifdef __cplusplus
}
#endif

