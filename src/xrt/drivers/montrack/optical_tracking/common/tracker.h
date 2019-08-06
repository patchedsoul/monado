#ifndef TRACKER_H
#define TRACKER_H
#include <xrt/xrt_defines.h>
#include <frameserver.h>
#include "tracked_object.h"
#include <sys/socket.h>
#include <netinet/in.h>
#define MAX_FRAMESERVERS                                                       \
	8 // maximum number of cameras/sources that can be bound to a tracker

#ifdef __cplusplus
extern "C" {
#endif

typedef enum tracker_measurement_flags {MEASUREMENT_NONE=0,
	                                    MEASUREMENT_POSITION=1,
	                                    MEASUREMENT_ROTATION=2,
	                                    MEASUREMENT_RAW_ACCEL=4,
	                                    MEASUREMENT_RAW_GYRO=8,
	                                    MEASUREMENT_RAW_MAG=16,
	                                    MEASUREMENT_OPTICAL=32,
	                                    MEASUREMENT_IMU=64,
                                       } tracker_measurement_flags_t;
typedef struct tracker_measurement
{
	struct xrt_pose pose;
	struct xrt_vec3 accel;
	struct xrt_vec3 gyro;
	struct xrt_vec3 mag;
	tracker_measurement_flags_t flags;
    int64_t source_timestamp;
    uint8_t source_id;
    uint64_t source_sequence;
} tracker_measurement_t;

typedef enum tracker_type
{
	TRACKER_TYPE_NONE,
	TRACKER_TYPE_SPHERE_STEREO,
	TRACKER_TYPE_SPHERE_MONO,
    TRACKER_TYPE_UVBI,
    TRACKER_TYPE_CALIBRATION_STEREO,
    TRACKER_TYPE_CALIBRATION_MONO,
    TRACKER_TYPE_PSVR_STEREO,
} tracker_type_t;

typedef enum tracker_source_type {
    TRACKER_SOURCE_TYPE_NONE,
    TRACKER_SOURCE_TYPE_RECTILINEAR,
    TRACKER_SOURCE_TYPE_FISHEYE
} tracker_source_type_t;



struct _tracker_instance;

typedef struct _tracker_instance* tracker_instance_ptr;
typedef void* tracker_internal_instance_ptr;
typedef void* tracker_configuration_ptr;

typedef void (*measurement_consumer_callback_func)(
    void* instance, tracker_measurement_t* measurement);

typedef struct tracker_mono_configuration
{
    tracker_source_type_t source_type;
    char camera_configuration_filename[256]; // TODO: maybe too small?
    char room_setup_filename[256];
    frame_format_t format;
	uint64_t source_id;
} tracker_mono_configuration_t;

typedef struct tracker_stereo_configuration
{
    tracker_source_type_t source_type;
    char camera_configuration_filename[256]; // TODO: maybe too small?
    char room_setup_filename[256];
    frame_format_t l_format;
	uint64_t l_source_id;
	frame_format_t r_format;
	uint64_t r_source_id;
	bool split_left; // single-frame stereo will split the left frame
	frame_rect_t l_rect;
	frame_rect_t r_rect;
} tracker_stereo_configuration_t;



// tracker interface

typedef struct _tracker_instance
{
	tracker_type_t tracker_type;
	capture_parameters_t (*tracker_get_capture_params)(
	    tracker_instance_ptr inst);
	bool (*tracker_queue)(tracker_instance_ptr inst, frame_t* frame);
	bool (*tracker_get_debug_frame)(tracker_instance_ptr inst,
	                                frame_t* frame);
	bool (*tracker_get_poses)(tracker_instance_ptr inst,
	                          tracked_object_t* tracked_objects,
	                          uint32_t* count);
	bool (*tracker_has_new_poses)(tracker_instance_ptr inst);
	void (*tracker_register_measurement_callback)(
	    tracker_instance_ptr inst,
	    void* target_instance,
	    measurement_consumer_callback_func target_func);
	void (*tracker_register_event_callback)(
	    tracker_instance_ptr inst,
	    void* target_instance,
	    event_consumer_callback_func target_func);
	bool (*tracker_configure)(tracker_instance_ptr inst,
	                          tracker_configuration_ptr config);
	tracker_internal_instance_ptr internal_instance;
	int debug_fd, debug_socket, socket_read;
	int debug_client_fd;
	bool client_connected;
    bool configured;
	struct sockaddr_in debug_address;
} tracker_instance_t;

tracker_instance_t*
tracker_create(tracker_type_t t);
bool
tracker_destroy(tracker_instance_t* inst);
bool
tracker_send_debug_frame(tracker_instance_t* inst);

bool
trackers_test();

#ifdef __cplusplus
}
#endif

#endif // TRACKER_H
