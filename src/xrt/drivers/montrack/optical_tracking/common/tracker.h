#ifndef TRACKER_H
#define TRACKER_H
#include <xrt/xrt_defines.h>
#include "../frameservers/common/frameserver.h"
#include "calibration.h"
#include "tracked_object.h"
#include <sys/socket.h>
#include <netinet/in.h>
#define MAX_FRAMESERVERS                                                       \
	8 // maximum number of cameras/sources that can be bound to a tracker

#ifdef __cplusplus
extern "C" {
#endif

typedef enum tracker_calibration_mode
{
	CALIBRATION_MODE_NONE,
	CALIBRATION_MODE_CHESSBOARD
} tracker_calibration_mode_t;
typedef enum tracker_event_desc
{
	TRACKER_EVENT_NONE,
	TRACKER_EVENT_RECONFIGURED
} tracker_event_desc_t;

typedef struct tracker_measurement
{
	struct xrt_pose pose;
	bool has_position;
	bool has_rotation;
	timepoint_ns timestamp;
} tracker_measurement_t;

typedef enum tracker_type
{
	TRACKER_TYPE_NONE,
	TRACKER_TYPE_SPHERE_STEREO,
	TRACKER_TYPE_SPHERE_MONO,
	TRACKER_TYPE_OSVR_UVBI
} tracker_type_t;

typedef struct tracker_event
{
	tracker_type_t type;
	tracker_event_desc_t event;
} tracker_event_t;

typedef void* tracker_instance_ptr;
typedef void* tracker_internal_instance_ptr;
typedef void* tracker_configuration_ptr;

typedef void (*measurement_consumer_callback_func)(
    void* instance, tracker_measurement_t* measurement);

typedef struct tracker_mono_configuration
{
	char configuration_filename[256]; // TODO: maybe too small?
	tracker_calibration_mode_t calibration_mode;
	// camera_calibration_t calibration;
	frame_format_t format;
	uint64_t source_id;
} tracker_mono_configuration_t;

typedef struct tracker_stereo_configuration
{
	char configuration_filename[256]; // TODO: maybe too small?
	tracker_calibration_mode_t calibration_mode;
	// camera_calibration_t l_calibration;
	// camera_calibration_t r_calibration;
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
