#ifndef TRACKER3D_SPHERE_MONO_H
#define TRACKER3D_SPHERE_MONO_H

#include <xrt/xrt_defines.h>
#include "common/calibration.h"
#include "common/distortion.h"
#include "common/tracked_object.h"
#include "../frameservers/ffmpeg/ffmpeg_frameserver.h"

#define TRACKER_QUEUE_LENGTH 8 //cache the previous mesurements in case we need to replay

typedef struct tracker3D_sphere_mono_instance {
    camera_calibration_t calibration;
    camera_distortion_t distortion;
    //TODO: frameserver API
    ffmpeg_frameserver_instance_t frameserver_instance;
    tracker2D_blue_led_instance_t blob_tracker;

} tracker3D_sphere_mono_instance_t;




bool tracker3D_sphere_mono_create(tracker3D_sphere_mono_descriptor_t* desc);

bool tracker3D_sphere_mono_destroy();

bool tracker3D_sphere_mono_track(frame_t* frame);

bool tracker3D_sphere_mono_get_poses(struct xrt_pose* poses,time_t* pose_times,uint32_t* count);



#endif //TRACKER3D_SPHERE_MONO_H
