#include "tracker.h"
#include "tracker3D_sphere_mono.h"
#include "../frameservers/uvc/uvc_frameserver.h"
#include <string.h>

tracker_instance_t* tracker_create(tracker_type_t t) {
	tracker_instance_t* i = calloc(1,sizeof(tracker_instance_t));
	if (i) {
		switch (t) {
		    case TRACKER_TYPE_SPHERE_MONO:
			    i->tracker_type = t;
				i->internal_instance = tracker3D_sphere_mono_create(i);
				i->tracker_get_capture_params = tracker3D_sphere_mono_get_capture_params;
				i->tracker_get_poses = tracker3D_sphere_mono_get_poses;
				i->tracker_get_debug_frame= tracker3D_sphere_mono_get_debug_frame;
				i->tracker_queue = tracker3D_sphere_mono_queue;
				i->tracker_register_measurement_callback = tracker3D_sphere_mono_register_measurement_callback;
				i->tracker_has_new_poses = tracker3D_sphere_mono_new_poses;
				i->tracker_configure = tracker3D_sphere_mono_configure;
			    break;
		    case TRACKER_TYPE_NONE:
		    default:
			    free(i);
			    return NULL;
			break;
		}
		return i;
	}
	return NULL;
}

bool trackers_test(){

	//create a tracker
	tracker_instance_t* tracker = tracker_create(TRACKER_TYPE_SPHERE_MONO);
	if (! tracker)
	{
		return false;
	}
	//how many objects does this tracker support? (this is a maximum)
	uint32_t tracked_object_count=0;
	tracker->tracker_get_poses(tracker,NULL,&tracked_object_count);
	tracked_object_t* tracked_objects = NULL;
	if (tracked_object_count > 0)
	{
		tracked_objects = calloc(tracked_object_count,sizeof(tracked_object_t));
	}

	//create our frameserver, that will feed our tracker
	frameserver_instance_t* frame_source = frameserver_create(FRAMESERVER_TYPE_UVC);

	//ask our frameserver for available sources - note this will return a frameserver-specific struct
	//that we need to deal with.
	uint32_t source_count=0;
	frame_source->frameserver_enumerate_sources(frame_source,NULL,&source_count);
	if (source_count == 0){
		return false;
	}
	uvc_source_descriptor_t* descriptors = calloc(source_count,sizeof(uvc_source_descriptor_t));

	// TODO: we will currently leak calloced strings that are bound to the descriptors. we need to clean
	// this memory up.
	frame_source->frameserver_enumerate_sources(frame_source,descriptors,&source_count);

	tracker_mono_configuration_t tracker_config = {};

	//select a frame source that is acceptable to our tracker
	//normally we would have a clear idea which source to choose (i.e. camera ids, resolution, format)

	bool configured = false;
	uint32_t highest_width = 0;
	uint32_t highest_height = 0;
	uint32_t best_index = 0;
	uint32_t best_uncompressed=0;
	uint32_t highest_u_width = 0;
	uint32_t highest_u_height = 0;
	uint32_t uncompressed_count =0;
	for (uint32_t i=0; i< source_count;i++){
		//TODO: is there an existing camera descriptor bound
		// to this tracker in the config? if so try and use it

		//otherwise, try and configure the tracker for the highest width/height
		//available on any camera source. we probably need to filter on framerate too

		tracker_config.format = descriptors[i].format;
		tracker_config.source_id =descriptors[i].source_id;

		if (tracker->tracker_configure(tracker,&tracker_config)){ //our tracker accepts this
			configured=true;
			if (descriptors[i].stream_format == UVC_FRAME_FORMAT_YUYV && descriptors[i].width >= highest_u_width && descriptors[i].height > highest_u_height) {
				highest_u_width = descriptors[i].width;
				highest_u_height = descriptors[i].height;
				best_uncompressed=i;
				uncompressed_count ++;
			}

			if (descriptors[i].width >= highest_width && descriptors[i].height > highest_height) {
				highest_width = descriptors[i].width;
				highest_height = descriptors[i].height;
				best_index=i;
			}
		}
	}
	tracker_config.format = descriptors[best_index].format;
	tracker_config.source_id = descriptors[best_index].source_id;
	uint32_t source_index = best_index;
	if (uncompressed_count > 0) {
	//prefer the best uncompressed mode
		tracker_config.format = descriptors[best_uncompressed].format;
		tracker_config.source_id =descriptors[best_uncompressed].source_id;
		source_index = best_uncompressed;
	}

	// look up our calibration from some as-yet undefined data source
	// using the selected source data.
	// currently hardcoded to a camera #define in calibration.h
	float camera_size[2] = LOGITECH_C270_SIZE;
	float camera_intr[INTRINSICS_SIZE] = LOGITECH_C270_INTR;
	float camera_dist[DISTORTION_SIZE] = LOGITECH_C270_DIST;

	tracker_config.calibration.calib_size[0]=camera_size[0];
	tracker_config.calibration.calib_size[1]=camera_size[1];
	memcpy(tracker_config.calibration.intrinsics,camera_intr,sizeof(tracker_config.calibration.intrinsics));
	memcpy(tracker_config.calibration.distortion,camera_dist,sizeof(tracker_config.calibration.distortion));


	configured = tracker->tracker_configure(tracker,&tracker_config);

	if (! configured)
	{
		printf("ERROR: no compatible source for tracker from frameserver\n");
		return false;
	}

	//bind our frame source frame event to our trackers queue function
	//TODO - the function signature invoked depending on the FRAMESERVER_EVENT_TYPE needs to be documented/codified
	frame_source->frameserver_register_frame_callback(frame_source,tracker,tracker->tracker_queue);

	printf("frame source path: %s %d x %d\n",&(descriptors[source_index].name), descriptors[source_index].width,descriptors[source_index].height,descriptors[source_index].format);

	frame_source->frameserver_configure_capture(frame_source,tracker->tracker_get_capture_params(tracker));

	frame_source->frameserver_stream_start(frame_source,&(descriptors[source_index]));

	//we can now poll our tracker for data, and update our xrt-side objects position
	//just poll 20 times for our test
	for (uint32_t i=0;i<20;i++){
		tracker->tracker_get_poses(tracker,tracked_objects,&tracked_object_count);
		// tracked_object_count will contain the number of objects found in the frame
		// there is no guarantee that they will be in the same order across calls
		for (uint32_t i =0; i< tracked_object_count;i++)
		{
			tracked_object_t o = tracked_objects[i];
			//printf("tracked object id %d tag %d pos %f %f %f\n",o.tracking_id,o.tracking_tag,o.pose.position.x,o.pose.position.y,o.pose.position.z);
		}
		usleep(5000);
	}


	return true;
}
