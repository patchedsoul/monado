#include "tracker3D_sphere_mono.h"
#include "opencv4/opencv2/opencv.hpp"

typedef struct tracker3D_sphere_mono_instance {
	bool configured;
	camera_calibration_t calibration;
	tracked_object_t tracked_object;
	bool poses_consumed;
	cv::SimpleBlobDetector::Params params;
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::SimpleBlobDetector> sbd;
	cv::Ptr<cv::BackgroundSubtractorMOG2> background_subtractor;
	cv::Mat frame_gray;
	cv::Mat mask_gray;
	cv::Mat debug_rgb;
	bool alloced_frames;
} tracker3D_sphere_mono_instance_t;

tracker3D_sphere_mono_instance_t* tracker3D_sphere_mono_create(tracker_instance_t* inst) {
	tracker3D_sphere_mono_instance_t* i = (tracker3D_sphere_mono_instance_t*)calloc(1,sizeof(tracker3D_sphere_mono_instance_t));
	if (i) {
		i->params.filterByArea=false;
		i->params.filterByConvexity=false;
		i->params.filterByInertia=false;
		i->params.filterByColor=true;
		i->params.blobColor=0; //0 or 255 - color comes from binarized image?
		i->params.minArea=1;
		i->params.maxArea=1000;
		i->params.maxThreshold=51; //using a wide threshold span slows things down bigtime
		i->params.minThreshold=50;
		i->params.thresholdStep=1;
		i->params.minDistBetweenBlobs=5;
		i->params.minRepeatability=1; //need this to avoid error?

		i->sbd = cv::SimpleBlobDetector::create(i->params);
		i->background_subtractor = cv::createBackgroundSubtractorMOG2(128,64,false);

		i->poses_consumed=false;
		i->configured=false;
		i->alloced_frames=false;
		return i;
	}
	return NULL;
}
bool tracker3D_sphere_mono_get_debug_frame(tracker_instance_t* inst,frame_t* frame){
	//not implemented
	return false;
}
capture_parameters_t tracker3D_sphere_mono_get_capture_params(tracker_instance_t* inst) {
	capture_parameters_t cp={};
	cp.exposure = 0.1f;
	cp.gain=0.1f;
	return cp;
}

bool tracker3D_sphere_mono_queue(tracker_instance_t* inst,frame_t* frame) {
	tracker3D_sphere_mono_instance_t* internal = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;
	if (! internal->configured){
		printf("ERROR: you must configure this tracker before it can accept frames\n");
		return false;
	}
	printf("received frame, tracking!\n");
	if (!internal->alloced_frames)
	{
		internal->frame_gray = cv::Mat(frame->height,frame->stride,CV_8UC1,cv::Scalar(0,0,0));
		internal->mask_gray = cv::Mat(frame->height,frame->stride,CV_8UC1,cv::Scalar(0,0,0));
		internal->debug_rgb = cv::Mat(frame->height,frame->width,CV_8UC3,cv::Scalar(0,0,0));
		internal->alloced_frames =true;
	}
	//we will just 'do the work' here, normally this frame
	//would be added to a queue and a tracker thread
	//would analyse it asynchronously.

	//we will just randomise the pose to show 'something'
	//is being done
	internal->keypoints.clear();
	memcpy(internal->frame_gray.data,frame->data,frame->size_bytes);
	cv::bitwise_not ( internal->frame_gray,internal->frame_gray);
	bool ret = cv::imwrite("/tmp/out.jpg",internal->frame_gray);

	//add this frame to the background average mask generator
	internal->background_subtractor->apply(internal->frame_gray,internal->mask_gray);
	//we always want to be able to track small motions, so write white blocks into the masks that encompass the last seen positions of the blobs



	for (uint32_t i=0;i<TRACKED_POINTS;i++)
	{
		//xrt_vec3 lastPos = lastTrackData.positions2D[i];
		//if (lastTrackData.confidence[i] > 0)
		//{
		 //   cv::rectangle(leftCameraGray[1], cv::Point2f(lastPos.x-32,lastPos.y-32),cv::Point2f(lastPos.x+32,lastPos.y+32),cv::Scalar( 255 ),-1,0);
		 //   cv::rectangle(rightCameraGray[1],cv::Point2f(lastPos.z-32,lastPos.w-32),cv::Point2f(lastPos.z+32,lastPos.w+32),cv::Scalar( 255 ),-1,0);
		//}
	}

	//do blob detection with our mask
	internal->sbd->detect(internal->frame_gray, internal->keypoints);//,internal->mask_gray);
	for (uint32_t i=0;i<internal->keypoints.size();i++)
	{
		cv::KeyPoint blob = internal->keypoints.at(i);
		//printf ("2D blob X: %f Y: %f\n",blob.pt.x,blob.pt.y);

		//intrinsics are 3x3
		float cx = internal->calibration.intrinsics[2];
		float cy = internal->calibration.intrinsics[5];
		float focalx=internal->calibration.intrinsics[0];
		float focaly=internal->calibration.intrinsics[4];

		float pixelConstant =1.0f;
		float z = blob.size * pixelConstant;
		float x = ((cx - blob.pt.x) * z) / focalx;
		float y = ((cy - blob.pt.y ) * z) / focaly;

		printf("%f %f %f\n",x,y,z);
	}
	return true;
}

bool tracker3D_sphere_mono_get_poses(tracker_instance_t* inst,tracked_object_t* objects, uint32_t* count) {
	if (objects == NULL)
    {
		*count = TRACKED_POINTS; //tracking a single object
        return true;
    }

	tracker3D_sphere_mono_instance_t* internal = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;
	for (uint32_t i = 0;i< 1;i++) {

		objects[i] = internal->tracked_object;
    }
	*count=1;
	internal->poses_consumed=true;
	return true;
}

bool tracker3D_sphere_mono_new_poses(tracker_instance_t* inst)
{
	tracker3D_sphere_mono_instance_t*  internal = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;
	return internal->poses_consumed;
}

bool tracker3D_sphere_mono_configure(tracker_instance_t* inst,tracker_mono_configuration_t* config)
{
	tracker3D_sphere_mono_instance_t*  internal = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;
	//return false if we cannot handle this config

	if (config->format != FORMAT_Y_UINT8) {
		internal->configured=false;
		return false;
	}
	internal->calibration = config->calibration;
	internal->configured=true;
	return true;
}

