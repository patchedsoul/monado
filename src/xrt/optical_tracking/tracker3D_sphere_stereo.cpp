#include "tracker3D_sphere_stereo.h"
#include "opencv4/opencv2/opencv.hpp"

typedef struct tracker3D_sphere_stereo_instance {
	bool configured;
	tracker_stereo_configuration_t configuration;
	measurement_consumer_callback_func measurement_target_callback;
	void* measurement_target_instance; //where we send our measurements
	camera_calibration_t l_calibration;
	camera_calibration_t r_calibration;

	tracked_object_t tracked_object;
	tracked_blob_t l_tracked_blob;
	tracked_blob_t r_tracked_blob;

	bool poses_consumed;
	cv::SimpleBlobDetector::Params params;
	std::vector<cv::KeyPoint> l_keypoints;
	std::vector<cv::KeyPoint> r_keypoints;
	//these components hold no state so we can use a single instance for l and r
	cv::Ptr<cv::SimpleBlobDetector> sbd;
	cv::Ptr<cv::BackgroundSubtractorMOG2> background_subtractor;
	bool split_left;
	cv::Mat l_frame_gray;
	cv::Mat r_frame_gray;
	cv::Mat l_mask_gray;
	cv::Mat r_mask_gray;

	cv::Mat debug_rgb;
	cv::Mat l_intrinsics;
	cv::Mat l_distortion;
	cv::Mat r_intrinsics;
	cv::Mat r_distortion;

	bool l_alloced_frames;
	bool r_alloced_frames;

	bool got_left;
	bool got_right;
} tracker3D_sphere_stereo_instance_t;

tracker3D_sphere_stereo_instance_t* tracker3D_sphere_stereo_create(tracker_instance_t* inst) {
	tracker3D_sphere_stereo_instance_t* i = (tracker3D_sphere_stereo_instance_t*)calloc(1,sizeof(tracker3D_sphere_stereo_instance_t));
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
		i->background_subtractor = cv::createBackgroundSubtractorMOG2(32,16,false);

		i->poses_consumed=false;
		i->configured=false;
		i->l_alloced_frames=false;
		i->r_alloced_frames=false;
		int intrinsics_dim = sqrt(INTRINSICS_SIZE);
		i->l_intrinsics = cv::Mat(intrinsics_dim,intrinsics_dim,CV_32F);
		i->l_distortion = cv::Mat(DISTORTION_SIZE,1,CV_32F);
		i->r_intrinsics = cv::Mat(intrinsics_dim,intrinsics_dim,CV_32F);
		i->r_distortion = cv::Mat(DISTORTION_SIZE,1,CV_32F);

		//alloc our debug frame here - opencv is h,w, not w,h
		i->debug_rgb = cv::Mat(480,640,CV_8UC3,cv::Scalar(0,0,0));

		i->got_left = false;
		i->got_right = false;
		return i;
	}
	return NULL;
}
bool tracker3D_sphere_stereo_get_debug_frame(tracker_instance_t* inst,frame_t* frame){
	tracker3D_sphere_stereo_instance_t* internal = (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	frame->format = FORMAT_RGB_UINT8;
	frame->width = internal->debug_rgb.cols;
	frame->stride = internal->debug_rgb.cols * format_bytes_per_pixel(frame->format);
	frame->height = internal->debug_rgb.rows;
	frame->data = internal->debug_rgb.data;
	frame->size_bytes = frame_size_in_bytes(frame);
	return true;
}
capture_parameters_t tracker3D_sphere_stereo_get_capture_params(tracker_instance_t* inst) {
	capture_parameters_t cp={};
	cp.exposure = 0.5f;
	cp.gain=0.1f;
	return cp;
}

bool tracker3D_sphere_stereo_queue(tracker_instance_t* inst,frame_t* frame) {
	tracker3D_sphere_stereo_instance_t* internal = (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	if (! internal->configured){
		printf("ERROR: you must configure this tracker before it can accept frames\n");
		return false;
	}
	printf("received frame, tracking!\n");

	//if we have a composite stereo frame, alloc both left and right eyes when we see a left frame

	if (frame->source_id == internal->configuration.l_source_id &&  !internal->l_alloced_frames)
	{
		uint16_t eye_width = frame->width/2;
		if (internal->configuration.split_left == true) {
			eye_width = frame->width /2;
			internal->r_frame_gray = cv::Mat(frame->height,eye_width,CV_8UC1,cv::Scalar(0,0,0));
			internal->r_mask_gray = cv::Mat(frame->height,eye_width,CV_8UC1,cv::Scalar(0,0,0));
			internal->r_alloced_frames =true;
		}
		internal->l_frame_gray = cv::Mat(frame->height,eye_width,CV_8UC1,cv::Scalar(0,0,0));
		internal->l_mask_gray = cv::Mat(frame->height,eye_width,CV_8UC1,cv::Scalar(0,0,0));
		internal->l_alloced_frames =true;
	}

	if (frame->source_id == internal->configuration.r_source_id &&  !internal->r_alloced_frames)
	{
		uint16_t eye_width = frame->width/2;
		internal->r_frame_gray = cv::Mat(frame->height,eye_width,CV_8UC1,cv::Scalar(0,0,0));
		internal->r_mask_gray = cv::Mat(frame->height,eye_width,CV_8UC1,cv::Scalar(0,0,0));
		internal->r_alloced_frames =true;
	}

	//we will just 'do the work' here, normally this frame
	//would be added to a queue and a tracker thread
	//would analyse it asynchronously.


	if (frame->source_id == internal->configuration.l_source_id) {

		if (internal->configuration.split_left == true) {
			internal->l_keypoints.clear();
			internal->r_keypoints.clear();
			internal->got_left=true;
			internal->got_right=true;
			//TODO: other pixel formats
			cv::Mat tmp(frame->height, frame->width, CV_8UC1, frame->data);
			cv::bitwise_not ( tmp,tmp);
			cv::Rect lr(internal->configuration.l_rect.tl.x,internal->configuration.l_rect.tl.y,internal->configuration.l_rect.br.x,internal->configuration.l_rect.br.y);
			cv::Rect rr(internal->configuration.r_rect.tl.x,internal->configuration.r_rect.tl.y,internal->configuration.r_rect.br.x - internal->configuration.r_rect.tl.x,internal->configuration.r_rect.br.y);
			tmp(lr).copyTo(internal->l_frame_gray);
			tmp(rr).copyTo(internal->r_frame_gray);
			internal->background_subtractor->apply(internal->l_frame_gray,internal->l_mask_gray);
			internal->background_subtractor->apply(internal->r_frame_gray,internal->r_mask_gray);

			xrt_vec2 lastPos = internal->l_tracked_blob.center;
			float offset = ROI_OFFSET;
			if (internal->l_tracked_blob.diameter > ROI_OFFSET) {
				offset = internal->l_tracked_blob.diameter;
			}

			cv::rectangle(internal->l_mask_gray, cv::Point2f(lastPos.x-offset,lastPos.y-offset),cv::Point2f(lastPos.x+offset,lastPos.y+offset),cv::Scalar( 255 ),-1,0);
			lastPos = internal->r_tracked_blob.center;
			cv::rectangle(internal->r_mask_gray, cv::Point2f(lastPos.x-offset,lastPos.y-offset),cv::Point2f(lastPos.x+offset,lastPos.y+offset),cv::Scalar( 255 ),-1,0);
		}
		else
		{
			internal->l_keypoints.clear();
			internal->got_left=true;
			memcpy(internal->l_frame_gray.data,frame->data,frame->size_bytes);
			cv::bitwise_not ( internal->l_frame_gray,internal->l_frame_gray);
			internal->background_subtractor->apply(internal->l_frame_gray,internal->l_mask_gray);
			xrt_vec2 lastPos = internal->l_tracked_blob.center;
			float offset = ROI_OFFSET;
			if (internal->l_tracked_blob.diameter > ROI_OFFSET) {
				offset = internal->l_tracked_blob.diameter;
			}

			cv::rectangle(internal->l_mask_gray, cv::Point2f(lastPos.x-offset,lastPos.y-offset),cv::Point2f(lastPos.x+offset,lastPos.y+offset),cv::Scalar( 255 ),-1,0);

		}


	}
	if (frame->source_id == internal->configuration.r_source_id) {
		internal->r_keypoints.clear();
		internal->got_right=true;
		memcpy(internal->r_frame_gray.data,frame->data,frame->size_bytes);
		cv::bitwise_not ( internal->r_frame_gray,internal->r_frame_gray);
		internal->background_subtractor->apply(internal->r_frame_gray,internal->r_mask_gray);
		xrt_vec2 lastPos = internal->r_tracked_blob.center;
		float offset = ROI_OFFSET;
		if (internal->r_tracked_blob.diameter > ROI_OFFSET) {
			offset = internal->r_tracked_blob.diameter;
		}

		cv::rectangle(internal->r_mask_gray, cv::Point2f(lastPos.x-offset,lastPos.y-offset),cv::Point2f(lastPos.x+offset,lastPos.y+offset),cv::Scalar( 255 ),-1,0);


	}

	if (internal->got_left && internal->got_right)
	{
		cv::KeyPoint blob;
		tracker_measurement_t m = {};
		//clear our debug image
		cv::rectangle(internal->debug_rgb, cv::Point2f(0,0),cv::Point2f(internal->debug_rgb.cols,internal->debug_rgb.rows),cv::Scalar( 0,0,0 ),-1,0);

		//do blob detection with our masks
		internal->sbd->detect(internal->l_frame_gray, internal->l_keypoints,internal->l_mask_gray);
		internal->sbd->detect(internal->r_frame_gray, internal->r_keypoints,internal->r_mask_gray);


		//TODO: select the most likely blob here
		for (uint32_t i=0;i<internal->l_keypoints.size();i++)
		{
			blob = internal->l_keypoints.at(i);
			printf ("LEFT 2D blob X: %f Y: %f D:%f\n",blob.pt.x,blob.pt.y,blob.size);
		}
		for (uint32_t i=0;i<internal->r_keypoints.size();i++)
		{
			blob = internal->r_keypoints.at(i);
			printf ("RIGHT 2D blob X: %f Y: %f D:%f\n",blob.pt.x,blob.pt.y,blob.size);
		}

		cv::drawKeypoints(internal->debug_rgb,internal->l_keypoints,internal->debug_rgb,cv::Scalar(128,255,32),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv::drawKeypoints(internal->debug_rgb,internal->r_keypoints,internal->debug_rgb,cv::Scalar(32,128,255),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		/*
		if (internal->keypoints.size() > 0) {
			internal->tracked_blob.center = {blob.pt.x,blob.pt.y};
			internal->tracked_blob.diameter=blob.size;

			// intrinsics are 3x3 - compensate for difference between
			// measurement framesize and current frame size
			// TODO: handle differing aspect ratio
			float xscale = internal->frame_gray.cols / internal->l_calibration.calib_capture_size[0];
			float yscale = internal->frame_gray.rows / internal->l_calibration.calib_capture_size[1];

			float cx = internal->l_calibration.intrinsics[2] *  xscale;
			float cy = internal->l_calibration.intrinsics[5] * yscale;
			float focalx=internal->l_calibration.intrinsics[0] * xscale;
			float focaly=internal->l_calibration.intrinsics[4] * yscale;

			// we can just undistort our blob-centers, rather than undistorting
			// every pixel in the frame
			cv::Mat srcArray(1,1,CV_32FC2);
			cv::Mat dstArray(1,1,CV_32FC2);

			srcArray.at<cv::Point2f>(1,1) = cv::Point2f(internal->tracked_blob.center.x,internal->tracked_blob.center.y);
			cv::undistortPoints(srcArray,dstArray,internal->l_intrinsics,internal->l_distortion);

			float pixelConstant =1.0f;
			float z = internal->tracked_blob.diameter * pixelConstant;
			float x = ((cx - dstArray.at<float>(0,0)) * z) / focalx;
			float y = ((cy - dstArray.at<float>(0,1) ) * z) / focaly;

			printf("%f %f %f\n",x,y,z);

			m.has_position = true;
			m.timestamp =0;
			m.pose.position.x = x;
			m.pose.position.y = y;
			m.pose.position.z = z;


			if (internal->measurement_target_callback){
				internal->measurement_target_callback(internal->measurement_target_instance,&m);
			}
		}
		*/
		//write our debug frame out
		tracker_send_debug_frame(inst); //publish our debug frame
		internal->got_left = false;
		internal->got_right = false;

	} else
	{
		return true;
	}
	//we should never get here
	return false;
}

bool tracker3D_sphere_stereo_get_poses(tracker_instance_t* inst,tracked_object_t* objects, uint32_t* count) {
	if (objects == NULL)
	{
		*count = TRACKED_POINTS; //tracking a single object
		return true;
	}

	tracker3D_sphere_stereo_instance_t* internal = (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	for (uint32_t i = 0;i< 1;i++) {

		objects[i] = internal->tracked_object;
	}
	*count=1;
	internal->poses_consumed=true;
	return true;
}

bool tracker3D_sphere_stereo_new_poses(tracker_instance_t* inst)
{
	tracker3D_sphere_stereo_instance_t*  internal = (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	return internal->poses_consumed;
}

bool tracker3D_sphere_stereo_configure(tracker_instance_t* inst,tracker_stereo_configuration_t* config)
{
	tracker3D_sphere_stereo_instance_t*  internal = (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	//return false if we cannot handle this config

	if (config->l_format != FORMAT_Y_UINT8) {
		internal->configured=false;
		return false;
	}
	internal->configuration = *config;
	//copy configuration data into opencv mats
	memcpy(internal->l_intrinsics.ptr(0),internal->configuration.l_calibration.intrinsics,sizeof(internal->configuration.l_calibration.intrinsics));
	memcpy(internal->l_distortion.ptr(0),internal->configuration.l_calibration.distortion,sizeof(internal->configuration.l_calibration.distortion));
	memcpy(internal->r_intrinsics.ptr(0),internal->configuration.r_calibration.intrinsics,sizeof(internal->configuration.r_calibration.intrinsics));
	memcpy(internal->r_distortion.ptr(0),internal->configuration.r_calibration.distortion,sizeof(internal->configuration.r_calibration.distortion));

	internal->configured=true;
	return true;
}

void tracker3D_sphere_stereo_register_measurement_callback (tracker_instance_t* inst, void* target_instance, measurement_consumer_callback_func target_func) {
	tracker3D_sphere_stereo_instance_t* internal = (tracker3D_sphere_stereo_instance_t*)inst->internal_instance;
	internal->measurement_target_instance = target_instance;
	internal->measurement_target_callback = target_func;
}

