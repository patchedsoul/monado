#include "tracker3D_sphere_mono.h"
#include "opencv4/opencv2/opencv.hpp"
#include "common/opencv_utils.hpp"


#define MAX_CALIBRATION_SAMPLES 23

typedef struct tracker3D_sphere_mono_instance {
	bool configured;
	measurement_consumer_callback_func measurement_target_callback;
	void* measurement_target_instance; //where we send our measurements
	event_consumer_callback_func event_target_callback;
	void* event_target_instance; //where we send our measurements
	tracker_mono_configuration_t configuration;
	tracked_object_t tracked_object;
	tracked_blob_t tracked_blob;
	bool poses_consumed;
	cv::SimpleBlobDetector::Params params;
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::SimpleBlobDetector> sbd;
	cv::Ptr<cv::BackgroundSubtractorMOG2> background_subtractor;
	cv::Mat frame_gray;
	cv::Mat mask_gray;
	cv::Mat debug_rgb;
	cv::Mat intrinsics;
	cv::Mat distortion;
	cv::Mat distortion_fisheye;

	cv::Mat undistort_map_x;
	cv::Mat undistort_map_y;

	//calibration data structures
	std::vector<std::vector<cv::Point3f>> chessboards_model;
	std::vector<std::vector<cv::Point2f>> chessboards_measured;

	bool calibrated;
	bool alloced_frames;
} tracker3D_sphere_mono_instance_t;

tracker3D_sphere_mono_instance_t* tracker3D_sphere_mono_create(tracker_instance_t* inst) {
	tracker3D_sphere_mono_instance_t* i = (tracker3D_sphere_mono_instance_t*)calloc(1,sizeof(tracker3D_sphere_mono_instance_t));
	if (i) {
		i->params.filterByArea=false;
		i->params.filterByConvexity=false;
		i->params.filterByInertia=false;
		i->params.filterByColor=true;
		i->params.blobColor=255; //0 or 255 - color comes from binarized image?
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
		i->alloced_frames=false;
		int intrinsics_dim = sqrt(INTRINSICS_SIZE);
		i->intrinsics = cv::Mat(intrinsics_dim,intrinsics_dim,CV_32F);
		i->distortion = cv::Mat(DISTORTION_SIZE,1,CV_32F);

		return i;
	}
	return NULL;
}
bool tracker3D_sphere_mono_get_debug_frame(tracker_instance_t* inst,frame_t* frame){
    tracker3D_sphere_mono_instance_t* internal = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;
    frame->format = FORMAT_RGB_UINT8;
    frame->width = internal->debug_rgb.cols;
    frame->stride = internal->debug_rgb.cols * format_bytes_per_pixel(frame->format);
    frame->height = internal->debug_rgb.rows;
    frame->data = internal->debug_rgb.data;
    frame->size_bytes = frame_size_in_bytes(frame);
    return true;
}
capture_parameters_t tracker3D_sphere_mono_get_capture_params(tracker_instance_t* inst) {
	capture_parameters_t cp={};
    cp.exposure = 0.5f;
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
	//we will just 'do the work' here.
	//TODO: asynchronous tracker thread

	memcpy(internal->frame_gray.data,frame->data,frame->size_bytes);

	switch (internal->configuration.calibration_mode) {
	        case CALIBRATION_MODE_NONE:
		        return tracker3D_sphere_mono_track(inst);
		        break;
	        case CALIBRATION_MODE_CHESSBOARD:
		        return tracker3D_sphere_mono_calibrate(inst);
		        break;
	        default:
		        printf("ERROR: unrecognised calibration mode\n");
		        return false;
	    }
	return true;
}




bool tracker3D_sphere_mono_track(tracker_instance_t* inst)
{
	printf("tracking...\n");
	tracker3D_sphere_mono_instance_t* internal = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;

	internal->keypoints.clear();
	cv::Size image_size(internal->frame_gray.cols,internal->frame_gray.rows);
	// TODO: save data indicating calibration image size
	// and multiply intrinsics accordingly


	//add this frame to the background average mask generator
	internal->background_subtractor->apply(internal->frame_gray,internal->mask_gray);
	//we always want to be able to track small motions, so write white blocks into the masks that encompass the last seen positions of the blobs
	xrt_vec2 lastPos = internal->tracked_blob.center;
	float offset = ROI_OFFSET;
	if (internal->tracked_blob.diameter > ROI_OFFSET) {
		offset = internal->tracked_blob.diameter;
	}
	//ensure we dont mask out our blob
	cv::rectangle(internal->mask_gray, cv::Point2f(lastPos.x-offset,lastPos.y-offset),cv::Point2f(lastPos.x+offset,lastPos.y+offset),cv::Scalar( 255 ),-1,0);

	//write something into our debug image
	cv::rectangle(internal->debug_rgb, cv::Point2f(0,0),cv::Point2f(internal->debug_rgb.cols,internal->debug_rgb.rows),cv::Scalar( 0,0,0 ),-1,0);
	cv::rectangle(internal->debug_rgb, cv::Point2f(lastPos.x-offset,lastPos.y-offset),cv::Point2f(lastPos.x+offset,lastPos.y+offset),cv::Scalar( 0,0,255 ),-1,0);

	//do blob detection with our mask
	internal->sbd->detect(internal->frame_gray, internal->keypoints,internal->mask_gray);
	//bool ret = cv::imwrite("/tmp/out.jpg",internal->frame_gray);
	//ret = cv::imwrite("/tmp/mask.jpg",internal->mask_gray);

	cv::KeyPoint blob;
	tracker_measurement_t m = {};
	//we can just grab the last blob in our list.

	//TODO: select the most likely blob here
	for (uint32_t i=0;i<internal->keypoints.size();i++)
	{
		blob = internal->keypoints.at(i);
		printf ("2D blob X: %f Y: %f D:%f\n",blob.pt.x,blob.pt.y,blob.size);
	}
	cv::drawKeypoints(internal->frame_gray,internal->keypoints,internal->debug_rgb,cv::Scalar(128,255,18),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	if (internal->keypoints.size() > 0) {
		internal->tracked_blob.center = {blob.pt.x,blob.pt.y};
		internal->tracked_blob.diameter=blob.size;

		float cx = internal->intrinsics.at<double>(0,2);
		float cy = internal->intrinsics.at<double>(1,2);
		float focalx=internal->intrinsics.at<double>(0,0);
		float focaly=internal->intrinsics.at<double>(1,1);

		// we can just undistort our tracked blob-center, rather than undistorting
		// every pixel in the frame
		std::vector<cv::Point2f> src;
		cv::Mat dst;

		src.push_back(cv::Point2f(internal->tracked_blob.center.x,internal->tracked_blob.center.y));

		cv::undistortPoints(src,dst,internal->intrinsics,internal->distortion,cv::noArray(),cv::noArray());
		float pixelConstant =1.0f;
		float z = internal->tracked_blob.diameter * pixelConstant;
		float x = dst.at<float>(0,0);
		float y = dst.at<float>(0,1);

		cv::circle(internal->debug_rgb,cv::Point2f(x*focalx + cx,y *focaly +cy),3,cv::Scalar(32,32,192));

		//printf("%f %f %f\n",x,y,z);

		m.has_position = true;
		m.timestamp =0;
		m.pose.position.x = x;
		m.pose.position.y = y;
		m.pose.position.z = z;


		if (internal->measurement_target_callback){
			internal->measurement_target_callback(internal->measurement_target_instance,&m);
		}
	}

	//publish our debug frame
	tracker_send_debug_frame(inst); //publish our debug frame

	return true;
}
bool tracker3D_sphere_mono_calibrate(tracker_instance_t* inst)
{
	printf("calibrating...\n");
	tracker3D_sphere_mono_instance_t* internal = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;
	cv::Size image_size(internal->frame_gray.cols,internal->frame_gray.rows);

	//TODO: use multiple env vars? - centralise this
	char path_string[1024];
	char* config_path = secure_getenv("HOME");
	snprintf(path_string,1024,"%s/.config/monado/%s.calibration",config_path,internal->configuration.configuration_filename);

	printf("TRY LOADING CONFIG FROM %s\n",path_string);
	FILE* calib_file = fopen(path_string,"rb");
	if (calib_file) {
		//read our calibration from this file
		read_mat(calib_file,&internal->intrinsics);
		read_mat(calib_file,&internal->distortion);
		read_mat(calib_file,&internal->distortion_fisheye);

		// TODO: save data indicating calibration image size
		// and multiply intrinsics accordingly

		cv::initUndistortRectifyMap(internal->intrinsics, internal->distortion, cv::noArray(), internal->intrinsics, image_size, CV_32FC1, internal->undistort_map_x, internal->undistort_map_y);

		printf("calibrated cameras! setting tracking mode\n");
		internal->calibrated=true;

		internal->configuration.calibration_mode = CALIBRATION_MODE_NONE;
		//send an event to notify our driver of the switch into tracking mode.
		driver_event_t e ={};
		e.type = EVENT_TRACKER_RECONFIGURED;
		internal->event_target_callback(internal->event_target_instance,e);
		return true;
	}

	//no saved file - perform interactive calibration.
	//try and find a chessboard in the image, and run the calibration.
	//TODO: we need to define some mechanism for UI/user interaction.

	// TODO: initialise this on construction and move this to internal state
	cv::Size board_size(8,6);
	std::vector<cv::Point3f> chessboard_model;

	for (uint32_t i=0;i< board_size.width * board_size.height;i++) {
		cv::Point3f p(i/board_size.width,i % board_size.width,0.0f);
		chessboard_model.push_back(p);
	}

	cv::Mat chessboard_measured;

	//clear our debug image
	cv::rectangle(internal->debug_rgb, cv::Point2f(0,0),cv::Point2f(internal->debug_rgb.cols,internal->debug_rgb.rows),cv::Scalar( 0,0,0 ),-1,0);

	//we will collect samples continuously - the user should be able to wave a chessboard around randomly
	//while the system calibrates..

	//TODO: we need a coverage measurement and an accuracy measurement,
	// so we can converge to something that is as complete and correct as possible.

	bool found_board = cv::findChessboardCorners(internal->frame_gray,board_size,chessboard_measured);
	char message[128];
	message[0]=0x0;

	if ( found_board ){
		//we will use the last n samples to calculate our calibration
		if (internal->chessboards_measured.size() > MAX_CALIBRATION_SAMPLES)
		{
			internal->chessboards_measured.erase(internal->chessboards_measured.begin());
		}
		else
		{
			internal->chessboards_model.push_back(chessboard_model);
		}

		internal->chessboards_measured.push_back(chessboard_measured);

		if (internal->chessboards_measured.size() == MAX_CALIBRATION_SAMPLES)
		{
			//TODO - run this if coverage test passes
			cv::Mat rvecs,tvecs;

			float rp_error = cv::calibrateCamera(internal->chessboards_model,internal->chessboards_measured,image_size,internal->intrinsics,internal->distortion,rvecs,tvecs);

			cv::initUndistortRectifyMap(internal->intrinsics, internal->distortion, cv::noArray(), internal->intrinsics, image_size, CV_32FC1, internal->undistort_map_x, internal->undistort_map_y);

			char path_string[PATH_MAX];
			char file_string[PATH_MAX];
			//TODO: use multiple env vars?
			char* config_path = secure_getenv("HOME");
			snprintf(path_string,PATH_MAX,"%s/.config/monado",config_path);
			snprintf(file_string,PATH_MAX,"%s/.config/monado/%s.calibration",config_path,internal->configuration.configuration_filename);

			printf("TRY WRITING CONFIG TO %s\n",file_string);
			FILE* calib_file = fopen(file_string,"wb");
			if (! calib_file) {
				mkpath(path_string);
			}
			calib_file = fopen(file_string,"wb");
			if (! calib_file) {
				printf("ERROR. could not create calibration file %s\n",file_string);
			} else {
				write_mat(calib_file,&internal->intrinsics);
				write_mat(calib_file,&internal->distortion);
				write_mat(calib_file,&internal->distortion_fisheye);
				fclose(calib_file);
			}

			printf("calibrated cameras! setting tracking mode\n");
			internal->calibrated=true;
			internal->configuration.calibration_mode = CALIBRATION_MODE_NONE;
			//send an event to notify our driver of the switch into tracking mode.
			driver_event_t e ={};
			e.type = EVENT_TRACKER_RECONFIGURED;
			internal->event_target_callback(internal->event_target_instance,e);
		} else {
			snprintf(message,128,"COLLECTING SAMPLE: %d/%d",internal->chessboards_measured.size() +1,MAX_CALIBRATION_SAMPLES);
		}
	}

	cv::drawChessboardCorners(internal->debug_rgb,board_size,chessboard_measured,found_board);

	cv::putText(internal->debug_rgb,"CALIBRATION MODE",cv::Point2i(160,240),0,1.0f,cv::Scalar(192,192,192));
	cv::putText(internal->debug_rgb,message,cv::Point2i(160,460),0,0.5f,cv::Scalar(192,192,192));

	tracker_send_debug_frame(inst);

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
	internal->configuration = *config;
	internal->configured=true;
	return true;
}

void tracker3D_sphere_mono_register_measurement_callback (tracker_instance_t* inst, void* target_instance, measurement_consumer_callback_func target_func) {
	tracker3D_sphere_mono_instance_t* internal = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;
	internal->measurement_target_instance = target_instance;
	internal->measurement_target_callback = target_func;
}
void tracker3D_sphere_mono_register_event_callback (tracker_instance_t* inst, void* target_instance, event_consumer_callback_func target_func) {
	tracker3D_sphere_mono_instance_t* internal = (tracker3D_sphere_mono_instance_t*)inst->internal_instance;
	internal->event_target_instance = target_instance;
	internal->event_target_callback = target_func;
}
