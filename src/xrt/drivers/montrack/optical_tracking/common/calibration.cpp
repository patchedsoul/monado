

/* TODO: move the current in-tracker calibration here */
#include <opencv2/opencv.hpp>

#include "calibration.h"
#include "common/opencv_utils.hpp"

#include <sys/stat.h>
#include <linux/limits.h>

#include "util/u_misc.h"

#define MAX_CALIBRATION_SAMPLES                                                \
    23 // mo' samples, mo' calibration accuracy, at the expense of time.

static bool tracker3D_calibration_stereo_calibrate(tracker_instance_t* inst);

bool calibration_get_stereo(char* configuration_filename,bool use_fisheye, cv::Mat* l_undistort_map_x,cv::Mat* l_undistort_map_y,cv::Mat* l_rectify_map_x,cv::Mat* l_rectify_map_y,cv::Mat* r_undistort_map_x,cv::Mat* r_undistort_map_y,cv::Mat* r_rectify_map_x,cv::Mat* r_rectify_map_y,cv::Mat* disparity_to_depth){

    //intermediate data - we may want to expose these at some point.
    cv::Mat l_intrinsics;
    cv::Mat l_distortion;
    cv::Mat l_distortion_fisheye;
    cv::Mat l_translation;
    cv::Mat l_rotation;
    cv::Mat l_projection;
    cv::Mat r_intrinsics;
    cv::Mat r_distortion;
    cv::Mat r_distortion_fisheye;
    cv::Mat r_translation;
    cv::Mat r_rotation;
    cv::Mat r_projection;
    cv::Mat mat_image_size;

    cv::Mat zero_distortion = cv::Mat(DISTORTION_SIZE, 1, CV_32F, cv::Scalar(0.0f));

    char path_string[256]; // TODO: 256 maybe not enough
    // TODO: use multiple env vars?
    char* config_path = secure_getenv("HOME");
    snprintf(path_string, 256, "%s/.config/monado/%s.calibration",
             config_path, configuration_filename); // TODO: hardcoded 256

    printf("TRY LOADING CONFIG FROM %s\n", path_string);
    FILE* calib_file = fopen(path_string, "rb");
    if (calib_file) {
        // read our calibration from this file
        read_mat(calib_file, &l_intrinsics);
        read_mat(calib_file, &r_intrinsics);
        read_mat(calib_file, &l_distortion);
        read_mat(calib_file, &r_distortion);
        read_mat(calib_file, &l_distortion_fisheye);
        read_mat(calib_file, &r_distortion_fisheye);
        read_mat(calib_file, &l_rotation);
        read_mat(calib_file, &r_rotation);
        read_mat(calib_file, &l_translation);
        read_mat(calib_file, &r_translation);
        read_mat(calib_file, &l_projection);
        read_mat(calib_file, &r_projection);
        read_mat(calib_file,disparity_to_depth); //provided by caller
        read_mat(calib_file,&mat_image_size);

        cv::Size image_size(mat_image_size.at<float>(0,0),mat_image_size.at<float>(0,1));

        if (use_fisheye){
            cv::fisheye::initUndistortRectifyMap(
                l_intrinsics, l_distortion_fisheye,
                cv::noArray(), l_intrinsics, image_size, CV_32FC1,
                *l_undistort_map_x, *l_undistort_map_y);
            cv::fisheye::initUndistortRectifyMap(
                r_intrinsics, r_distortion_fisheye,
                cv::noArray(), r_intrinsics, image_size, CV_32FC1,
                *r_undistort_map_x, *r_undistort_map_y);
        } else {
            cv::fisheye::initUndistortRectifyMap(
                l_intrinsics, l_distortion_fisheye,
                cv::noArray(), l_intrinsics, image_size, CV_32FC1,
                *l_undistort_map_x, *l_undistort_map_y);
            cv::fisheye::initUndistortRectifyMap(
                r_intrinsics, r_distortion_fisheye,
                cv::noArray(), r_intrinsics, image_size, CV_32FC1,
                *r_undistort_map_x, *r_undistort_map_y);
        }

        cv::initUndistortRectifyMap(
            l_intrinsics, zero_distortion,
            l_rotation, l_projection, image_size,
            CV_32FC1, *l_rectify_map_x,
            *l_rectify_map_y);
        cv::initUndistortRectifyMap(
            r_intrinsics, zero_distortion,
            r_rotation, r_projection, image_size,
            CV_32FC1, *r_rectify_map_x,
            *r_rectify_map_y);
}
}

    bool calibration_get_mono(char* camera_id,cv::Mat* undistort_map_x,cv::Mat* undistort_map_y){
        return false;
    }



static bool
tracker3D_calibration_stereo(tracker_instance_t* inst);


typedef struct tracker3D_calibration_stereo_instance
{
    tracker_stereo_configuration_t configuration;
    event_consumer_callback_func event_target_callback;
    void* event_target_instance; // where we send our events

    cv::Mat l_frame_gray;
    cv::Mat r_frame_gray;
    cv::Mat l_frame_u;
    cv::Mat l_frame_v;
    cv::Mat r_frame_u;
    cv::Mat r_frame_v;


    cv::Mat debug_rgb;

    // calibration data structures
    std::vector<std::vector<cv::Point3f>> chessboards_model;
    std::vector<std::vector<cv::Point2f>> l_chessboards_measured;
    std::vector<std::vector<cv::Point2f>> r_chessboards_measured;

    bool calibrated;

    bool l_alloced_frames;
    bool r_alloced_frames;

    bool got_left;
    bool got_right;

} tracker3D_calibration_stereo_instance_t;



tracker3D_calibration_stereo_instance_t*
tracker3D_calibration_stereo_create(tracker_instance_t* inst)
{
    tracker3D_calibration_stereo_instance_t* i =
        U_TYPED_CALLOC(tracker3D_calibration_stereo_instance_t);
    if (i) {
        i->l_alloced_frames = false;
        i->r_alloced_frames = false;
        i->got_left = false;
        i->got_right = false;


        // alloc our debug frame here - opencv is h,w, not w,h
        i->debug_rgb = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        return i;
    }
    return NULL;
}
bool
tracker3D_calibration_stereo_get_debug_frame(tracker_instance_t* inst,
                                        frame_t* frame)
{
    tracker3D_calibration_stereo_instance_t* internal =
        (tracker3D_calibration_stereo_instance_t*)inst->internal_instance;

    // wrap a frame struct around our debug cv::Mat and return it.

    frame->format = FORMAT_RGB_UINT8;
    frame->width = internal->debug_rgb.cols;
    frame->stride =
        internal->debug_rgb.cols * format_bytes_per_pixel(frame->format);
    frame->height = internal->debug_rgb.rows;
    frame->data = internal->debug_rgb.data;
    frame->size_bytes = frame_size_in_bytes(frame);
    return true;
}
capture_parameters_t
tracker3D_calibration_stereo_get_capture_params(tracker_instance_t* inst)
{
    tracker3D_calibration_stereo_instance_t* internal =
        (tracker3D_calibration_stereo_instance_t*)inst->internal_instance;
    capture_parameters_t cp = {};
    cp.exposure = 1.0 / 2048.0;
    cp.gain = 0.01f;
    return cp;
}

bool
tracker3D_calibration_stereo_queue(tracker_instance_t* inst, frame_t* frame)
{
    tracker3D_calibration_stereo_instance_t* internal =
        (tracker3D_calibration_stereo_instance_t*)inst->internal_instance;
    if (!inst->configured) {
        printf(
            "ERROR: you must configure this tracker before it can "
            "accept frames\n");
        return false;
    }

    // alloc left if required - if we have a composite stereo frame, alloc
    // both left and right eyes.

    if (frame->source_id == internal->configuration.l_source_id &&
        !internal->l_alloced_frames) {
        uint16_t eye_width = internal->configuration.r_rect.br.x - internal->configuration.r_rect.tl.x;
        if (internal->configuration.split_left == true) {

            internal->r_frame_gray =
                cv::Mat(frame->height, eye_width, CV_8UC1,
                        cv::Scalar(0, 0, 0));
            internal->r_frame_u =
                cv::Mat(frame->height, eye_width, CV_8UC1,
                        cv::Scalar(0, 0, 0));
            internal->r_frame_v =
                cv::Mat(frame->height, eye_width, CV_8UC1,
                        cv::Scalar(0, 0, 0));

            internal->r_alloced_frames = true;
        }
        eye_width = internal->configuration.l_rect.br.x - internal->configuration.l_rect.tl.x;
        internal->l_frame_gray = cv::Mat(frame->height, eye_width,
                                         CV_8UC1, cv::Scalar(0, 0, 0));
        internal->l_frame_u = cv::Mat(frame->height, eye_width, CV_8UC1,
                                      cv::Scalar(0, 0, 0));
        internal->l_frame_v = cv::Mat(frame->height, eye_width, CV_8UC1,
                                      cv::Scalar(0, 0, 0));
        internal->l_alloced_frames = true;
    }

    // if we have a right frame, alloc if required.

    if (frame->source_id == internal->configuration.r_source_id &&
        !internal->r_alloced_frames) {
        uint16_t eye_width = internal->configuration.r_rect.br.x - internal->configuration.r_rect.tl.x;
        internal->r_frame_gray = cv::Mat(frame->height, eye_width,
                                         CV_8UC1, cv::Scalar(0, 0, 0));
        internal->r_frame_u = cv::Mat(frame->height, eye_width, CV_8UC1,
                                      cv::Scalar(0, 0, 0));
        internal->r_frame_v = cv::Mat(frame->height, eye_width, CV_8UC1,
                                      cv::Scalar(0, 0, 0));
        internal->r_alloced_frames = true;
    }

    // copy our data from our video buffer into our cv::Mats
    // TODO: initialise once
    cv::Mat l_chans[3];
    l_chans[0] = internal->l_frame_gray;
    l_chans[1] = internal->l_frame_u;
    l_chans[2] = internal->l_frame_v;

    cv::Mat r_chans[3];
    r_chans[0] = internal->r_frame_gray;
    r_chans[1] = internal->r_frame_u;
    r_chans[2] = internal->r_frame_v;


    if (frame->source_id == internal->configuration.l_source_id) {

        if (internal->configuration.split_left == true) {
            internal->got_left = true;
            internal->got_right = true;
            cv::Mat tmp(frame->height, frame->width, CV_8UC3,
                        frame->data);
            cv::Rect lr(internal->configuration.l_rect.tl.x,
                        internal->configuration.l_rect.tl.y,
                        internal->configuration.l_rect.br.x,
                        internal->configuration.l_rect.br.y);
            cv::Rect rr(internal->configuration.r_rect.tl.x,
                        internal->configuration.r_rect.tl.y,
                        internal->configuration.r_rect.br.x -
                            internal->configuration.r_rect.tl.x,
                        internal->configuration.r_rect.br.y);
            cv::split(tmp(lr), l_chans);
            cv::split(tmp(rr), r_chans);
        } else {
            internal->got_left = true;
            cv::Mat tmp(frame->height, frame->width, CV_8UC3,
                        frame->data);
            cv::split(tmp, l_chans);
        }
    }
    if (frame->source_id == internal->configuration.r_source_id &&
        internal->configuration.split_left == false) {
        internal->got_right = true;
        cv::Mat tmp(frame->height, frame->width, CV_8UC3, frame->data);
        cv::split(tmp, r_chans);
    }

    // we have our pair of frames, now we can process them - we should do
    // this async, rather than in queue

    if (internal->got_left && internal->got_right) {
        return tracker3D_calibration_stereo_calibrate(inst);
    }
    return true;
}

bool
tracker3D_calibration_stereo_calibrate(tracker_instance_t* inst)
{

    printf("calibrating...\n");

    // check if we have saved calibration data. if so, just use it.
    tracker3D_calibration_stereo_instance_t* internal =
        (tracker3D_calibration_stereo_instance_t*)inst->internal_instance;

    // TODO: initialise this on construction and move this to internal state
    cv::Size board_size(8, 6);
    std::vector<cv::Point3f> chessboard_model;

    for (uint32_t i = 0; i < board_size.width * board_size.height; i++) {
        cv::Point3f p(i / board_size.width, i % board_size.width, 0.0f);
        chessboard_model.push_back(p);
    }

    cv::Mat l_chessboard_measured;
    cv::Mat r_chessboard_measured;


    // clear our debug image
    cv::rectangle(
        internal->debug_rgb, cv::Point2f(0, 0),
        cv::Point2f(internal->debug_rgb.cols, internal->debug_rgb.rows),
        cv::Scalar(0, 0, 0), -1, 0);
    cv::Mat disp8;
    cv::cvtColor(internal->r_frame_gray, disp8, CV_GRAY2BGR);
    // disp8.copyTo(internal->debug_rgb);
    // we will collect samples continuously - the user should be able to
    // wave a chessboard around randomly while the system calibrates.. we
    // only add a sample when it increases the coverage area substantially,
    // to give the solver a decent dataset.

    bool found_left = cv::findChessboardCorners(
        internal->l_frame_gray, board_size, l_chessboard_measured);
    bool found_right = cv::findChessboardCorners(
        internal->r_frame_gray, board_size, r_chessboard_measured);
    char message[128];
    message[0] = 0x0;

    if (found_left && found_right) {
        std::vector<cv::Point2f> coverage;
        for (uint32_t i = 0;
             i < internal->l_chessboards_measured.size(); i++) {
            cv::Rect brect = cv::boundingRect(
                internal->l_chessboards_measured[i]);
            cv::rectangle(internal->debug_rgb, brect.tl(),
                          brect.br(), cv::Scalar(0, 64, 32));

            coverage.push_back(cv::Point2f(brect.tl()));
            coverage.push_back(cv::Point2f(brect.br()));
        }
        cv::Rect pre_rect = cv::boundingRect(coverage);
        cv::Rect brect = cv::boundingRect(l_chessboard_measured);
        coverage.push_back(cv::Point2f(brect.tl()));
        coverage.push_back(cv::Point2f(brect.br()));
        cv::Rect post_rect = cv::boundingRect(coverage);

        // std::cout << "COVERAGE: " << brect.area() << "\n";

        cv::rectangle(internal->debug_rgb, post_rect.tl(),
                      post_rect.br(), cv::Scalar(0, 255, 0));

        if (post_rect.area() > pre_rect.area() + 500) {
            // we will use the last n samples to calculate our
            // calibration

            if (internal->l_chessboards_measured.size() >
                MAX_CALIBRATION_SAMPLES) {
                internal->l_chessboards_measured.erase(
                    internal->l_chessboards_measured.begin());
                internal->r_chessboards_measured.erase(
                    internal->r_chessboards_measured.begin());
            } else {
                internal->chessboards_model.push_back(
                    chessboard_model);
            }

            internal->l_chessboards_measured.push_back(
                l_chessboard_measured);
            internal->r_chessboards_measured.push_back(
                r_chessboard_measured);
        }

        if (internal->l_chessboards_measured.size() ==
            MAX_CALIBRATION_SAMPLES) {
            cv::Size image_size(internal->l_frame_gray.cols,
                                internal->l_frame_gray.rows);



            //we dont serialise these
            cv::Mat errors;
            cv::Mat camera_rotation;
            cv::Mat camera_translation;
            cv::Mat camera_essential;
            cv::Mat camera_fundamental;

            cv::Mat l_intrinsics;
            cv::Mat l_distortion;
            cv::Mat l_distortion_fisheye;
            cv::Mat l_translation;
            cv::Mat l_rotation;
            cv::Mat l_projection;
            cv::Mat r_intrinsics;
            cv::Mat r_distortion;
            cv::Mat r_distortion_fisheye;
            cv::Mat r_translation;
            cv::Mat r_rotation;
            cv::Mat r_projection;
            cv::Mat disparity_to_depth;

            cv::Mat zero_distortion = cv::Mat(DISTORTION_SIZE, 1, CV_32F, cv::Scalar(0.0f));

            // TODO: handle both fisheye and normal cameras -right
            // now I only have the fisheye

            float rp_error = cv::fisheye::stereoCalibrate(
                internal->chessboards_model,
                internal->l_chessboards_measured,
                internal->r_chessboards_measured,
                l_intrinsics,
                l_distortion_fisheye,
                r_intrinsics,
                r_distortion_fisheye, image_size,
                camera_rotation, camera_translation,
                cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC);

            //non-fisheye version
            // float rp_error =
            // cv::stereoCalibrate(internal->chessboards_model,internal->l_chessboards_measured,internal->r_chessboards_measured,internal->l_intrinsics,internal->l_distortion,internal->r_intrinsics,internal->r_distortion,image_size,camera_rotation,camera_translation,camera_essential,camera_fundamental,errors,0);

            std::cout << "calibration rp_error" << rp_error << "\n";
            std::cout << "calibration camera_translation"
                      << camera_translation << "\n";

            cv::stereoRectify(
                l_intrinsics, zero_distortion,
                r_intrinsics, zero_distortion,
                image_size, camera_rotation, camera_translation,
                l_rotation, r_rotation,
                l_projection, r_projection,
                disparity_to_depth,
                cv::CALIB_ZERO_DISPARITY);


            char path_string[PATH_MAX];
            char file_string[PATH_MAX];
            // TODO: centralise this - use multiple env vars?
            char* config_path = secure_getenv("HOME");
            snprintf(path_string, PATH_MAX, "%s/.config/monado",
                     config_path);
            snprintf(
                file_string, PATH_MAX,
                "%s/.config/monado/%s.calibration", config_path,
                internal->configuration.configuration_filename);

            printf("TRY WRITING CONFIG TO %s\n", file_string);
            FILE* calib_file = fopen(file_string, "wb");
            if (!calib_file) {
                // try creating it
                mkpath(path_string);
            }
            calib_file = fopen(file_string, "wb");
            if (!calib_file) {
                printf(
                    "ERROR. could not create calibration file "
                    "%s\n",
                    file_string);
            } else {
                write_mat(calib_file, &l_intrinsics);
                write_mat(calib_file, &r_intrinsics);
                write_mat(calib_file, &l_distortion);
                write_mat(calib_file, &r_distortion);
                write_mat(calib_file, &l_distortion_fisheye);
                write_mat(calib_file, &r_distortion_fisheye);
                write_mat(calib_file, &l_rotation);
                write_mat(calib_file, &r_rotation);
                write_mat(calib_file, &l_translation);
                write_mat(calib_file, &r_translation);
                write_mat(calib_file, &l_projection);
                write_mat(calib_file, &r_projection);
                write_mat(calib_file, &disparity_to_depth);

                cv::Mat mat_image_size;
                mat_image_size.create(1,2,CV_32F);
                mat_image_size.at<float>(0,0) = image_size.width;
                mat_image_size.at<float>(0,1) = image_size.height;
                write_mat(calib_file, &mat_image_size);

                fclose(calib_file);
            }

            printf("calibrated cameras!\n");
            internal->calibrated = true;
            driver_event_t e = {};
            e.type = EVENT_CALIBRATION_DONE;
            internal->event_target_callback(
                internal->event_target_instance, e);
        }

        snprintf(message, 128, "COLLECTING SAMPLE: %d/%d",
                 internal->l_chessboards_measured.size() + 1,
                 MAX_CALIBRATION_SAMPLES);
    }


    cv::drawChessboardCorners(internal->debug_rgb, board_size,
                              l_chessboard_measured, found_left);
    cv::drawChessboardCorners(internal->debug_rgb, board_size,
                              r_chessboard_measured, found_right);

    cv::putText(internal->debug_rgb, "CALIBRATION MODE",
                cv::Point2i(160, 240), 0, 1.0f, cv::Scalar(192, 192, 192));
    cv::putText(internal->debug_rgb, message, cv::Point2i(160, 460), 0,
                0.5f, cv::Scalar(192, 192, 192));

    // DEBUG: write out our image planes to confirm imagery is arriving as
    // expected
    /*cv::imwrite("/tmp/l_out_y.jpg",internal->l_frame_gray);
    cv::imwrite("/tmp/r_out_y.jpg",internal->r_frame_gray);
    cv::imwrite("/tmp/l_out_u.jpg",internal->l_frame_u);
    cv::imwrite("/tmp/r_out_u.jpg",internal->r_frame_u);
    cv::imwrite("/tmp/l_out_v.jpg",internal->l_frame_v);
    cv::imwrite("/tmp/r_out_v.jpg",internal->r_frame_v);*/
    tracker_send_debug_frame(inst);
    return true;
}


bool
tracker3D_calibration_stereo_get_poses(tracker_instance_t* inst,
                                  tracked_object_t* objects,
                                  uint32_t* count)
{
    return false;
}

bool
tracker3D_calibration_stereo_new_poses(tracker_instance_t* inst)
{
    return false;
}

bool
tracker3D_calibration_stereo_configure(tracker_instance_t* inst,
                                  tracker_stereo_configuration_t* config)
{
    tracker3D_calibration_stereo_instance_t* internal =
        (tracker3D_calibration_stereo_instance_t*)inst->internal_instance;
    // return false if we cannot handle this config

    if (config->l_format != FORMAT_YUV444_UINT8) {
        inst->configured = false;
        return false;
    }
    internal->configuration = *config;
    inst->configured=true;
    return true;
}

void
tracker3D_calibration_stereo_register_event_callback(
    tracker_instance_t* inst,
    void* target_instance,
    event_consumer_callback_func target_func)
{
    tracker3D_calibration_stereo_instance_t* internal =
        (tracker3D_calibration_stereo_instance_t*)inst->internal_instance;
    internal->event_target_instance = target_instance;
    internal->event_target_callback = target_func;
}


#include <opencv2/opencv.hpp>

#include "tracker3D_sphere_mono.h"
#include "common/opencv_utils.hpp"

#include "util/u_misc.h"

#define MAX_CALIBRATION_SAMPLES 23

static bool
tracker3D_sphere_mono_calibrate(tracker_instance_t* inst);

typedef struct tracker3D_calibration_mono_instance
{
    event_consumer_callback_func event_target_callback;
    void* event_target_instance; // where we send our measurements
    tracker_mono_configuration_t configuration;

    cv::Mat frame_gray;
    cv::Mat mask_gray;
    cv::Mat debug_rgb;

    // calibration data structures
    std::vector<std::vector<cv::Point3f>> chessboards_model;
    std::vector<std::vector<cv::Point2f>> chessboards_measured;

    bool alloced_frames;
} tracker3D_calibration_mono_instance_t;

tracker3D_calibration_mono_instance_t*
tracker3D_calibration_mono_create(tracker_instance_t* inst)
{
    tracker3D_calibration_mono_instance_t* i =
        U_TYPED_CALLOC(tracker3D_calibration_mono_instance_t);
    if (i) {
        i->alloced_frames = false;
        return i;
    }
    return NULL;
}
bool
tracker3D_calibration_mono_get_debug_frame(tracker_instance_t* inst, frame_t* frame)
{
    tracker3D_calibration_mono_instance_t* internal =
        (tracker3D_calibration_mono_instance_t*)inst->internal_instance;
    frame->format = FORMAT_RGB_UINT8;
    frame->width = internal->debug_rgb.cols;
    frame->stride =
        internal->debug_rgb.cols * format_bytes_per_pixel(frame->format);
    frame->height = internal->debug_rgb.rows;
    frame->data = internal->debug_rgb.data;
    frame->size_bytes = frame_size_in_bytes(frame);
    return true;
}
capture_parameters_t
tracker3D_calibration_mono_get_capture_params(tracker_instance_t* inst)
{
    capture_parameters_t cp = {};
    cp.exposure = 0.5f;
    cp.gain = 0.1f;
    return cp;
}

bool
tracker3D_calibration_mono_queue(tracker_instance_t* inst, frame_t* frame)
{
    tracker3D_calibration_mono_instance_t* internal =
        (tracker3D_calibration_mono_instance_t*)inst->internal_instance;
    if (!inst->configured) {
        printf(
            "ERROR: you must configure this tracker before it can "
            "accept frames\n");
        return false;
    }
    printf("received frame, tracking!\n");
    if (!internal->alloced_frames) {
        internal->frame_gray = cv::Mat(frame->height, frame->stride,
                                       CV_8UC1, cv::Scalar(0, 0, 0));
        internal->mask_gray = cv::Mat(frame->height, frame->stride,
                                      CV_8UC1, cv::Scalar(0, 0, 0));
        internal->debug_rgb = cv::Mat(frame->height, frame->width,
                                      CV_8UC3, cv::Scalar(0, 0, 0));
        internal->alloced_frames = true;
    }
    // we will just 'do the work' here.
    // TODO: asynchronous tracker thread

    memcpy(internal->frame_gray.data, frame->data, frame->size_bytes);
    return tracker3D_sphere_mono_calibrate(inst);
}

bool
tracker3D_calibration_mono_calibrate(tracker_instance_t* inst)
{
    printf("calibrating...\n");
    tracker3D_calibration_mono_instance_t* internal =
        (tracker3D_calibration_mono_instance_t*)inst->internal_instance;
    cv::Size image_size(internal->frame_gray.cols,
                        internal->frame_gray.rows);

    cv::Size board_size(8, 6);
    std::vector<cv::Point3f> chessboard_model;

    for (uint32_t i = 0; i < board_size.width * board_size.height; i++) {
        cv::Point3f p(i / board_size.width, i % board_size.width, 0.0f);
        chessboard_model.push_back(p);
    }

    cv::Mat intrinsics;
    cv::Mat distortion;
    cv::Mat distortion_fisheye;
    cv::Mat mat_image_size;

    cv::Mat chessboard_measured;

    // clear our debug image
    cv::rectangle(
        internal->debug_rgb, cv::Point2f(0, 0),
        cv::Point2f(internal->debug_rgb.cols, internal->debug_rgb.rows),
        cv::Scalar(0, 0, 0), -1, 0);

    // we will collect samples continuously - the user should be able to
    // wave a chessboard around randomly while the system calibrates..

    // TODO: we need a coverage measurement and an accuracy measurement,
    // so we can converge to something that is as complete and correct as
    // possible.

    bool found_board = cv::findChessboardCorners(
        internal->frame_gray, board_size, chessboard_measured);
    char message[128];
    message[0] = 0x0;

    if (found_board) {
        // we will use the last n samples to calculate our calibration
        if (internal->chessboards_measured.size() >
            MAX_CALIBRATION_SAMPLES) {
            internal->chessboards_measured.erase(
                internal->chessboards_measured.begin());
        } else {
            internal->chessboards_model.push_back(chessboard_model);
        }

        internal->chessboards_measured.push_back(chessboard_measured);

        if (internal->chessboards_measured.size() ==
            MAX_CALIBRATION_SAMPLES) {
            // TODO - run this if coverage test passes
            cv::Mat rvecs, tvecs;

            float rp_error = cv::calibrateCamera(
                internal->chessboards_model,
                internal->chessboards_measured, image_size,
                intrinsics, distortion, rvecs,
                tvecs);


            char path_string[PATH_MAX];
            char file_string[PATH_MAX];
            // TODO: use multiple env vars?
            char* config_path = secure_getenv("HOME");
            snprintf(path_string, PATH_MAX, "%s/.config/monado",
                     config_path);
            snprintf(
                file_string, PATH_MAX,
                "%s/.config/monado/%s.calibration", config_path,
                internal->configuration.configuration_filename);

            printf("TRY WRITING CONFIG TO %s\n", file_string);
            FILE* calib_file = fopen(file_string, "wb");
            if (!calib_file) {
                mkpath(path_string);
            }
            calib_file = fopen(file_string, "wb");
            if (!calib_file) {
                printf(
                    "ERROR. could not create calibration file "
                    "%s\n",
                    file_string);
            } else {

                mat_image_size.create(1,2,CV_32F);
                mat_image_size.at<float>(0,0) = image_size.width;
                mat_image_size.at<float>(0,0) = image_size.height;
                write_mat(calib_file, &intrinsics);
                write_mat(calib_file, &distortion);
                write_mat(calib_file, &distortion_fisheye);
                write_mat(calib_file, &mat_image_size);
                fclose(calib_file);
            }

            driver_event_t e = {};
            e.type = EVENT_TRACKER_RECONFIGURED;
            internal->event_target_callback(
                internal->event_target_instance, e);
        } else {
            snprintf(message, 128, "COLLECTING SAMPLE: %d/%d",
                     internal->chessboards_measured.size() + 1,
                     MAX_CALIBRATION_SAMPLES);
        }
    }

    cv::drawChessboardCorners(internal->debug_rgb, board_size,
                              chessboard_measured, found_board);

    cv::putText(internal->debug_rgb, "CALIBRATION MODE",
                cv::Point2i(160, 240), 0, 1.0f, cv::Scalar(192, 192, 192));
    cv::putText(internal->debug_rgb, message, cv::Point2i(160, 460), 0,
                0.5f, cv::Scalar(192, 192, 192));

    tracker_send_debug_frame(inst);

    return true;
}


bool
tracker3D_calibration_mono_configure(tracker_instance_t* inst,
                                tracker_mono_configuration_t* config)
{
    tracker3D_calibration_mono_instance_t* internal =
        (tracker3D_calibration_mono_instance_t*)inst->internal_instance;
    // return false if we cannot handle this config

    if (config->format != FORMAT_Y_UINT8) {
        inst->configured = false;
        return false;
    }
    internal->configuration = *config;
    inst->configured = true;
    return true;
}


void
tracker3D_calibration_mono_register_event_callback(
    tracker_instance_t* inst,
    void* target_instance,
    event_consumer_callback_func target_func)
{
    tracker3D_calibration_mono_instance_t* internal = (tracker3D_calibration_mono_instance_t*)inst->internal_instance;
    internal->event_target_instance = target_instance;
    internal->event_target_callback = target_func;
}


bool
tracker3D_calibration_get_poses(tracker_instance_t* inst,
                                tracked_object_t* objects,
                                uint32_t* count)
{
    return false;
}

bool
tracker3D_calibration_new_poses(tracker_instance_t* inst)
{
    return false;
}

void
tracker3D_calibration_register_measurement_callback(
    tracker_instance_t* /*inst*/,
    void* /*target_instance*/,
    measurement_consumer_callback_func /*target_func*/)
{
    //do nothing
}
