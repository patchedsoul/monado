#include <opencv2/opencv.hpp>

#include "tracker3D_psvr_stereo.h"
#include "common/calibration_opencv.hpp"
#include "common/opencv_utils.hpp"

#include <sys/stat.h>
#include <sys/time.h>
#include <linux/limits.h>

#include "util/u_misc.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

static bool
tracker3D_psvr_stereo_track(tracker_instance_t* inst);


typedef struct psvr_track_data
{
    uint64_t timestamp;
    struct xrt_vec3 positions_3d[NUM_LEDS]; // x,y,z position for up to (currently)
                                     // 9 points - LU,RU,C,LL,RL,LS,RS,LB,RB
    struct xrt_vec2 l_positions_2d[NUM_LEDS]; // 2d positions in left and right images
    struct xrt_vec2 r_positions_2d[NUM_LEDS];
    int8_t confidence[NUM_LEDS]; //-1 if point is not tracked, TODO: 0-128
                                 // for confidence
    uint8_t tracked_count; //count of leds with confidence >= 1
    float longest_dist;
    struct xrt_matrix_3x3 rotation_matrix; // SVD-fitted head rotation matrix
    struct xrt_vec3 translation; // head translation

} psvr_track_data_t;


typedef struct point_dist
{
    uint32_t index;
    struct xrt_vec3 translation;
    float length;
    float angle;
} point_dist_t;


typedef struct psvr_led
{
    uint32_t world_point_index;
    struct xrt_vec2 l_position_2d;
    struct xrt_vec2 r_position_2d;
    float radius;
    cv::Point2f distance;
    struct xrt_vec3 position;
    struct xrt_vec3 scaled_position;
    std::vector<point_dist> point_dists;
    float rms_distance;
    float longest_distance;
    int sign_x;
    int sign_y;
} psvr_led_t;


typedef struct tracker3D_psvr_stereo_instance
{
    tracker_stereo_configuration_t configuration;
    measurement_consumer_callback_func measurement_target_callback;
    void* measurement_target_instance; // where we send our measurements
    event_consumer_callback_func event_target_callback;
    void* event_target_instance; // where we send our events

    tracked_object_t tracked_object;
    tracked_blob_t l_tracked_blob;
    tracked_blob_t r_tracked_blob;

    bool poses_consumed;
    cv::SimpleBlobDetector::Params blob_params;
    std::vector<cv::KeyPoint> l_keypoints;
    std::vector<cv::KeyPoint> r_keypoints;
    // these components hold no state so we can use a single instance for l
    // and r ?
    cv::Ptr<cv::SimpleBlobDetector> sbd;
    cv::Ptr<cv::BackgroundSubtractorMOG2> background_subtractor;
    cv::Mat l_frame_gray;
    cv::Mat r_frame_gray;
    cv::Mat l_mask_gray;
    cv::Mat r_mask_gray;

    cv::Mat l_frame_u;
    cv::Mat l_frame_v;
    cv::Mat r_frame_u;
    cv::Mat r_frame_v;


    cv::Mat debug_rgb;

    cv::Mat disparity_to_depth;

    cv::Mat l_undistort_map_x;
    cv::Mat l_undistort_map_y;
    cv::Mat r_undistort_map_x;
    cv::Mat r_undistort_map_y;

    cv::Mat l_rectify_map_x;
    cv::Mat l_rectify_map_y;
    cv::Mat r_rectify_map_x;
    cv::Mat r_rectify_map_y;

    bool calibrated;
    room_setup rs;

    bool l_alloced_frames;
    bool r_alloced_frames;

    bool got_left;
    bool got_right;

    psvr_track_data_t prev_track_data;
    psvr_led_t model_leds[NUM_LEDS];

} tracker3D_psvr_stereo_instance_t;

static bool
psvr_disambiguate_5points(std::vector<psvr_led_t>* leds, psvr_track_data_t* t);

static bool psvr_compute_svd(psvr_track_data_t* t);
static bool psvr_remove_outliers(std::vector<psvr_led_t>* leds);
static bool psvr_classify_leds(tracker_instance_t* inst,std::vector<psvr_led_t>* leds,psvr_track_data_t* t);

static void psvr_blob_to_led(std::vector<cv::Point3f>* world_points, psvr_led_t* led_data);


tracker3D_psvr_stereo_instance_t*
tracker3D_psvr_stereo_create(tracker_instance_t* inst)
{
    tracker3D_psvr_stereo_instance_t* i =
        U_TYPED_CALLOC(tracker3D_psvr_stereo_instance_t);
    if (i) {
        i->blob_params.filterByArea = false;
        i->blob_params.filterByConvexity = false;
        i->blob_params.filterByInertia = false;
        i->blob_params.filterByColor = true;
        i->blob_params.blobColor =
            255; // 0 or 255 - color comes from binarized image?
        i->blob_params.minArea = 1;
        i->blob_params.maxArea = 1000;
        i->blob_params.maxThreshold =
            51; // using a wide threshold span slows things down bigtime
        i->blob_params.minThreshold = 50;
        i->blob_params.thresholdStep = 1;
        i->blob_params.minDistBetweenBlobs = 5;
        i->blob_params.minRepeatability = 1; // need this to avoid
                                             // error?

        i->sbd = cv::SimpleBlobDetector::create(i->blob_params);
        i->background_subtractor =
            cv::createBackgroundSubtractorMOG2(32, 16, false);

        i->poses_consumed = false;
        i->l_alloced_frames = false;
        i->r_alloced_frames = false;
        i->got_left = false;
        i->got_right = false;

        i->calibrated = false;


        // alloc our debug frame here - opencv is h,w, not w,h
        i->debug_rgb = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));

        std::vector<cv::Point3f> model_points;
        for (uint32_t j=0;j<NUM_LEDS;j++)
        {
            model_points.push_back(cv::Vec3f(physical_led_positions[j].x,physical_led_positions[j].y,physical_led_positions[j].z));

        }
        for (uint32_t j=0;j<NUM_LEDS;j++)
        {
           i->model_leds[j].world_point_index =j; //this index must be set
           psvr_blob_to_led(&model_points,&i->model_leds[j]);
        }


        return i;
    }
    return NULL;
}
bool
tracker3D_psvr_stereo_get_debug_frame(tracker_instance_t* inst,
                                        frame_t* frame)
{
    tracker3D_psvr_stereo_instance_t* internal =
        (tracker3D_psvr_stereo_instance_t*)inst->internal_instance;

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
tracker3D_psvr_stereo_get_capture_params(tracker_instance_t* inst)
{
    tracker3D_psvr_stereo_instance_t* internal =
        (tracker3D_psvr_stereo_instance_t*)inst->internal_instance;
    capture_parameters_t cp = {};
    cp.exposure = 1.0 / 2048.0;
    cp.gain = 0.01f;
    return cp;
}

bool
tracker3D_psvr_stereo_queue(tracker_instance_t* inst, frame_t* frame)
{
    tracker3D_psvr_stereo_instance_t* internal =
        (tracker3D_psvr_stereo_instance_t*)inst->internal_instance;
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
        uint16_t eye_width = frame->width / 2;
        if (internal->configuration.split_left == true) {
            eye_width = frame->width / 2;
            internal->r_frame_gray =
                cv::Mat(frame->height, eye_width, CV_8UC1,
                        cv::Scalar(0, 0, 0));
            internal->r_frame_u =
                cv::Mat(frame->height, eye_width, CV_8UC1,
                        cv::Scalar(0, 0, 0));
            internal->r_frame_v =
                cv::Mat(frame->height, eye_width, CV_8UC1,
                        cv::Scalar(0, 0, 0));
            internal->r_mask_gray =
                cv::Mat(frame->height, eye_width, CV_8UC1,
                        cv::Scalar(0, 0, 0));
            internal->r_alloced_frames = true;
        }
        internal->l_frame_gray = cv::Mat(frame->height, eye_width,
                                         CV_8UC1, cv::Scalar(0, 0, 0));
        internal->l_frame_u = cv::Mat(frame->height, eye_width, CV_8UC1,
                                      cv::Scalar(0, 0, 0));
        internal->l_frame_v = cv::Mat(frame->height, eye_width, CV_8UC1,
                                      cv::Scalar(0, 0, 0));

        internal->l_mask_gray = cv::Mat(frame->height, eye_width,
                                        CV_8UC1, cv::Scalar(0, 0, 0));
        internal->l_alloced_frames = true;
    }

    // if we have a right frame, alloc if required.

    if (frame->source_id == internal->configuration.r_source_id &&
        !internal->r_alloced_frames) {
        uint16_t eye_width = frame->width / 2;
        internal->r_frame_gray = cv::Mat(frame->height, eye_width,
                                         CV_8UC1, cv::Scalar(0, 0, 0));
        internal->r_frame_u = cv::Mat(frame->height, eye_width, CV_8UC1,
                                      cv::Scalar(0, 0, 0));
        internal->r_frame_v = cv::Mat(frame->height, eye_width, CV_8UC1,
                                      cv::Scalar(0, 0, 0));

        internal->r_mask_gray = cv::Mat(frame->height, eye_width,
                                        CV_8UC1, cv::Scalar(0, 0, 0));
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
        return tracker3D_psvr_stereo_track(inst);
    }
    return true;
}

bool
tracker3D_psvr_stereo_track(tracker_instance_t* inst)
{

    tracker3D_psvr_stereo_instance_t* internal =
        (tracker3D_psvr_stereo_instance_t*)inst->internal_instance;
    internal->l_keypoints.clear();
    internal->r_keypoints.clear();


    // DEBUG: dump all our planes out for inspection
   /* cv::imwrite("/tmp/l_out_y.jpg",internal->l_frame_gray);
    cv::imwrite("/tmp/r_out_y.jpg",internal->r_frame_gray);
    cv::imwrite("/tmp/l_out_u.jpg",internal->l_frame_u);
    cv::imwrite("/tmp/r_out_u.jpg",internal->r_frame_u);
    cv::imwrite("/tmp/l_out_v.jpg",internal->l_frame_v);
    cv::imwrite("/tmp/r_out_v.jpg",internal->r_frame_v);*/


    // disabled channel combining in favour of using v plane directly - Y is
    // 2x resolution
    // so we do want to use that eventually

    // combine our yuv channels to isolate blue leds - y channel is all we
    // will use from now on
    // cv::subtract(internal->l_frame_u,internal->l_frame_v,internal->l_frame_u);
    // cv::subtract(internal->r_frame_u,internal->r_frame_v,internal->r_frame_u);

    // cv::subtract(internal->l_frame_u,internal->l_frame_gray,internal->l_frame_gray);
    // cv::subtract(internal->r_frame_u,internal->r_frame_gray,internal->r_frame_gray);

    // just use the u plane directly
    cv::bitwise_not(internal->l_frame_v, internal->l_frame_v);
    cv::bitwise_not(internal->r_frame_v, internal->r_frame_v);

    cv::threshold(internal->l_frame_v, internal->l_frame_gray, 150.0, 255.0,
                  0);
    cv::threshold(internal->r_frame_v, internal->r_frame_gray, 150.0, 255.0,
                  0);


    cv::Mat l_frame_undist;
    cv::Mat r_frame_undist;

    // undistort the whole image
    cv::remap(internal->l_frame_gray, l_frame_undist,
              internal->l_undistort_map_x, internal->l_undistort_map_y,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    cv::remap(internal->r_frame_gray, r_frame_undist,
              internal->r_undistort_map_x, internal->r_undistort_map_y,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // rectify the whole image
    cv::remap(l_frame_undist, internal->l_frame_gray,
              internal->l_rectify_map_x, internal->l_rectify_map_y,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    cv::remap(r_frame_undist, internal->r_frame_gray,
              internal->r_rectify_map_x, internal->r_rectify_map_y,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

   // block-match for disparity calculation - disabled

    cv::Mat disp;
    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(128, 5);
    sbm->setNumDisparities(64);
    sbm->setBlockSize(5);
    sbm->setPreFilterCap(61);
    sbm->setPreFilterSize(15);
    sbm->setTextureThreshold(32);
    sbm->setSpeckleWindowSize(100);
    sbm->setSpeckleRange(32);
    sbm->setMinDisparity(2);
    sbm->setUniquenessRatio(10);
    sbm->setDisp12MaxDiff(0);
    //	sbm->compute(internal->l_frame_gray,
    // internal->r_frame_gray,disp); 	cv::normalize(disp, disp8, 0.1,
    // 255, CV_MINMAX, CV_8UC1);


    // disabled background subtraction for now

    // internal->background_subtractor->apply(internal->l_frame_gray,internal->l_mask_gray);
    // internal->background_subtractor->apply(internal->r_frame_gray,internal->r_mask_gray);

    xrt_vec2 lastPos = internal->l_tracked_blob.center;
    float offset = ROI_OFFSET;
    if (internal->l_tracked_blob.diameter > ROI_OFFSET) {
        offset = internal->l_tracked_blob.diameter;
    }

    // cv::rectangle(internal->l_mask_gray,
    // cv::Point2f(lastPos.x-offset,lastPos.y-offset),cv::Point2f(lastPos.x+offset,lastPos.y+offset),cv::Scalar(
    // 255 ),-1,0); lastPos = internal->r_tracked_blob.center;
    // cv::rectangle(internal->r_mask_gray,
    // cv::Point2f(lastPos.x-offset,lastPos.y-offset),cv::Point2f(lastPos.x+offset,lastPos.y+offset),cv::Scalar(
    // 255 ),-1,0);

    // cv::rectangle(internal->debug_rgb,
    // cv::Point2f(0,0),cv::Point2f(internal->debug_rgb.cols,internal->debug_rgb.rows),cv::Scalar(
    // 0,0,0 ),-1,0);

    cv::threshold(internal->l_frame_gray, internal->l_frame_gray, 32.0,
                  255.0, 0);
    cv::threshold(internal->r_frame_gray, internal->r_frame_gray, 32.0,
                  255.0, 0);

    // TODO: handle source images larger than debug_rgb
    cv::Mat debug_img;
    cv::cvtColor(internal->l_frame_gray, debug_img, CV_GRAY2BGR);
    int32_t cropped_rows = debug_img.rows;
    int32_t cropped_cols = debug_img.cols;

    if (debug_img.rows > internal->debug_rgb.rows || debug_img.cols > internal->debug_rgb.cols)
    {
        cropped_rows = internal->debug_rgb.rows;
        cropped_cols = internal->debug_rgb.cols;

    }
    cv::Mat dst_roi =
        internal->debug_rgb(cv::Rect(0, 0, cropped_cols, cropped_rows));
    debug_img.copyTo(dst_roi);


    tracker_measurement_t m = {};

    // do blob detection with our masks
    internal->sbd->detect(internal->l_frame_gray,
                          internal->l_keypoints); //,internal->l_mask_gray);
    internal->sbd->detect(internal->r_frame_gray,
                          internal->r_keypoints); //,internal->r_mask_gray);
    // do some basic matching to come up with likely disparity-pairs
    std::vector<cv::KeyPoint> l_blobs, r_blobs;
    for (uint32_t i = 0; i < internal->l_keypoints.size(); i++) {
        cv::KeyPoint l_blob = internal->l_keypoints[i];
        int l_index = -1;
        int r_index = -1;
        for (uint32_t j = 0; j < internal->r_keypoints.size(); j++) {
            float lowest_dist = 128;
            cv::KeyPoint r_blob = internal->r_keypoints[j];
            // find closest point on same-ish scanline
            if ((l_blob.pt.y < r_blob.pt.y + 3) &&
                (l_blob.pt.y > r_blob.pt.y - 3) &&
                ((r_blob.pt.x - l_blob.pt.x) < lowest_dist)) {
                lowest_dist = r_blob.pt.x - l_blob.pt.x;
                r_index = j;
                l_index = i;
            }
        }
        if (l_index > -1 && r_index > -1) {
            l_blobs.push_back(internal->l_keypoints.at(l_index));
            r_blobs.push_back(internal->r_keypoints.at(r_index));
        }
    }

    // draw our disparity markers into our debug frame

    for (uint32_t i = 0; i < l_blobs.size(); i++) {
        cv::line(internal->debug_rgb, l_blobs[i].pt, r_blobs[i].pt,
                 cv::Scalar(255, 0, 0));
    }

    // convert our 2d point + disparities into 3d points.

    std::vector<cv::Point3f> world_points;
    if (l_blobs.size() > 0) {
        for (uint32_t i = 0; i < l_blobs.size(); i++) {
            float disp = r_blobs[i].pt.x - l_blobs[i].pt.x;
            cv::Vec4d xydw(l_blobs[i].pt.x, l_blobs[i].pt.y, disp,
                           1.0f);
            // Transform
            cv::Vec4d h_world = (cv::Matx44d)internal->disparity_to_depth * xydw;
            // Divide by scale to get 3D vector from homogeneous
            // coordinate. invert x while we are at it.
            world_points.push_back(cv::Point3f(
                -h_world[0] / h_world[3], h_world[1] / h_world[3],
                h_world[2] / h_world[3]));
        }
    }

    // initial implementation simply tracks a single blob.
    // keep the last position of the blob, and use the closest
    // 'new' blob so noise or random reflections etc. have
    // less chance of throwing tracking off.

    int tracked_index = -1;
    float lowest_dist = 65535.0f;
    xrt_vec3 position = internal->tracked_object.pose.position;
    cv::Point3f last_point(position.x, position.y, position.z);

    for (uint32_t i = 0; i < world_points.size(); i++) {
        cv::Point3f world_point = world_points[i];
        // show our tracked world points (just x,y) in our debug output
        cv::Point2f img_point;
        img_point.x = world_point.x * internal->debug_rgb.cols / 2 +
                      internal->debug_rgb.cols / 2;
        img_point.y = world_point.y * internal->debug_rgb.rows / 2 +
                      internal->debug_rgb.rows / 2;
        cv::circle(internal->debug_rgb, img_point, 3,
                   cv::Scalar(0, 255, 0));

        float dist = cv_dist_3d(world_point, last_point);
        if (dist < lowest_dist) {
            tracked_index = i;
            lowest_dist = dist;
        }
    }

    std::vector<psvr_led_t> led_data;
    psvr_track_data_t track_data;
    memset(track_data.confidence,0,NUM_LEDS * sizeof(char));

    // longest length gives us the highest distance between any 2 points.
    // scaling by this value gives us a normalised distance.
    for (uint32_t i=0;i<world_points.size();i++) {
        cv::Point3f point_a = world_points[i];
        psvr_led_t l;
        l.sign_x=0;
        l.sign_y=0;
        l.l_position_2d.x = l_blobs[i].pt.x;
        l.l_position_2d.y = l_blobs[i].pt.y;
        l.r_position_2d.x = r_blobs[i].pt.x;
        l.r_position_2d.y = r_blobs[i].pt.y;

        l.position.x=point_a.x;
        l.position.y=point_a.y;
        l.position.z=point_a.z;

        l.world_point_index = i;

        // fill in our led struct with distances etc.
        psvr_blob_to_led(&world_points, &l);

        l.rms_distance = sqrt(l.rms_distance);
        //printf("longest distance %f\n", l.longest_distance);
        led_data.push_back(l);
    }

    psvr_remove_outliers(&led_data);

    psvr_classify_leds(inst,&led_data,&track_data);

    //draw our points in the debug output
    for (uint32_t i=0;i<NUM_LEDS;i++)
    {
        cv::Point2f pos(track_data.l_positions_2d[i].x,track_data.l_positions_2d[i].y);
        cv::circle(internal->debug_rgb,pos,3,cv::Scalar(96,128,192));
        char label[128];
        snprintf(label, 128, "LED %d %f %f %f", i,track_data.positions_3d[i].x,track_data.positions_3d[i].y,track_data.positions_3d[i].z);
        cv::putText(internal->debug_rgb,label,pos,0, 0.3f, cv::Scalar(255, 192, 0));
    }

    if (track_data.tracked_count> 2) {
    psvr_compute_svd(&track_data);
    Eigen::Matrix3f rot_mat = Eigen::Map<Eigen::Matrix<float,3,3>>(track_data.rotation_matrix.v);
    Eigen::Quaternionf q(rot_mat);
    Eigen::Vector3f ea = rot_mat.eulerAngles(0, 1, 2);

    float rad2deg = 180.0f/EIGEN_PI;

    std::cout << "QUAT: " <<  q.vec() << q.w() << "\n";
    std::cout << "ANGLE: " <<  ea.x() * rad2deg << "," <<ea.y() * rad2deg << "," << ea.z() * rad2deg  << "\n";
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    }


    if (tracked_index != -1) {
        cv::Point3f world_point = world_points[tracked_index];

        // create our measurement for the filter
        m.flags = (tracker_measurement_flags_t)(MEASUREMENT_POSITION | MEASUREMENT_OPTICAL);
        struct timeval tp;
        gettimeofday(&tp,NULL);
        m.source_timestamp = tp.tv_sec * 1000000 + tp.tv_usec;

        //apply our room setup transforms
        Eigen::Vector3f p = Eigen::Map<Eigen::Vector3f>(&track_data.translation.x);
        Eigen::Vector4f pt;
        pt.x() = p.x();
        pt.y() = p.y();
        pt.z() = p.z();
        pt.w() = 1.0f;

        //this is a glm mat4 written out 'flat'
        Eigen::Matrix4f mat = Eigen::Map<Eigen::Matrix<float,4,4>>(internal->rs.origin_transform.v);
        pt = mat * pt;

        m.pose.position.x = pt.x();
        m.pose.position.y = pt.y();
        m.pose.position.z = pt.z();

        // update internal state
        cv::KeyPoint l_kp = l_blobs[tracked_index];
        cv::KeyPoint r_kp = l_blobs[tracked_index];

        internal->l_tracked_blob.center.x = l_kp.pt.x;
        internal->l_tracked_blob.center.y = l_kp.pt.y;
        internal->l_tracked_blob.diameter = l_kp.size;

        internal->r_tracked_blob.center.x = r_kp.pt.x;
        internal->r_tracked_blob.center.y = r_kp.pt.y;
        internal->r_tracked_blob.diameter = r_kp.size;

        internal->tracked_object.pose.position.x = world_point.x;
        internal->tracked_object.pose.position.y = world_point.y;
        internal->tracked_object.pose.position.z = world_point.z;
        internal->tracked_object.tracking_id = 1;





        char message[128];
        snprintf(message, 128, "X: %f Y: %f Z: %f", world_point.x,
                 world_point.y, world_point.z);

        cv::putText(internal->debug_rgb, message, cv::Point2i(10, 50),
                    0, 0.5f, cv::Scalar(96, 128, 192));
        if (internal->measurement_target_callback) {
            //printf("STTR queueing timestamp %lld\n",m.source_timestamp);
              //printf("X: %f Y: %f Z: %f mx: %f my: %f mz: %f\n", world_point.x,
              //       world_point.y, world_point.z,m.pose.position.x,m.pose.position.y,m.pose.position.z);
            internal->measurement_target_callback(
                internal->measurement_target_instance, &m);
        }
    }

    tracker_send_debug_frame(inst); // publish our debug frame


    return true;
}

bool
tracker3D_psvr_stereo_get_poses(tracker_instance_t* inst,
                                  tracked_object_t* objects,
                                  uint32_t* count)
{
    if (objects == NULL) {
        *count = TRACKED_POINTS; // tracking a single object
        return true;
    }

    tracker3D_psvr_stereo_instance_t* internal =
        (tracker3D_psvr_stereo_instance_t*)inst->internal_instance;
    for (uint32_t i = 0; i < 1; i++) {

        objects[i] = internal->tracked_object;
    }
    *count = 1;
    internal->poses_consumed = true;
    return true;
}

bool
tracker3D_psvr_stereo_new_poses(tracker_instance_t* inst)
{
    tracker3D_psvr_stereo_instance_t* internal =
        (tracker3D_psvr_stereo_instance_t*)inst->internal_instance;
    return internal->poses_consumed;
}

bool
tracker3D_psvr_stereo_configure(tracker_instance_t* inst,
                                  tracker_stereo_configuration_t* config)
{
    tracker3D_psvr_stereo_instance_t* internal =
        (tracker3D_psvr_stereo_instance_t*)inst->internal_instance;
    // return false if we cannot handle this config

    if (config->l_format != FORMAT_YUV444_UINT8) {
        inst->configured = false;
        return false;
    }

    //load calibration data

    if (! internal->calibrated) {
        if ( calibration_get_stereo(config->camera_configuration_filename,true,
                               &internal->l_undistort_map_x,
                               &internal->l_undistort_map_y,
                               &internal->l_rectify_map_x,
                               &internal->l_rectify_map_y,
                               &internal->r_undistort_map_x,
                               &internal->r_undistort_map_y,
                               &internal->r_rectify_map_x,
                               &internal->r_rectify_map_y,
                               &internal->disparity_to_depth) ) {
            printf("loaded calibration for camera!\n");

          }
        if (! calibration_get_roomsetup(config->room_setup_filename,&internal->rs))
        {
            printf("Could not load room setup. tracking frame will be b0rked.\n");
        }

            internal->calibrated = true;


    }

    internal->configuration = *config;
    inst->configured=true;
    return true;
}

void
tracker3D_psvr_stereo_register_measurement_callback(
    tracker_instance_t* inst,
    void* target_instance,
    measurement_consumer_callback_func target_func)
{
    tracker3D_psvr_stereo_instance_t* internal =
        (tracker3D_psvr_stereo_instance_t*)inst->internal_instance;
    internal->measurement_target_instance = target_instance;
    internal->measurement_target_callback = target_func;
}

void
tracker3D_psvr_stereo_register_event_callback(
    tracker_instance_t* inst,
    void* target_instance,
    event_consumer_callback_func target_func)
{
    tracker3D_psvr_stereo_instance_t* internal =
        (tracker3D_psvr_stereo_instance_t*)inst->internal_instance;
    internal->event_target_instance = target_instance;
    internal->event_target_callback = target_func;
}


bool psvr_remove_outliers(std::vector<psvr_led_t>* leds)
{
    return false;
}


void psvr_blob_to_led(std::vector<cv::Point3f>* world_points, psvr_led_t* led_data)
{
    printf("world points: %d\n",world_points->size());
    cv::Point3f point_a = world_points->at(led_data->world_point_index);
    led_data->longest_distance = 0.0f;

    cv::Point3f centroid(0.0f,0.0f,0.0f);
    for (uint32_t i=0;i<world_points->size();i++) {
        centroid.x += world_points->at(i).x;
        centroid.y += world_points->at(i).y;
        centroid.z += world_points->at(i).z;
    }
    centroid.x /= world_points->size();
    centroid.y /= world_points->size();
    centroid.z /= world_points->size();

    printf("CENTROID: %f %f %f\n",centroid.x,centroid.y,centroid.z);


    for (uint32_t j=0;j<world_points->size();j++) {
        if (led_data->world_point_index != j) {
            cv::Point3f point_b = world_points->at(j);
            point_dist pd;
            pd.index = j;
            pd.translation.x = point_b.x - point_a.x;
            pd.translation.y = point_b.y - point_a.y;
            pd.translation.z = point_b.z - point_a.z;
            pd.length = cv_dist_3d(point_b,point_a);
            cv::Vec3f pa,pb,cr;
            pa = cv::normalize((cv::Vec3f)(point_a - centroid));
            pb = cv::normalize((cv::Vec3f)(point_b - centroid));
            cv::Point3f n = pa.cross(pb);
            cv::Vec3f o(0.0f,0.0f,0.0f);
            cr = pa.cross(pb);
            float s = cv_dist_3d(o,cr);
            float c = pa.dot(pb);
            pd.angle = atan2(s, c);


            //pd.angle = acos(pa.dot(pb));
            //if (pa.cross(pb) < 0) {
            //    pd.angle *=-1;
            //}


            std::cout << "raw a: " << point_a << " raw b: " << point_b << " pA: " << pa << " pB: " << pb << " NORM: " << n << " ANGLE: " << pd.angle << "\n";


            led_data->rms_distance += pd.length * pd.length;
            //accumulate the signs of the x and y translations
            //so we can determine the 'center' led
            led_data->sign_x += copysignf(1.0, pd.translation.x);
            led_data->sign_y += copysignf(1.0, pd.translation.y);
            float absl = abs(pd.length);
            //determine the longest distance - used to normalise
            //distances
            if ( absl > led_data->longest_distance) {
                led_data->longest_distance = absl;
            }
            led_data->point_dists.push_back(pd);
            //printf ("pA index %d pB index %d trans: %f %f %f %f %d %d\n",i,j,pd.translation.x,pd.translation.y,pd.translation.z,pd.length,l.sign_x,l.sign_y);
         }
    }
}

bool psvr_classify_leds(tracker_instance_t* inst,std::vector<psvr_led_t>* leds,psvr_track_data_t* t) {
    tracker3D_psvr_stereo_instance_t* internal =
        (tracker3D_psvr_stereo_instance_t*)inst->internal_instance;
    printf("CLASSIFY LEDS ----\n");
    for (uint32_t i=0;i<leds->size();i++) {
        psvr_led_t l = leds->at(i);
        //printf("led %f %d %d\n",l.rms_distance,l.sign_x,l.sign_y);
        float lowest_err=65535.0f;
        float confidence[NUM_LEDS];
        memset (confidence,0,sizeof(float) * NUM_LEDS);
        uint32_t matched_led;
        for (uint32_t j=0;j<l.point_dists.size();j++) {
            point_dist_t measured_pd = l.point_dists[j];
            printf("measured angle %f \n",-measured_pd.angle);
            for (uint32_t k=0;k<NUM_LEDS;k++)
            {
                float lowest_diff=65535.0f;
                for (uint32_t m=0;m<internal->model_leds[k].point_dists.size();m++) {
                point_dist_t model_pd = internal->model_leds[k].point_dists[m];
                printf("%d %d model angle: %f\n",k,m,model_pd.angle);
                float diff = -measured_pd.angle - model_pd.angle;
                confidence[k] += diff;
                }

            }

        }
    printf("confidence %f %f %f %f %f\n",confidence[0],confidence[1],confidence[2],confidence[3],confidence[4]);
    }
    printf("----\n");



    //if we have 5 leds visible, we can use our geometric approach
    if (leds->size() ==5) {
        psvr_disambiguate_5points(leds,t);
    } else {
        // we dont have 5 - we can at least assume that the leds are the same as the closest led in the previous frame

        // if we dont have previous track data, we cannot continue.
        if (internal->prev_track_data.tracked_count == 0) {
        //    return false;
        }
        t->longest_dist = 0.0f;
        for (uint32_t i=0;i<leds->size();i++) {
            psvr_led_t l = leds->at(i);
            float lowest_dist= 65535.0f;
            int32_t closest_index = -1;
            for (int32_t j=0;j<NUM_LEDS;j++) {
                if (internal->prev_track_data.confidence[j] > 0) {
                    float d = math_distance(internal->prev_track_data.positions_3d[j],l.position);
                    //TODO: do not hardcode 0.5f distance threshold
                    if (d < lowest_dist && d < 0.5f) {
                        lowest_dist = d;
                        closest_index=j;
                    }
                }
            }

            t->positions_3d[closest_index] = l.position;
            t->l_positions_2d[closest_index] = l.l_position_2d;
            t->r_positions_2d[closest_index] = l.r_position_2d;
            t->confidence[closest_index] = 1;

            //update our longest distance
            if (t->longest_dist < l.longest_distance) {
                t->longest_dist = l.longest_distance;
            }

        }
        //if we dont have a center led, just use old position for now
        t->translation = internal->prev_track_data.translation;
        if (t->confidence[2] > 0){
            t->translation = t->positions_3d[2];
        }
        t->tracked_count =0;
        for (uint32_t i=0;i<NUM_LEDS;i++) {
            if (t->confidence[i] >0){
                t->tracked_count++;
            }
        }

    }
    internal->prev_track_data = *t;
    return true;
}

bool
psvr_disambiguate_5points(std::vector<psvr_led_t>* leds, psvr_track_data_t* t)
{
	// create a list of the corners, ignoring the center
    bool found_center = false;
    t->longest_dist = 0.0f;
    std::vector<uint32_t> corner_indices;
	for (uint32_t i = 0; i < leds->size(); i++) {
		psvr_led_t p = leds->at(i);
		if (p.sign_x == 0 && p.sign_y == 0) {
            //this is our center
            t->translation = p.position;
            t->confidence[2] = 1;
			t->positions_3d[2] = p.position;
			t->l_positions_2d[2] = p.l_position_2d;
			t->r_positions_2d[2] = p.r_position_2d;
            found_center = true;
		} else {
            //everything else is a corner
            corner_indices.push_back(i);
		}
        if (t->longest_dist < p.longest_distance)
        {
            t->longest_dist = p.longest_distance;
        }

	}
    if (! found_center)
    {
        return false;
    }
    t->tracked_count = 5;

	// find the leftmost and rightmost points - these will belong to our
	// left and right side edges.

    float lowest_x = 65535.0f;
    float highest_x = -65535.0f;
    uint32_t lowest_x_index =0;
    uint32_t highest_x_index =0;
	for (uint32_t i = 0; i < corner_indices.size(); i++) {
		psvr_led_t p = leds->at(corner_indices[i]);
		if (p.position.x < lowest_x) {
			lowest_x = p.position.x;
            lowest_x_index = p.world_point_index;
		}
		if (p.position.x > highest_x) {
			highest_x = p.position.x;
            highest_x_index = p.world_point_index;
		}
	}
	// printf("lowestX %f lowestXIndex %d highestX %f highestXIndex
	// %d\n",lowestX,lowestXIndex,highestX,highestXIndex); find the
	// corresponding (closest) point on the 'short side' for the left and
	// right extremities

	float lowest_l_x_distance = 65535.0f;
	float lowest_h_x_distance = 65535.0f;
	uint32_t lowest_x_pair_index;
	uint32_t highest_x_pair_index;

	psvr_led_t lcA = leds->at(lowest_x_index);
    for (uint32_t i = 0; i < corner_indices.size(); i++) {
        psvr_led_t lcB = leds->at(corner_indices[i]);
		if (corner_indices[i] != lowest_x_index) {
            float dist_l_x = math_distance(lcA.position, lcB.position);
			if (dist_l_x < lowest_l_x_distance) {
                lowest_x_pair_index = lcB.world_point_index;
				lowest_l_x_distance = dist_l_x;
			}
		}
	}
	psvr_led_t hcA = leds->at(highest_x_index);
	for (uint32_t i = 0; i < corner_indices.size(); i++) {
		psvr_led_t hcB = leds->at(corner_indices[i]);
		if (corner_indices[i] != highest_x_index) {
            float dist_h_x = math_distance(hcA.position, hcB.position);
			if (dist_h_x < lowest_h_x_distance) {
                highest_x_pair_index = hcB.world_point_index;
				lowest_h_x_distance = dist_h_x;
			}
		}
	}

	// now we have 4 points, and can know which 2 are left and which 2 are
	// right.

    psvr_led_t lA = leds->at(lowest_x_index);
    psvr_led_t lB = leds->at(lowest_x_pair_index);
	if (lA.position.y < lB.position.y) {
		// lA is upper left and lB is lower left
		t->positions_3d[0] = lA.position;
		t->l_positions_2d[0] = lA.l_position_2d;
		t->r_positions_2d[0] = lA.r_position_2d;
		t->confidence[0] = 1;
		t->positions_3d[3] = lB.position;
		t->l_positions_2d[3] = lB.l_position_2d;
		t->r_positions_2d[3] = lB.r_position_2d;
		t->confidence[3] = 1;

	} else {
		// lA is lower left and lB is upper left
		t->positions_3d[0] = lB.position;
		t->l_positions_2d[0] = lB.l_position_2d;
		t->r_positions_2d[0] = lB.r_position_2d;
		t->confidence[0] = 1;

		t->positions_3d[3] = lA.position;
		t->l_positions_2d[3] = lA.l_position_2d;
		t->r_positions_2d[3] = lA.r_position_2d;
		t->confidence[3] = 1;
	}

    psvr_led_t hA = leds->at(highest_x_index);
    psvr_led_t hB = leds->at(highest_x_pair_index);
	if (hA.position.y < hB.position.y) {
		// hA is upper right and rB is lower right
		t->positions_3d[1] = hA.position;
		t->l_positions_2d[1] = hA.l_position_2d;
		t->r_positions_2d[1] = hA.r_position_2d;
		t->confidence[1] = 1;

		t->positions_3d[4] = hB.position;
		t->l_positions_2d[4] = hB.l_position_2d;
		t->r_positions_2d[4] = hB.r_position_2d;
		t->confidence[4] = 1;
	} else {
		// hA is lower right and hB is upper right
		t->positions_3d[1] = hB.position;
		t->l_positions_2d[1] = hB.l_position_2d;
		t->r_positions_2d[1] = hB.r_position_2d;
		t->confidence[1] = 1;

		t->positions_3d[4] = hA.position;
		t->l_positions_2d[4] = hA.l_position_2d;
        t->r_positions_2d[4] = hA.r_position_2d;
		t->confidence[4] = 1;
	}
	return true;
}

bool psvr_compute_svd(psvr_track_data_t* t)
{
    //compute SVD for the points we have found, assuming we have at least 3 points
    if (t->tracked_count > 2)
    {
        cv::Mat measurement(t->tracked_count, 3, cv::DataType<float>::type);
        cv::Mat model(t->tracked_count, 3, cv::DataType<float>::type);
        cv::Mat xCovar;
        uint8_t c = 0;
        for (uint32_t i=0;i<NUM_LEDS;i++)
        {
                if (t->confidence[i] > 0)
                {
                        //normalised, and offset from center (represented as translation)
                        measurement.at<float>(c,0) = (t->translation.x - t->positions_3d[i].x) / t->longest_dist;
                        measurement.at<float>(c,1) = (t->translation.y - t->positions_3d[i].y) / t->longest_dist;
                        measurement.at<float>(c,2) = (t->translation.z - t->positions_3d[i].z) / t->longest_dist;
                        //printf("MEASURED POINT %d  %f %f %f %f %f %f\n",i,t->translation.x,t->translation.y,t->translation.z,measurement.at<float>(c,0),measurement.at<float>(c,1),measurement.at<float>(c,2));
                        model.at<float>(c,0) = physical_led_positions[i].x;
                        model.at<float>(c,1) = physical_led_positions[i].y;
                        model.at<float>(c,2) = physical_led_positions[i].z;
                        c++;
                }
        }

        // create our cross-covariance matrix
        cv::transpose(model,model);
        xCovar =  model * measurement;
        cv::Mat w,u,v,ut;
        cv::SVD::compute(xCovar,w,u,v);
        cv::transpose(u,ut);
        //TODO: compute determinant


        cv::Mat rot = v * ut;

        // if our matrix determinant is negative, we need to fix it
        // so the matrix is orthogonal

        if(cv::determinant(rot) < 0) {
            rot.at<float>(0,2) *= -1;
            rot.at<float>(1,2) *= -1;
            rot.at<float>(2,2) *= -1;
        }
        std::cout << rot << "det: " << cv::determinant(rot) <<"\n";

        cv::transpose(rot,rot);
        // we assume this is a 3x3 matrix
        memcpy(t->rotation_matrix.v,rot.data, 9 * sizeof(float));


        return true;
    }
    else
    {
        return false;
    }
}
