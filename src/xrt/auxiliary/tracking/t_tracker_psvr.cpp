// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR tracker code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_tracking
 */

#include "xrt/xrt_tracking.h"

#include "tracking/t_tracking.h"
#include "tracking/t_calibration_opencv.hpp"


#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_frame.h"
#include "util/u_format.h"

#include "math/m_api.h"

#include "os/os_threading.h"

#include <stdio.h>
#include <assert.h>
#include <pthread.h>

#include <permutations.hpp>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <inttypes.h>

#define PSVR_NUM_LEDS 7
#define PSVR_OPTICAL_SOLVE_THRESH 5
#define PSVR_MODEL_CENTER_INDEX 2 //we should use the tag enum for this

struct View
{
    cv::Mat undistort_rectify_map_x;
    cv::Mat undistort_rectify_map_y;

    std::vector<cv::KeyPoint> keypoints;

    cv::Mat frame_undist_rectified;

    void
    populate_from_calib(t_camera_calibration & /* calib */,
                        const RemapPair &rectification)
    {
        undistort_rectify_map_x = rectification.remap_x;
        undistort_rectify_map_y = rectification.remap_y;
    }
};

typedef enum led_tag
{
	TAG_NONE,
	TAG_TL,
	TAG_TR,
	TAG_C,
	TAG_BL,
	TAG_BR,
	TAG_SL,
	TAG_SR
} led_tag_t;

typedef struct model_vertex
{
    int32_t vertex_index;
    Eigen::Vector4f position;

	led_tag_t tag;
	bool active;

	bool
    operator<(const model_vertex &mv) const {
        return (vertex_index < mv.vertex_index);
	}
	bool
    operator>(const model_vertex &mv) const {
        return (vertex_index > mv.vertex_index);
	}

} model_vertex_t;

typedef struct match_data
{
    float angle;
    float distance;
    int32_t vertex_index;
    Eigen::Vector4f position;
} match_data_t;

typedef struct match_vertex
{
    int32_t vertex_index;
    std::vector<match_data_t> vertex_data;
} match_vertex_t;


typedef struct match_vertices
{
    std::vector<match_vertex_t> verts;
} match_vertices_t;

typedef struct match_model
{
    std::vector<match_data_t> measurements;
} match_model_t;


class TrackerPSVR
{
public:
	struct xrt_tracked_psvr base = {};
	struct xrt_frame_sink sink = {};
	struct xrt_frame_node node = {};

	//! Frame waiting to be processed.
	struct xrt_frame *frame;

	//! Thread and lock helper.
	struct os_thread_helper oth;

	//! Have we received a new IMU sample.
	bool has_imu = false;

	timepoint_ns last_imu{0};

	struct
	{
		struct xrt_vec3 pos = {};
		struct xrt_quat rot = {};
	} fusion;

    struct
    {
        struct xrt_vec3 pos = {};
        struct xrt_quat rot = {};
    } optical;

    Eigen::Quaternionf optical_rotation_correction;
    Eigen::Matrix4f corrected_imu_rotation;

	model_vertex_t model_vertices[PSVR_NUM_LEDS];
    std::vector<match_data_t> last_vertices;
	cv::KalmanFilter track_filters[PSVR_NUM_LEDS];

	View view[2];

	bool calibrated;

	cv::Mat disparity_to_depth;

	cv::Ptr<cv::SimpleBlobDetector> sbd;
	std::vector<cv::KeyPoint> l_blobs, r_blobs;
    std::vector<match_model_t> matches;

    std::vector<cv::Point3f> world_points;

    std::vector<Eigen::Vector4f> pruned_points;
    std::vector<Eigen::Vector4f> merged_points;

    std::vector<model_vertex_t> processed_points;
    std::vector<match_data_t> match_vertices;


    float model_scale_factor=40.0f;
    float outlier_thresh=7.0f;
    float merge_thresh=0.5f;

    FILE* dump_file;
};

static float dist_3d(Eigen::Vector4f a, Eigen::Vector4f b) {
    return sqrt((a[0] - b[0]) * (a[0] - b[0]) +
           (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

static void
init_filter(cv::KalmanFilter &kf, float process_cov, float meas_cov, float dt)
{
	kf.init(6, 3);
	kf.transitionMatrix =
	    (cv::Mat_<float>(6, 6) << 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 1.0,
	     0.0, 0.0, dt, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 0.0, 1.0,
	     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	     1.0);

	cv::setIdentity(kf.measurementMatrix, cv::Scalar::all(1.0f));
	cv::setIdentity(kf.errorCovPost, cv::Scalar::all(0.0f));

	// our filter parameters set the process and measurement noise
	// covariances.

	cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(process_cov));
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(meas_cov));
}

static void
filter_predict(model_vertex_t *pose, cv::KalmanFilter *filters, float dt)
{
	for (uint32_t i = 0; i < PSVR_NUM_LEDS; i++) {
		model_vertex_t *current_led = pose + i;
		cv::KalmanFilter *current_kf = filters + i;

		// set our dt components in the transition matrix
		current_kf->transitionMatrix.at<float>(0, 3) = dt;
		current_kf->transitionMatrix.at<float>(1, 4) = dt;
		current_kf->transitionMatrix.at<float>(2, 5) = dt;

        current_led->vertex_index = i;
        current_led->tag = (led_tag_t)(i + 1); // increment, as 0 is TAG_NONE
		cv::Mat prediction = current_kf->predict();
		current_led->position[0] = prediction.at<float>(0, 0);
		current_led->position[1] = prediction.at<float>(1, 0);
		current_led->position[2] = prediction.at<float>(2, 0);
	}
}

static void
filter_update(model_vertex_t *pose, cv::KalmanFilter *filters, float dt)
{
	for (uint32_t i = 0; i < PSVR_NUM_LEDS; i++) {
		model_vertex_t *current_led = pose + i;
		cv::KalmanFilter *current_kf = filters + i;

		// set our dt components in the transition matrix
		current_kf->transitionMatrix.at<float>(0, 3) = dt;
		current_kf->transitionMatrix.at<float>(1, 4) = dt;
		current_kf->transitionMatrix.at<float>(2, 5) = dt;

        current_led->vertex_index = i;
		current_led->tag =
		    (led_tag_t)(i + 1); // increment, as 0 is TAG_NONE
		cv::Mat measurement = cv::Mat(3, 1, CV_32F);
		measurement.at<float>(0, 0) = current_led->position[0];
		measurement.at<float>(1, 0) = current_led->position[1];
		measurement.at<float>(2, 0) = current_led->position[2];
		current_kf->correct(measurement);
	}
}


static bool match_possible(match_model_t* match) {

    // check if this match makes sense e.g. that top is 'above' bottom, that 'left' is left of right
    // check that unobservable combinations do not appear in the first n entries e.g. SR cannot be observed at the same time as SL
    return  true;
}

static void verts_to_measurement(std::vector<model_vertex_t>* meas_data, std::vector<match_data_t>* match_vertices)
{
    match_vertices->clear();
    if (meas_data->size() < PSVR_OPTICAL_SOLVE_THRESH) {
        for (uint32_t i=0;i<meas_data->size();i++) {
            match_data_t md;
            md.vertex_index=-1;
            md.position=meas_data->at(i).position;
            match_vertices->push_back(md);
        }

        return;

    }

    model_vertex_t ref_a = meas_data->at(0);
    model_vertex_t ref_b = meas_data->at(1);
    Eigen::Vector4f ref_vec = ref_b.position-ref_a.position;
    float ref_len = dist_3d(ref_a.position,ref_b.position);

    for (uint32_t i=0;i<meas_data->size();i++) {
        model_vertex_t vp = meas_data->at(i);
        Eigen::Vector4f point_vec = vp.position-ref_a.position;
        match_data_t md;
        md.vertex_index = -1;
        md.position = vp.position;
        Eigen::Vector3f ref_vec3 = ref_vec.head<3>();
        Eigen::Vector3f point_vec3 = point_vec.head<3>();
        Eigen::Vector3f vp_pos3 = vp.position.head<3>();

        if (i != 0) {
            Eigen::Vector3f plane_norm = ref_vec3.cross(point_vec3).normalized();
            if (plane_norm.z() > 0) {
                md.angle = -1 * acos(point_vec3.normalized().dot(ref_vec3.normalized()));
            } else {
                md.angle = acos(point_vec3.normalized().dot(ref_vec3.normalized()));
            }
            //printf("WAT: %f %f\n",dist_3d(point_vec,ref_a.position),ref_len);
            md.distance = dist_3d(vp.position,ref_a.position)/ref_len;
        }
        else
        {
           md.angle = 0.0f;
           md.distance = 0.0f;
        }
        //fix up any NaNs
        if (md.angle != md.angle) {md.angle = 0.0f;}
        if (md.distance != md.distance) {md.distance = 0.0f;}

        match_vertices->push_back(md);
    }


}

static float
last_diff(TrackerPSVR &t,
          std::vector<match_data_t>* meas_pose,
          std::vector<match_data_t>* last_pose)
{
    float diff = 0.0f;
    for (uint32_t i = 0; i < meas_pose->size(); i++) {
            uint32_t last_index = meas_pose->at(i).vertex_index;
            for (uint32_t j = 0; j < last_pose->size(); j++) {
                if (last_pose->at(j).vertex_index == last_index)
                {
                    float d = fabs(dist_3d(meas_pose->at(i).position,last_pose->at(j).position));
                    diff += d;

                }
            }

    }
    return diff;
}


static void
remove_outliers(std::vector<cv::Point3f> *orig_points,
                std::vector<Eigen::Vector4f> *pruned_points,
                float outlier_thresh)
{

    if (orig_points->size() == 0) {
        return;
    }
    // immediately prune anything that is measured as
	// 'behind' the camera
    std::vector<Eigen::Vector4f> temp_points;

	for (uint32_t i = 0; i < orig_points->size(); i++) {
		cv::Point3f p = orig_points->at(i);
		if (p.z < 0) {
            temp_points.push_back(Eigen::Vector4f(p.x, p.y, p.z,1.0f));
		}
	}

    if (temp_points.size() == 0)
    {
        return;
    }
	std::vector<float> x_values;
	std::vector<float> y_values;
	std::vector<float> z_values;
	for (uint32_t i = 0; i < temp_points.size(); i++) {
		x_values.push_back(temp_points[i][0]);
		y_values.push_back(temp_points[i][1]);
		z_values.push_back(temp_points[i][2]);
	}

	std::nth_element(x_values.begin(),
	                 x_values.begin() + x_values.size() / 2,
	                 x_values.end());
	float median_x = x_values[x_values.size() / 2];
	std::nth_element(y_values.begin(),
	                 y_values.begin() + y_values.size() / 2,
	                 y_values.end());
	float median_y = y_values[y_values.size() / 2];
	std::nth_element(z_values.begin(),
	                 z_values.begin() + z_values.size() / 2,
	                 z_values.end());
	float median_z = z_values[z_values.size() / 2];

	for (uint32_t i = 0; i < temp_points.size(); i++) {
		float error_x = temp_points[i][0] - median_x;
		float error_y = temp_points[i][1] - median_y;
		float error_z = temp_points[i][2] - median_z;

		float rms_error =
		    sqrt((error_x * error_x) + (error_y * error_y) +
		         (error_z * error_z));

		if (rms_error < outlier_thresh) {
			pruned_points->push_back(temp_points[i]);
		}
	}
}

struct close_pair {
    int index_a;
    int index_b;
};

static void merge_close_points( std::vector<Eigen::Vector4f> *orig_points,
                                std::vector<Eigen::Vector4f> *merged_points,
                                float merge_thresh) {
    std::vector<struct close_pair> pairs;
    for (uint32_t i=0;i<orig_points->size();i++) {
        for (uint32_t j=0;j<orig_points->size();j++) {
            if (i!= j) {
                if (dist_3d(orig_points->at(i),orig_points->at(j)) < merge_thresh) {
                    struct close_pair p;
                    p.index_a =i;
                    p.index_b=j;
                    pairs.push_back(p);
                }
            }
        }
    }
    std::vector<int> indices_to_remove;
    for (uint32_t i=0;i<pairs.size();i++) {
        if (pairs[i].index_a < pairs[i].index_b){
            indices_to_remove.push_back(pairs[i].index_a);
        } else {
            indices_to_remove.push_back(pairs[i].index_b);
        }
    }


    for (uint32_t i=0;i<orig_points->size();i++) {
        bool remove_index = false;
        for (uint32_t j=0;j<indices_to_remove.size();j++) {
            if (i == indices_to_remove[j]) {
                remove_index=true;
            }
        }
        if (! remove_index) {
            merged_points->push_back(orig_points->at(i));
        }
    }

};

static void
match_triangles(Eigen::Matrix4f *t1_mat,
                Eigen::Matrix4f *t1_to_t2_mat,
                Eigen::Vector4f t1_a,
                Eigen::Vector4f t1_b,
                Eigen::Vector4f t1_c,
                Eigen::Vector4f t2_a,
                Eigen::Vector4f t2_b,
                Eigen::Vector4f t2_c)
{
    *t1_mat = Eigen::Matrix4f().Identity();
    Eigen::Matrix4f t2_mat = Eigen::Matrix4f().Identity();

    Eigen::Vector3f t1_x_vec = (t1_b - t1_a).head<3>().normalized();
    Eigen::Vector3f t1_z_vec = (t1_c - t1_a).head<3>().cross((t1_b - t1_a).head<3>()).normalized();
    Eigen::Vector3f t1_y_vec = t1_x_vec.cross(t1_z_vec).normalized();

    Eigen::Vector3f t2_x_vec = (t2_b - t2_a).head<3>().normalized();
    Eigen::Vector3f t2_z_vec = (t2_c - t2_a).head<3>().cross((t2_b - t2_a).head<3>()).normalized();
    Eigen::Vector3f t2_y_vec = t2_x_vec.cross(t2_z_vec).normalized();

    t1_mat->col(0) << t1_x_vec[0], t1_x_vec[1], t1_x_vec[2], 0.0f;
    t1_mat->col(1) << t1_y_vec[0], t1_y_vec[1], t1_y_vec[2], 0.0f;
    t1_mat->col(2) << t1_z_vec[0], t1_z_vec[1], t1_z_vec[2], 0.0f;
    t1_mat->col(3) << t1_a[0], t1_a[1], t1_a[2], 1.0f;

    t2_mat.col(0) << t2_x_vec[0], t2_x_vec[1], t2_x_vec[2], 0.0f;
    t2_mat.col(1) << t2_y_vec[0], t2_y_vec[1], t2_y_vec[2], 0.0f;
    t2_mat.col(2) << t2_z_vec[0], t2_z_vec[1], t2_z_vec[2], 0.0f;
    t2_mat.col(3) << t2_a[0], t2_a[1], t2_a[2], 1.0f;

    //std::cout << "T1_MAT: \n" << *t1_mat << "\n";
    //std::cout << "T2_MAT: \n" << t2_mat << "\n";

    *t1_to_t2_mat = t1_mat->inverse() * t2_mat;

}

static Eigen::Matrix4f solve_for_measurement(TrackerPSVR* t,std::vector<match_data_t>* measurement,std::vector<match_data_t>* solved) {

    Eigen::Vector4f meas_ref_a = measurement->at(0).position;
    Eigen::Vector4f meas_ref_b = measurement->at(1).position;
    int meas_index_a = measurement->at(0).vertex_index;
    int meas_index_b = measurement->at(1).vertex_index;

    Eigen::Vector4f model_ref_a = t->model_vertices[meas_index_a].position;
    Eigen::Vector4f model_ref_b = t->model_vertices[meas_index_b].position;

    float highest_length = 0.0f;
    int best_model_index = 0;
    int most_distant_index = 0;

    for (uint32_t i=0;i < measurement->size();i++){
        int model_tag_index = measurement->at(i).vertex_index;
        Eigen::Vector4f model_vert = t->model_vertices[model_tag_index].position;
        if (most_distant_index > 1 && dist_3d(model_vert,model_ref_a) > highest_length) {
            best_model_index = most_distant_index;
        }
        most_distant_index++;
    }

    Eigen::Vector4f meas_ref_c = measurement->at(best_model_index).position;
    int meas_index_c = measurement->at(best_model_index).vertex_index;

    Eigen::Vector4f model_ref_c =  t->model_vertices[ meas_index_c ].position;


    Eigen::Matrix4f tri_basis;
    Eigen::Matrix4f model_to_measurement;


    //std::cout << "MODEL_REF_A: \n" << model_ref_a << "\n";
    //std::cout << "MODEL_REF_B: \n" << model_ref_b << "\n";
    //std::cout << "MODEL_REF_C: \n" << model_ref_c << "\n";
    //std::cout << "MEAS_REF_A: \n" << meas_ref_a << "\n";
    //std::cout << "MEAS_REF_B: \n" << meas_ref_b << "\n";
    //std::cout << "MEAS_REF_C: \n" << meas_ref_c << "\n";

    match_triangles(&tri_basis,&model_to_measurement,model_ref_a,model_ref_b,model_ref_c,meas_ref_a,meas_ref_b,meas_ref_c);

    Eigen::Matrix4f model_center_transform = tri_basis * model_to_measurement * tri_basis.inverse();

    solved->clear();
    for (uint32_t i=0;i<PSVR_NUM_LEDS;i++) {
        match_data_t md;
        md.vertex_index = i;
        md.position =  model_center_transform * t->model_vertices[i].position;
        solved->push_back(md);
        //printf("MEAS SOLVED VERT: %d %f %f %f \n",i,md.position.x(),md.position.y(),md.position.z());
    }
    Eigen::Matrix4f pose =  model_center_transform;
    //std::cout << "MEAS POSE:\n" << pose << "\n";

    return pose;

}

typedef struct proximity_data {
    Eigen::Vector4f position;
    float lowest_distance;
    int vertex_index;
} proximity_data_t;

static Eigen::Matrix4f solve_with_imu(TrackerPSVR& t,std::vector<match_data_t>* measurements,std::vector<match_data_t>* match_measurements,std::vector<match_data_t>* solved,float search_radius) {
    std::vector<proximity_data_t> proximity_data;
    printf("measurements %d last measurements: %d\n",measurements->size(),match_measurements->size());
    for (uint32_t i=0; i< measurements->size();i++){
        float lowest_distance = 65535.0;
        int closest_index=0;
        proximity_data_t p;
        match_data_t measurement = measurements->at(i);
        if (measurement.vertex_index == -1) { //we are closest-matching this vertex
            for (uint32_t j=0; j< match_measurements->size();j++){
                match_data_t match_measurement = match_measurements->at(j);
                float distance = dist_3d(measurement.position,match_measurement.position);
                if (distance < lowest_distance) {
                    lowest_distance = distance;
                    closest_index=match_measurement.vertex_index;
                }
            }
            if (lowest_distance < search_radius) {
                p.position = measurement.position;
                p.vertex_index = closest_index;
                p.lowest_distance = lowest_distance;
                proximity_data.push_back(p);
            }
        } else {
            p.position = measurement.position;
            p.vertex_index = measurement.vertex_index;
            p.lowest_distance = 0.0f;
            proximity_data.push_back(p);
        }
    }

    if (proximity_data.size() > 0) {

        std::vector<match_model_t> temp_measurement_list;
        for (uint32_t i=0;i<proximity_data.size();i++) {
            proximity_data_t p = proximity_data[i];
            Eigen::Vector4f model_vertex = t.model_vertices[p.vertex_index].position;
            Eigen::Vector4f model_center = t.model_vertices[PSVR_MODEL_CENTER_INDEX].position;
            Eigen::Vector4f measurement_vertex = p.position;
            Eigen::Vector4f measurement_offset = t.corrected_imu_rotation * model_vertex;
            Eigen::Affine3f translation(Eigen::Translation3f((measurement_vertex-measurement_offset).head<3>()));
            Eigen::Matrix4f model_to_measurement  =  translation.matrix() * t.corrected_imu_rotation;
            match_model_t temp_measurement;
            for (uint32_t j=0;j<PSVR_NUM_LEDS;j++){
                match_data_t md;
                md.position = model_to_measurement * t.model_vertices[j].position;
                md.vertex_index = j;
                temp_measurement.measurements.push_back(md);
            }
            temp_measurement_list.push_back(temp_measurement);
        }
        for (uint32_t i=0; i< PSVR_NUM_LEDS;i++) {
            match_data_t avg_data;
            avg_data.position = Eigen::Vector4f(0.0f,0.0f,0.0f,1.0f);
            for (uint32_t j=0;j<temp_measurement_list.size();j++) {
                avg_data.position += temp_measurement_list[j].measurements[i].position;
            }
            avg_data.position /= float(temp_measurement_list.size());
            avg_data.vertex_index = i;
            solved->push_back(avg_data);
            //printf("IMU SOLVED VERT: %d %f %f %f \n",i,avg_data.position.x(),avg_data.position.y(),avg_data.position.z());
        }
        Eigen::Vector3f center_pos = (solved->at(PSVR_MODEL_CENTER_INDEX).position).head<3>();
        Eigen::Translation3f center_trans(center_pos);
        Eigen::Affine3f translation(center_trans);
        Eigen::Matrix4f pose = translation.matrix() * t.corrected_imu_rotation;
        //std::cout << "IMU POSE:\n" << pose << "\n";
        return pose;

    }
   printf("RETURNING IDENTITY\n");
   return Eigen::Matrix4f().Identity();

}


static Eigen::Matrix4f disambiguate(TrackerPSVR &t,std::vector<match_data_t>* measured_points,std::vector<match_data_t>* last_measurement,std::vector<match_data_t>* solved,uint32_t frame_no) {

    if (measured_points->size() < PSVR_OPTICAL_SOLVE_THRESH && last_measurement->size() > 0) {
        //printf("SOLVING WITH IMU\n");
            return solve_with_imu(t,measured_points,last_measurement,solved,3.0f);
    }

    if (measured_points->size() < 3) {
        //equivalent to returning null, but better than crashing
        return solve_with_imu(t,measured_points,last_measurement,solved,3.0f);
    }

    float lowestError=65535.0f;
    int32_t bestModel = -1;
    for (uint32_t i=0;i< t.matches.size();i++ ) {
        match_model_t m = t.matches[i];

        float squaredSum =0.0f;
        float signDiff=0.0f;

        // we have 2 measurements per vertex (distance and angle)
        // and we are comparing only the 'non-basis vector' elements

        //fill in our 'proposed' vertex indices from the model data (this will be overwritten once our best model is selected
        for (uint32_t i=0;i<measured_points->size();i++) {
            measured_points->at(i).vertex_index = m.measurements.at(i).vertex_index;
        }

        float ld = last_diff(t,measured_points,&t.last_vertices);
        //printf("last diff: model %d %f\n",i,ld);

        float tl_y,tr_y,bl_y,br_y = -65535.0f;
        bool ignore = false;
        for (uint32_t j =1; j < measured_points->size() ;j++) {

/*
            //reject anything where TL ends up below BL or TR ends up below BR
            if (measured_points->at(j).vertex_index == 1) { //TL - TODO: use tags
                                tl_y = measured_points->at(j).position.y();
            }
            if (measured_points->at(j).vertex_index == 0) { //BL - TODO: use tags
                                bl_y = measured_points->at(j).position.y();
            }
            if (measured_points->at(j).vertex_index == 4) { //TR - TODO: use tags
                                tr_y = measured_points->at(j).position.y();
            }
            if (measured_points->at(j).vertex_index == 3) { //BR - TODO: use tags
                                br_y = measured_points->at(j).position.y();
            }


            if (tl_y != -65535.0f and bl_y != -65535.0f) {
                if (tl_y < bl_y) {
                    ignore = true;
                }
            }
            if (tr_y != -65535.0f and br_y != -65535.0f) {
                if (tr_y < br_y) {
                    ignore = true;
                }
            }
            */
            squaredSum += fabs(measured_points->at(j).distance - m.measurements.at(j).distance);// * (measured_points->at(j).distance - m.data.at(j).distance);
            squaredSum += fabs(measured_points->at(j).angle - m.measurements.at(j).angle);// * (measured_points->at(j).angle - m.data.at(j).angle);

            //int vi = measured_points->at(j).vertex_index;
            //Eigen::Vector4f lastPos = last_measurement->at(vi).position;
            //Eigen::Vector4f thisPos = measured_points->at(j).position;
            //squaredSum += dist_3d(thisPos,lastPos);

            if (copysign(1,measured_points->at(j).distance) != copysign(1,m.measurements.at(j).distance)){
                squaredSum +=10.0f;
             }
            if (copysign(1,measured_points->at(j).angle) != copysign(1,m.measurements.at(j).angle)){
                squaredSum += 10.0f;
             }

            //printf("%d squaredSum: %f\n",j,squaredSum);
        }
        float rmsError = squaredSum / measured_points->size() + ld;//sqrt(squaredSum);

        if (rmsError <= lowestError && ! ignore) {
            lowestError = rmsError;
            bestModel = i;
        }

    }
    printf ("ERR: %f BEST: %d\n",lowestError,bestModel);

    //printf("model %d:\t",bestModel);
    match_model_t m = t.matches[bestModel];

    for (uint32_t i=0;i < m.measurements.size();i++) {
        //printf("INDEX: %d DIST: %f ANGLE: %f ",m.measurements[i].vertex_index,m.measurements[i].distance,m.measurements[i].angle);

    }
    //printf("\n");

    //printf("meas  ?:\t");
    for (uint32_t i=0;i<measured_points->size();i++) {
        //printf("? %f %f ",measured_points->at(i).distance,measured_points->at(i).angle);
    }
    //printf("\n");



    for (uint32_t i=0;i<measured_points->size();i++) {
        measured_points->at(i).vertex_index = m.measurements.at(i).vertex_index;
    }

    return solve_for_measurement(&t,measured_points,solved);

}

static void
create_model(TrackerPSVR &t)
{
    t.model_vertices[0] = {0, Eigen::Vector4f(2.51408f, 3.77113f, 0.0f,1.0f),TAG_BL,true};
    t.model_vertices[1] = {1, Eigen::Vector4f(2.51408f, -3.77113f, 0.0f,1.0f),TAG_TL,true};
    t.model_vertices[2] = {2, Eigen::Vector4f(0.0f, 0.0f, 1.54926f,1.0f), TAG_C,true};
    t.model_vertices[3] = {3, Eigen::Vector4f(-2.51408f, 3.77113f, 0.0f,1.0f),TAG_BR,true};
    t.model_vertices[4] = {4, Eigen::Vector4f(-2.51408f, -3.77113f, 0.0f,1.0f),TAG_TR,true};
    t.model_vertices[5] = {5, Eigen::Vector4f(0.0f, 4.52535f, -2.62887f,1.0f),TAG_SL,true};
    t.model_vertices[6] = {6, Eigen::Vector4f(0.0f, -4.52535f, -2.62887f,1.0f),TAG_SR,true};
}



static void create_match_list(TrackerPSVR &t)
{
	// create our permutation list
	for (auto &&vec : iter::permutations(t.model_vertices)) {
        match_model_t m;

		model_vertex_t ref_pt_a = vec[0];
		model_vertex_t ref_pt_b = vec[1];
        Eigen::Vector3f ref_vec3 = (ref_pt_b.position - ref_pt_a.position).head<3>();

		float normScale = dist_3d(ref_pt_a.position, ref_pt_b.position);

        match_data_t md;
        for (auto &&i : vec) {
            Eigen::Vector3f point_vec3 = (i.position-ref_pt_a.position).head<3>();
            md.vertex_index = i.vertex_index;
            md.distance =
			    dist_3d(i.position, ref_pt_a.position) / normScale;
            if (i.position.head<3>().dot(Eigen::Vector3f(0.0,0.0,1.0f)) < 0 ) {
                        md.distance *= -1;
                    }

            Eigen::Vector3f plane_norm =
                ref_vec3.cross(point_vec3).normalized();
            if (ref_pt_a.position != i.position) {

                if (plane_norm.normalized().z() > 0) {
                    md.angle =
                        -1 * acos((point_vec3).normalized().dot(ref_vec3.normalized()));
				} else {
                    md.angle =
                        acos(point_vec3.normalized().dot(ref_vec3.normalized()));
				}
			} else {
                md.angle = 0.0f;
			}
            //fix up any NaNs
            if (md.angle != md.angle) {md.angle = 0.0f;}
            if (md.distance != md.distance) {md.distance = 0.0f;}

            m.measurements.push_back(md);
		}
        if (match_possible(&m)){
            t.matches.push_back(m);
        }
	}
}

static void
do_view(TrackerPSVR &t, View &view, cv::Mat &grey)
{
    // Undistort and rectify the whole image.
    cv::remap(grey,                         // src
              view.frame_undist_rectified,  // dst
              view.undistort_rectify_map_x, // map1
              view.undistort_rectify_map_y, // map2
              cv::INTER_LINEAR,             // interpolation
              cv::BORDER_CONSTANT,          // borderMode
              cv::Scalar(0, 0, 0));         // borderValue

    cv::threshold(view.frame_undist_rectified, // src
                  view.frame_undist_rectified, // dst
                  32.0,                        // thresh
                  255.0,                       // maxval
                  0);

                   // type

	// tracker_measurement_t m = {};

	// Do blob detection with our masks.
	//! @todo Re-enable masks.

    t.sbd->detect(view.frame_undist_rectified, // image
	              view.keypoints,       // keypoints
                  cv::noArray());       // mask
}

static void
process(TrackerPSVR &t, struct xrt_frame *xf)
{
	// Only IMU data
    if (xf == NULL) {
		return;
	}
	float dt = 1.0f;
	// get our predicted filtered led positions, if any
	model_vertex_t predicted_pose[PSVR_NUM_LEDS];
	filter_predict(predicted_pose, t.track_filters, dt);

	model_vertex_t measured_pose[PSVR_NUM_LEDS];

    // get our raw measurements

	t.view[0].keypoints.clear();
	t.view[1].keypoints.clear();
	t.l_blobs.clear();
	t.r_blobs.clear();
    t.world_points.clear();

	int cols = xf->width / 2;
	int rows = xf->height;
	int stride = xf->stride;

	cv::Mat l_grey(rows, cols, CV_8UC1, xf->data, stride);
	cv::Mat r_grey(rows, cols, CV_8UC1, xf->data + cols, stride);


	do_view(t, t.view[0], l_grey);
	do_view(t, t.view[1], r_grey);

	// do some basic matching to come up with likely disparity-pairs.

	for (uint32_t i = 0; i < t.view[0].keypoints.size(); i++) {
		cv::KeyPoint l_blob = t.view[0].keypoints[i];
		int l_index = -1;
		int r_index = -1;

		for (uint32_t j = 0; j < t.view[1].keypoints.size(); j++) {
			float lowest_dist = 128;
			cv::KeyPoint r_blob = t.view[1].keypoints[j];
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
			t.l_blobs.push_back(t.view[0].keypoints.at(l_index));
			t.r_blobs.push_back(t.view[1].keypoints.at(r_index));
		}
	}

	// Convert our 2d point + disparities into 3d points.
    if (t.l_blobs.size() > 0) {
		for (uint32_t i = 0; i < t.l_blobs.size(); i++) {
			float disp = t.r_blobs[i].pt.x - t.l_blobs[i].pt.x;
			cv::Vec4d xydw(t.l_blobs[i].pt.x, t.l_blobs[i].pt.y,
			               disp, 1.0f);
			// Transform
			cv::Vec4d h_world =
			    (cv::Matx44d)t.disparity_to_depth * xydw;

			// Divide by scale to get 3D vector from homogeneous
            // coordinate.
            t.world_points.push_back(cv::Point3f(
                h_world[0] / h_world[3], h_world[1] / h_world[3],
                h_world[2] / h_world[3]) * t.model_scale_factor);
		}
	}

    //raw debug output for Blender algo development
    for (int i=0;i<t.world_points.size();i++) {
        cv::Point3f unscaled = t.world_points[i] / t.model_scale_factor;
        fprintf(t.dump_file,"P,%" PRIu64 ", %f,%f,%f\n",xf->source_sequence,unscaled.x,unscaled.y,unscaled.z);
    }

    t.pruned_points.clear();
    t.merged_points.clear();
    t.processed_points.clear();

    // remove outliers from our measurement list
    printf("WORLD POINTS %d\n",t.world_points.size());
    remove_outliers(&t.world_points, &t.pruned_points, t.outlier_thresh);
    printf("PRUNED POINTS %d\n",t.pruned_points.size());

    merge_close_points(&t.pruned_points, &t.merged_points,t.merge_thresh);
    printf("MERGED POINTS %d\n",t.merged_points.size());

	// try matching our leds against the predictions
	// if error low, fit model using raw and filtered positions
    if (t.merged_points.size() < PSVR_NUM_LEDS) {
      for (uint32_t i=0;i< t.merged_points.size();i++) {
        model_vertex_t mv;
        mv.position = t.merged_points[i];
        t.processed_points.push_back(mv);
        //printf("INPUT: %f %f %f\n",mv.position.x(),mv.position.y(),mv.position.z());
    }
    } else {
        printf("Too many blobs to be a PSVR! %ld\n",t.processed_points.size());
    }
    fprintf(t.dump_file,"\n");

    // convert our points to match data, and disambiguate it, this tags our match_vertices with
    // everything we need to solve the pose.
    verts_to_measurement(&t.processed_points,&t.match_vertices);

    std::vector<match_data_t> solved;
    Eigen::Matrix4f model_center_transform = disambiguate(t, &t.match_vertices,&t.last_vertices,&solved,0);

    Eigen::Matrix3f r = model_center_transform.block(0,0,3,3);
    Eigen::Quaternionf rot(r);

    if (t.processed_points.size() ==5) {
        t.optical_rotation_correction = Eigen::Quaternionf(t.fusion.rot.w,t.fusion.rot.x,t.fusion.rot.y,t.fusion.rot.z).inverse() * rot;
    }
    t.optical.rot.x = rot.x();
    t.optical.rot.y = rot.y();
    t.optical.rot.z = rot.z();
    t.optical.rot.w = rot.w();

    t.optical.pos.x = model_center_transform(0,3); //row-first
    t.optical.pos.y = model_center_transform(1,3);
    t.optical.pos.z = model_center_transform(2,3);
    printf("POS: %f %f %f\n",t.optical.pos.x,t.optical.pos.y,t.optical.pos.z);
    printf("OPTICAL ROT: %f %f %f %f\n",t.optical.rot.x,t.optical.rot.y,t.optical.rot.z,t.optical.rot.w);


    //t.optical.rot.w = 1.0f;
    // update filter

    model_vertex_t f_update[PSVR_NUM_LEDS];
    t.last_vertices.clear();
    for (uint32_t i=0;i<solved.size();i++) {
        t.last_vertices.push_back(solved[i]);
        f_update[i].position = solved[i].position;
        fprintf(t.dump_file,"S,%" PRIu64 ", %f,%f,%f\n",xf->source_sequence,solved[i].position.x(),solved[i].position.y(),solved[i].position.z());
        //fprintf(t.dump_file,"S,%" PRIu64 ", %f,%f,%f\n",xf->source_sequence,predicted_pose[i].position.x(),predicted_pose[i].position.y(),predicted_pose[i].position.z());

    }
    fprintf(t.dump_file,"\n");

    filter_update(f_update, t.track_filters, dt);

    printf("FRAME: %ld\n",xf->source_sequence);
	xrt_frame_reference(&xf, NULL);
}

static void
run(TrackerPSVR &t)
{
	struct xrt_frame *frame = NULL;

	os_thread_helper_lock(&t.oth);

	while (os_thread_helper_is_running_locked(&t.oth)) {
		// No data
		if (!t.has_imu || t.frame == NULL) {
			os_thread_helper_wait_locked(&t.oth);
		}

		if (!os_thread_helper_is_running_locked(&t.oth)) {
			break;
		}

		// Take a reference on the current frame, this keeps it alive
		// if it is replaced during the consumer processing it, but
		// we no longer need to hold onto the frame on the queue we
		// just move the pointer.
		frame = t.frame;
		t.frame = NULL;

		// Unlock the mutex when we do the work.
		os_thread_helper_unlock(&t.oth);

		process(t, frame);

		// Have to lock it again.
		os_thread_helper_lock(&t.oth);
	}

	os_thread_helper_unlock(&t.oth);
}

static void
get_pose(TrackerPSVR &t,
         struct time_state *timestate,
         timepoint_ns when_ns,
         struct xrt_space_relation *out_relation)
{
	os_thread_helper_lock(&t.oth);

	// Don't do anything if we have stopped.
	if (!os_thread_helper_is_running_locked(&t.oth)) {
		os_thread_helper_unlock(&t.oth);
		return;
	}

    out_relation->pose.position = t.optical.pos;
    out_relation->pose.orientation = t.optical.rot;

	//! @todo assuming that orientation is actually currently tracked.
	out_relation->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_POSITION_VALID_BIT |
	    XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);

	os_thread_helper_unlock(&t.oth);
}

static void
imu_data(TrackerPSVR &t,
         timepoint_ns timestamp_ns,
         struct xrt_tracking_sample *sample)
{
	os_thread_helper_lock(&t.oth);

	// Don't do anything if we have stopped.
	if (!os_thread_helper_is_running_locked(&t.oth)) {
		os_thread_helper_unlock(&t.oth);
		return;
	}
	if (t.last_imu != 0) {
		time_duration_ns delta_ns = timestamp_ns - t.last_imu;
		float dt = time_ns_to_s(delta_ns);
		// Super simple fusion.
		math_quat_integrate_velocity(
		    &t.fusion.rot, &sample->gyro_rad_secs, dt, &t.fusion.rot);
	}
    fprintf(t.dump_file,"I,%" PRIu64 ", %f,%f,%f,%f\n\n",timestamp_ns,t.fusion.rot.x,t.fusion.rot.y,t.fusion.rot.z,t.fusion.rot.w);

    Eigen::Matrix4f fusion_rot = Eigen::Matrix4f().Identity();
    fusion_rot.block(0,0,3,3) = (Eigen::Quaternionf(t.fusion.rot.w,t.fusion.rot.x,t.fusion.rot.y,t.fusion.rot.z) * t.optical_rotation_correction).toRotationMatrix();
    t.corrected_imu_rotation = fusion_rot;
	t.last_imu = timestamp_ns;

	os_thread_helper_unlock(&t.oth);
}

static void
frame(TrackerPSVR &t, struct xrt_frame *xf)
{
	os_thread_helper_lock(&t.oth);

	// Don't do anything if we have stopped.
	if (!os_thread_helper_is_running_locked(&t.oth)) {
		os_thread_helper_unlock(&t.oth);
		return;
	}

	xrt_frame_reference(&t.frame, xf);

	// Wake up the thread.
	os_thread_helper_signal_locked(&t.oth);

	os_thread_helper_unlock(&t.oth);
}

static void
break_apart(TrackerPSVR &t)
{
	os_thread_helper_stop(&t.oth);
}


/*
 *
 * C wrapper functions.
 *
 */

extern "C" void
t_psvr_push_imu(struct xrt_tracked_psvr *xtvr,
                timepoint_ns timestamp_ns,
                struct xrt_tracking_sample *sample)
{
	auto &t = *container_of(xtvr, TrackerPSVR, base);
	imu_data(t, timestamp_ns, sample);
}

extern "C" void
t_psvr_get_tracked_pose(struct xrt_tracked_psvr *xtvr,
                        struct time_state *timestate,
                        timepoint_ns when_ns,
                        struct xrt_space_relation *out_relation)
{
	auto &t = *container_of(xtvr, TrackerPSVR, base);
	get_pose(t, timestate, when_ns, out_relation);
}

extern "C" void
t_psvr_fake_destroy(struct xrt_tracked_psvr *xtvr)
{
	auto &t = *container_of(xtvr, TrackerPSVR, base);
	(void)t;
	// Not the real destroy function
}

extern "C" void
t_psvr_sink_push_frame(struct xrt_frame_sink *xsink, struct xrt_frame *xf)
{
	auto &t = *container_of(xsink, TrackerPSVR, sink);
	frame(t, xf);
}

extern "C" void
t_psvr_node_break_apart(struct xrt_frame_node *node)
{
	auto &t = *container_of(node, TrackerPSVR, node);
	break_apart(t);
}

extern "C" void
t_psvr_node_destroy(struct xrt_frame_node *node)
{
	auto t_ptr = container_of(node, TrackerPSVR, node);

	os_thread_helper_destroy(&t_ptr->oth);

	delete t_ptr;
}

extern "C" void *
t_psvr_run(void *ptr)
{
	auto &t = *(TrackerPSVR *)ptr;
	run(t);
	return NULL;
}


/*
 *
 * Exported functions.
 *
 */

extern "C" int
t_psvr_start(struct xrt_tracked_psvr *xtvr)
{
	auto &t = *container_of(xtvr, TrackerPSVR, base);
	int ret;


	ret = os_thread_helper_start(&t.oth, t_psvr_run, &t);
	if (ret != 0) {
		return ret;
	}

	return ret;
}

extern "C" int
t_psvr_create(struct xrt_frame_context *xfctx,
              struct t_stereo_camera_calibration *data,
              struct xrt_tracked_psvr **out_xtvr,
              struct xrt_frame_sink **out_sink)
{
	fprintf(stderr, "%s\n", __func__);

	auto &t = *(new TrackerPSVR());
	int ret;

    for (uint32_t i=0;i<PSVR_NUM_LEDS;i++){
        init_filter(t.track_filters[i],0.1f,0.1f,1.0f);
    }

    StereoCameraCalibrationWrapper wrapped(*data);
    StereoRectificationMaps rectify(*data);
    t.view[0].populate_from_calib(data->view[0], rectify.view[0].rectify);
    t.view[1].populate_from_calib(data->view[1], rectify.view[1].rectify);
    t.disparity_to_depth = rectify.disparity_to_depth_mat;
    t.calibrated = true;

    // clang-format off
    cv::SimpleBlobDetector::Params blob_params;
    blob_params.filterByArea = false;
    blob_params.filterByConvexity = false;
    blob_params.filterByInertia = false;
    blob_params.filterByColor = true;
    blob_params.blobColor = 255; // 0 or 255 - color comes from binarized image?
    blob_params.minArea = 1;
    blob_params.maxArea = 1000;
    blob_params.maxThreshold = 51; // using a wide threshold span slows things down bigtime
    blob_params.minThreshold = 50;
    blob_params.thresholdStep = 1;
    blob_params.minDistBetweenBlobs = 5;
    blob_params.minRepeatability = 1; // need this to avoid error?
    // clang-format on

    t.sbd = cv::SimpleBlobDetector::create(blob_params);

    t.corrected_imu_rotation = Eigen::Matrix4f().Identity();

    create_model(t);
    create_match_list(t);

	t.base.get_tracked_pose = t_psvr_get_tracked_pose;
	t.base.push_imu = t_psvr_push_imu;
	t.base.destroy = t_psvr_fake_destroy;
	t.sink.push_frame = t_psvr_sink_push_frame;
	t.node.break_apart = t_psvr_node_break_apart;
	t.node.destroy = t_psvr_node_destroy;
	t.fusion.rot.w = 1.0f;

	ret = os_thread_helper_init(&t.oth);
	if (ret != 0) {
		delete (&t);
		return ret;
	}

	// HACK, to counter the tracking origin offset.
	t.fusion.pos.x = 0.0f;
	t.fusion.pos.y = 0.6f;
	t.fusion.pos.z = -2.0f;

	t.fusion.rot.x = 0.0f;
	t.fusion.rot.y = 1.0f;
	t.fusion.rot.z = 0.0f;
	t.fusion.rot.w = 0.0f;

	xrt_frame_context_add(xfctx, &t.node);

	*out_sink = &t.sink;
	*out_xtvr = &t.base;

    t.dump_file = fopen("/tmp/psvr_dump.txt","w");

	return 0;
}
