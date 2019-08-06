
#pragma once

#include <sys/stat.h>
#include <opencv2/opencv.hpp>

#include <xrt/xrt_defines.h>


static bool write_cv_mat(FILE* f, cv::Mat* m)
{
	uint32_t header[3];
	header[0] = m->elemSize();
	header[1] = m->rows;
	header[2] = m->cols;
	fwrite ((void*)header,sizeof(uint32_t),3,f);
	fwrite((void*)m->data,header[0],header[1]*header[2],f);
	return true;
}

static bool read_cv_mat(FILE* f, cv::Mat* m)
{
	uint32_t header[3];
	fread((void*)header,sizeof(uint32_t),3,f);
	//TODO: we may have written things other than CV_32F and CV_64F.
	if (header[0] == 4) {
	m->create(header[1],header[2],CV_32F);
	} else {
	m->create(header[1],header[2],CV_64F);
	}
	fread((void*)m->data,header[0],header[1]*header[2],f);
	return true;
}




static bool write_xrt_matrix44(FILE* f, struct xrt_matrix_4x4* m)
{
    fwrite((void*)m->v,sizeof(float),16,f);
    return true;
}

static bool read_xrt_matrix44(FILE* f, struct xrt_matrix_4x4* m)
{
    fread((void*)m->v,sizeof(float),16,f);
    return true;
}

static bool write_xrt_vec3(FILE* f, struct xrt_vec3* m)
{
    fwrite((void*)&m->x,sizeof(float),3,f);
    return true;
}

static bool read_xrt_vec3(FILE* f, struct xrt_vec3* m)
{
    fread((void*)&m->x,sizeof(float),3,f);
    return true;
}

static bool write_xrt_quat(FILE* f, struct xrt_quat* m)
{
    fwrite((void*)&m->x,sizeof(float),4,f);
    return true;
}

static bool read_xrt_quat(FILE* f, struct xrt_quat* m)
{
    fread((void*)&m->x,sizeof(float),4,f);
    return true;
}


static float cv_dist_3d(cv::Point3f& p, cv::Point3f& q) {
	cv::Point3f d = p - q;
	return cv::sqrt(d.x*d.x + d.y*d.y + d.z * d.z);
}

static float cv_dist_3d(cv::Vec3f& p, cv::Vec3f& q) {
    cv::Point3f d = p - q;
    return cv::sqrt(d.x*d.x + d.y*d.y + d.z * d.z);
}


//TODO - move this as it is a generic helper
static int mkpath(char* path) {
	char tmp[PATH_MAX]; //TODO: PATH_MAX probably not strictly correct
	char* p = NULL;
	size_t len;

	snprintf(tmp, sizeof(tmp),"%s",path);
	len = strlen(tmp) -1;
	if(tmp[len] == '/'){
		tmp[len] = 0;
	}
	for(p = tmp + 1; *p; p++) {
		if(*p == '/') {
			*p = 0;
			if (mkdir(tmp,S_IRWXU) < 0 && errno != EEXIST)
				return -1;
			*p = '/';
		}
	}
	if (mkdir(tmp, S_IRWXU) < 0 && errno != EEXIST)
		return -1;
	return 0;
}
