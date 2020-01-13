#!/usr/bin/env python3

import json
import struct

import numpy as np
from scipy.spatial.transform import Rotation


class Mat:
    def __init__(self, fp, name):
        print(name)
        header = fp.read(4 * 3)
        header_data = np.frombuffer(header, np.uint32)
        self.elem_size, self.rows, self.cols = header_data[0], header_data[1], header_data[2]
        print(self.elem_size, self.rows, self.cols)
        if self.elem_size == 0:
            return
        self.data_buf = fp.read(self.elem_size * self.rows * self.cols)

        elements = self.rows * self.cols
        # Totally not general, but in current usage this is what we have.
        if self.elem_size in (4, 8):
            data = np.frombuffer(
                self.data_buf, dtype=np.dtype("f{}".format(self.elem_size)), count=elements)
        else:
            raise RuntimeError("Don't know how to interpret this element size")

        self.data = data.reshape((self.rows, self.cols)).astype(float)
        print(self.data)


def getIntrinsics(mat):
    ret = {
        "f_x": mat[0, 0],
        "f_y": mat[1, 1],
        "c_x": mat[0, 2],
        "c_y": mat[1, 2]
    }
    skew = mat[0, 1]
    if skew != 0:
        ret["s"] = skew
    return ret


def getResolution(mat):
    return {
        "width": mat[0, 0],
        "height": mat[0, 1]
    }


def getDistortion(mat):
    coeffs = mat.flatten().tolist()
    # If this fails, then our assumption of which coefficients we're using is faulty.
    assert(len(coeffs) == 5)
    # Pad out to full 11 coeffs
    coeffs.extend([0] * (11 - len(coeffs)))
    # Unpack into the named coefficients
    k1, k2, p1, p2, k3, k4, k5, s1, s2, s3, s4 = coeffs
    assert(k4 == 0)
    return {
        "radial": [k1, k2, k3], #, k4, k5],
        "tangential": [p1, p2],
        #"thinPrism": [s1, s2, s3, s4]
    }


def getMatrix33(mat):
    return [
        [mat[i, j] for j in range(3)]
        for i in range(3)
    ]


def getVec3(mat):
    flat_mat = mat.flatten()
    return [flat_mat[i] for i in range(3)]


def getCalibration(intrinsics_mat, distortion_mat, mat_image_size):
    return {
        "resolution": getResolution(mat_image_size.data),
        "intrinsics": getIntrinsics(intrinsics_mat.data),
        "distortion": getDistortion(distortion_mat.data)
    }


def getQuat(mat):
    q = Rotation.from_matrix(mat).as_quat()
    return [q[i] for i in range(4)]


if __name__ == "__main__":
    with open("/home/ryan/.config/monado/PS4_EYE.calibration", "rb") as fp:
        l_intrinsics_mat = Mat(fp, "l_intrinsics_mat")
        r_intrinsics_mat = Mat(fp, "r_intrinsics_mat")
        l_distortion_mat = Mat(fp, "l_distortion_mat")
        r_distortion_mat = Mat(fp, "r_distortion_mat")
        l_distortion_fisheye_mat = Mat(fp, "l_distortion_fisheye_mat")
        r_distortion_fisheye_mat = Mat(fp, "r_distortion_fisheye_mat")
        l_rotation_mat = Mat(fp, "l_rotation_mat")
        r_rotation_mat = Mat(fp, "r_rotation_mat")
        l_translation_dummy = Mat(fp, "l_translation_dummy")
        r_translation_dummy = Mat(fp, "r_translation_dummy")
        l_projection_mat = Mat(fp, "l_projection_mat")
        r_projection_mat = Mat(fp, "r_projection_mat")
        disparity_to_depth_mat = Mat(fp, "disparity_to_depth_mat")
        mat_image_size = Mat(fp, "mat_image_size")
        mat_new_image_size = Mat(fp, "mat_new_image_size")
        camera_translation_mat = Mat(fp, "camera_translation_mat")
        camera_rotation_mat = Mat(fp, "camera_rotation_mat")
        camera_essential_mat = Mat(fp, "camera_essential_mat")
        camera_fundamental_mat = Mat(fp, "camera_fundamental_mat")
    from pprint import pprint
    out = {
        "$schema": "./schemas/camera-calib-schema.json",
        "stereo_camera": {
            "sensors": [
                # left camera
                {
                    "calibrations": [
                        getCalibration(l_intrinsics_mat,
                                       l_distortion_mat, mat_image_size)
                    ]
                },
                # right camera
                {
                    "calibrations": [
                        getCalibration(r_intrinsics_mat,
                                       r_distortion_mat, mat_image_size)
                    ],

                }
            ],
            "essential_matrix": getMatrix33(camera_essential_mat.data),
            "fundamental_matrix": getMatrix33(camera_fundamental_mat.data),
            "transform": {

                # "rotation": getMatrix33(camera_rotation_mat.data),
                "rotation": getQuat(camera_rotation_mat.data),
                "translation": getVec3(camera_translation_mat.data)
            }
        }
    }

    import json
    with open("calib.json", 'w', encoding='utf-8') as fp:
        json.dump(out, fp, indent=4)
        # print(CustomJSONEncoder(indent=4).encode(out))
