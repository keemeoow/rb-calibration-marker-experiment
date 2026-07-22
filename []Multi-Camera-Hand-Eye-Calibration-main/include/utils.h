//
// Created by davide on 12/09/23.
//
#ifndef METRIC_CALIBRATOR_UTILS_H
#define METRIC_CALIBRATOR_UTILS_H

#include <string>
#include <filesystem>
#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <random>
#include <opencv2/core/affine.hpp>
#include <cmath>
#include <opencv2/features2d.hpp>
#include <algorithm>
#include <unordered_set>
#include "CameraInfo.h"
#include "CalibrationInfo.h"

struct Transformation {
    double tx, ty, tz; // Traslazione: x, y, z
    double rx, ry, rz; // Rotazione: x, y, z
};

struct LinearRegressionResult {
    double slope;
    double intercept;
};

void checkData(std::string data_folder, std::string prefix, int number_of_cameras);
int countImages(const std::string& path);
int countFolderImages(const std::string& base_path, const std::string& cam_prefix);

bool isFolderNotEmpty(const std::string& folder_path);
bool compareFilenames(const std::filesystem::directory_entry& a, const std::filesystem::directory_entry& b);

void readPoseFromCSV(const std::string& input_path, cv::Mat& out_mat, char delim);
bool filterRansac(std::vector<cv::Point3f> object_points, std::vector<cv::Point2f> corners, CameraInfo camera_info, cv::Mat& rvec, cv::Mat& tvec);
void setInitialGuess(std::vector<cv::Mat> &h2e_initial_guess_vec, cv::Mat &b2ee_initial_guess, const std::vector<std::vector<cv::Mat>> rototras_vec, const std::vector<std::vector<cv::Mat>> correct_poses, const int calibration_setup);
void createFolder(std::string folder_name);
int linesNumber(const std::string file_path);


template <typename _T>
void getRotoTras(cv::Mat rotation, cv::Mat translation, cv::Mat& G){
    G = (cv::Mat_<_T>(4,4) << rotation.at<_T>(0,0), rotation.at<_T>(0,1), rotation.at<_T>(0,2), translation.at<_T>(0),
            rotation.at<_T>(1,0), rotation.at<_T>(1,1), rotation.at<_T>(1,2), translation.at<_T>(1),
            rotation.at<_T>(2,0), rotation.at<_T>(2,1), rotation.at<_T>(2,2), translation.at<_T>(2),
            0.0, 0.0, 0.0, 1.0);
}

template <typename _T>
void getRoto(cv::Mat G, cv::Mat& rotation){
    rotation = (cv::Mat_<_T>(3,3) << G.at<_T>(0,0), G.at<_T>(0,1), G.at<_T>(0,2),
            G.at<_T>(1,0), G.at<_T>(1,1), G.at<_T>(1,2),
            G.at<_T>(2,0), G.at<_T>(2,1), G.at<_T>(2,2));
}

template <typename _T>
void getTras(cv::Mat G, cv::Mat& translation){
    translation = (cv::Mat_<_T>(1,3) << G.at<_T>(0,3), G.at<_T>(1,3), G.at<_T>(2,3));
}


template <typename _T>
cv::Mat rotationMatrixToEulerAngles(const cv::Mat &R) {
    assert(R.rows == 3 && R.cols == 3);

    _T sy = sqrt(R.at<_T>(0,0) * R.at<_T>(0,0) +  R.at<_T>(1,0) * R.at<_T>(1,0));

    bool singular = sy < 1e-6; // Se sy è vicino a zero, la direzione dell'asse z è vicino a singolarità

    _T x, y, z;
    if (!singular) {
        x = atan2(R.at<_T>(2,1), R.at<_T>(2,2));
        y = atan2(-R.at<_T>(2,0), sy);
        z = atan2(R.at<_T>(1,0), R.at<_T>(0,0));
    } else {
        x = atan2(-R.at<_T>(1,2), R.at<_T>(1,1));
        y = atan2(-R.at<_T>(2,0), sy);
        z = 0;
    }
    cv::Mat euler_mat = (cv::Mat_<_T>(1,3) << 0,0,0);
    euler_mat.at<_T>(0,0) = x;
    euler_mat.at<_T>(0,1) = y;
    euler_mat.at<_T>(0,2) = z;
    return euler_mat;
}

template <typename _T>
cv::Mat eulerAnglesToRotationMatrix(const cv::Mat& rvec) {

    cv::Mat R_x = (cv::Mat_<_T>(3, 3) <<
                                          1, 0, 0,
            0, cos(rvec.at<_T>(0)), -sin(rvec.at<_T>(0)),
            0, sin(rvec.at<_T>(0)), cos(rvec.at<_T>(0)));

    cv::Mat R_y = (cv::Mat_<_T>(3, 3) <<
                                          cos(rvec.at<_T>(1)), 0, sin(rvec.at<_T>(1)),
            0, 1, 0,
            -sin(rvec.at<_T>(1)), 0, cos(rvec.at<_T>(1)));

    cv::Mat R_z = (cv::Mat_<_T>(3, 3) <<
                                          cos(rvec.at<_T>(2)), -sin(rvec.at<_T>(2)), 0,
            sin(rvec.at<_T>(2)), cos(rvec.at<_T>(2)), 0,
            0, 0, 1);

    cv::Mat R = R_z * R_y * R_x;

    return R;
}

template <typename _T, int _ROWS, int _COLS>
void openCv2Eigen( const cv::Mat_<_T> &cv_mat,
                   Eigen::Matrix<_T, _ROWS, _COLS> &eigen_mat )
{
    for(int r = 0; r < _ROWS; r++)
        for(int c = 0; c < _COLS; c++)
            eigen_mat(r,c) = cv_mat(r,c);
}

template <typename _T, int _ROWS, int _COLS>
void eigen2openCv( const Eigen::Matrix<_T, _ROWS, _COLS> &eigen_mat,
                   cv::Mat_<_T> &cv_mat )
{
    cv_mat = cv::Mat_<_T>(_ROWS,_COLS);

    for(int r = 0; r < _ROWS; r++)
        for(int c = 0; c < _COLS; c++)
            cv_mat(r,c) = eigen_mat(r,c);
}

template <typename _T>
void rotMat2AngleAxis( const cv::Mat &r_mat, cv::Mat &r_vec )
{
    cv::Mat_<_T>tmp_r_mat(r_mat);
    cv::Rodrigues(tmp_r_mat, r_vec );
}

template <typename _T>
void rotMat2AngleAxis( const cv::Mat &r_mat, cv::Vec<_T, 3> &r_vec )
{
    cv::Mat_<_T>tmp_r_mat(r_mat), tmp_r_vec;
    cv::Rodrigues(tmp_r_mat, tmp_r_vec );
    r_vec[0] = tmp_r_vec(0); r_vec[1] = tmp_r_vec(1); r_vec[2] = tmp_r_vec(2);
}

template <typename _T>
void rotMat2AngleAxis( const Eigen::Matrix<_T, 3, 3> &r_mat, cv::Mat &r_vec )
{
    cv::Mat_<_T> tmp_r_mat;
    eigen2openCv<_T, 3, 3>(r_mat, tmp_r_mat);
    cv::Rodrigues(tmp_r_mat, r_vec );
}

template <typename _T>
void rotMat2AngleAxis( const Eigen::Matrix<_T, 3, 3> &r_mat, cv::Vec<_T, 3> &r_vec )
{
    cv::Mat_<_T> tmp_r_vec;
    rotMat2AngleAxis<_T>( r_mat, tmp_r_vec );
    r_vec[0] = tmp_r_vec(0); r_vec[1] = tmp_r_vec(1); r_vec[2] = tmp_r_vec(2);
}

template <typename _T>
void angleAxis2RotMat( const cv::Mat &r_vec, cv::Mat &r_mat )
{
    cv::Mat_<_T>tmp_r_vec(r_vec);
    cv::Rodrigues(tmp_r_vec, r_mat );
}

template <typename _T>
void angleAxis2RotMat( const cv::Vec<_T, 3> &r_vec, cv::Mat &r_mat )
{
    cv::Mat_<_T>tmp_r_vec(r_vec);
    cv::Rodrigues(tmp_r_vec, r_mat );
}

template <typename _T>
void angleAxis2RotMat( const cv::Mat &r_vec, Eigen::Matrix<_T, 3, 3> &r_mat )
{
    cv::Mat_<_T>tmp_r_vec(r_vec), tmp_r_mat;
    cv::Rodrigues(tmp_r_vec, tmp_r_mat );
    openCv2Eigen<_T, 3, 3>( tmp_r_mat, r_mat );
}

template <typename _T>
void angleAxis2RotMat( const cv::Vec<_T, 3> &r_vec, Eigen::Matrix<_T, 3, 3> &r_mat)
{
    cv::Mat_<_T>tmp_r_vec(r_vec), tmp_r_mat;
    cv::Rodrigues(tmp_r_vec, tmp_r_mat );
    openCv2Eigen<_T, 3, 3>( tmp_r_mat, r_mat );
}

template <typename _T>
void exp2TransfMat( const cv::Mat &r_vec, const cv::Mat &t_vec, cv::Mat &g_mat )
{
    cv::Mat_<_T> tmp_t_vec(t_vec), r_mat;
    angleAxis2RotMat<_T>( r_vec, r_mat );

    g_mat = (cv::Mat_< _T >(4, 4)
            << r_mat(0,0), r_mat(0,1), r_mat(0,2), tmp_t_vec(0,0),
            r_mat(1,0), r_mat(1,1), r_mat(1,2), tmp_t_vec(1,0),
            r_mat(2,0), r_mat(2,1), r_mat(2,2), tmp_t_vec(2,0),
            0,          0,          0,          1);

}
template <typename _T>
void transfMat2Exp( const cv::Mat &g_mat, cv::Mat &r_vec, cv::Mat &t_vec )
{
    cv::Mat_<_T> tmp_g_mat(g_mat);
    cv::Mat_< _T > r_mat = (cv::Mat_< _T >(3, 3)
            << tmp_g_mat(0,0), tmp_g_mat(0,1), tmp_g_mat(0,2),
            tmp_g_mat(1,0), tmp_g_mat(1,1), tmp_g_mat(1,2),
            tmp_g_mat(2,0), tmp_g_mat(2,1), tmp_g_mat(2,2));
    rotMat2AngleAxis<_T>(r_mat, r_vec);
    t_vec = (cv::Mat_< _T >(3, 1)<<tmp_g_mat(0,3), tmp_g_mat(1,3), tmp_g_mat(2,3));
}


#endif //METRIC_CALIBRATOR_UTILS_H
