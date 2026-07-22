//
// Created by davide on 09/10/23.
//

#ifndef METRIC_CALIBRATOR_HANDEYE_CALIBRATOR_H
#define METRIC_CALIBRATOR_HANDEYE_CALIBRATOR_H

#include "Eigen/Core"
#include "ceres/ceres.h"
#include <ceres/rotation.h>

#include "PinholeCameraModel.h"

struct CalibReprojectionError
{
    CalibReprojectionError( const PinholeCameraModel &cam_model,
                            const Eigen::Vector3d &robot_r_vec,
                            const Eigen::Vector3d &robot_t_vec,
                            const Eigen::Vector3d &pattern_pt,
                            const Eigen::Vector2d &observed_pt ) :
            cam_model(cam_model),
            robot_r_vec(robot_r_vec),
            robot_t_vec(robot_t_vec),
            pattern_pt(pattern_pt),
            observed_pt( observed_pt) {}

    //template <typename T>
    bool operator()(//const double* const weights,
            const double* const board2ee,
            const double* const h2e,
            double* residuals) const
    {
        //T ptn_pt[3] = {T(pattern_pt(0)), T(pattern_pt(1)), T(pattern_pt(2))},
        //  tcp_pt[3], base_r[3], cam_pt[3];
        double ptn_pt[3] = {double(pattern_pt(0)), double(pattern_pt(1)), double(pattern_pt(2))},
                tcp_pt[3], base_r[3], cam_pt[3];

        // Apply the current board2ee transformation
        ceres::AngleAxisRotatePoint(board2ee, ptn_pt, tcp_pt);

        tcp_pt[0] += board2ee[3];
        tcp_pt[1] += board2ee[4];
        tcp_pt[2] += board2ee[5];

        double robot_r[3] = {double(robot_r_vec(0)), double(robot_r_vec(1)), double(robot_r_vec(2))},
               robot_t[3] = {double(robot_t_vec(0)), double(robot_t_vec(1)), double(robot_t_vec(2))};

        // Apply the robot position transformation
        ceres::AngleAxisRotatePoint(robot_r, tcp_pt, base_r);

        base_r[0] += robot_t[0];
        base_r[1] += robot_t[1];
        base_r[2] += robot_t[2];

        // Apply the current h2e transformation
        ceres::AngleAxisRotatePoint(h2e, base_r, cam_pt);

        cam_pt[0] += h2e[3];
        cam_pt[1] += h2e[4];
        cam_pt[2] += h2e[5];


        // Projection.
        double proj_pt[2];
        cam_model.project(cam_pt, proj_pt);

        residuals[0] = (proj_pt[0] - double(observed_pt(0)));
        residuals[1] = (proj_pt[1] - double(observed_pt(1)));
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create( const PinholeCameraModel &cam_model,
                                        const Eigen::Vector3d &robot_r_vec,
                                        const Eigen::Vector3d &robot_t_vec,
                                        const Eigen::Vector3d &pattern_pt,
                                        const Eigen::Vector2d &observed_pt )
    {
        return (new ceres::NumericDiffCostFunction<CalibReprojectionError, ceres::RIDDERS,  2, 6, 6>(
                new CalibReprojectionError( cam_model, robot_r_vec, robot_t_vec, pattern_pt, observed_pt )));
    }

    const PinholeCameraModel &cam_model;
    Eigen::Vector3d robot_r_vec;
    Eigen::Vector3d robot_t_vec;
    Eigen::Vector3d pattern_pt;
    Eigen::Vector2d observed_pt;
};


struct EyeInHandCalibReprojectionError
{
    EyeInHandCalibReprojectionError(const PinholeCameraModel &cam_model,
                                    const Eigen::Vector3d &robot_r_vec,
                                    const Eigen::Vector3d &robot_t_vec,
                                    const Eigen::Vector3d &pattern_pt,
                                    const Eigen::Vector2d &observed_pt ) :
            cam_model(cam_model),
            robot_r_vec(robot_r_vec),
            robot_t_vec(robot_t_vec),
            pattern_pt(pattern_pt),
            observed_pt(observed_pt) {}

    bool operator()(
            const double* const b2w,
            const double* const h2e,
            double* residuals) const
    {
        double ptn_pt[3] = {double(pattern_pt(0)), double(pattern_pt(1)), double(pattern_pt(2))},
                tcp_pt[3], base_r[3], cam_pt[3], rot_mat_h2e[9], rot_mat_h2e_inv[9], h2e_vec_inv[3], h2e_tras_vec_inv[3];

        // Apply the current h2e transformation
        ceres::AngleAxisRotatePoint(b2w, ptn_pt, base_r);

        base_r[0] += b2w[3];
        base_r[1] += b2w[4];
        base_r[2] += b2w[5];

        double robot_r[3] = {double(robot_r_vec(0)), double(robot_r_vec(1)), double(robot_r_vec(2))},
          robot_t[3] = {double(robot_t_vec(0)), double(robot_t_vec(1)), double(robot_t_vec(2))};

        // Apply the robot position transformation
        ceres::AngleAxisRotatePoint(robot_r, base_r, tcp_pt);

        tcp_pt[0] += robot_t[0];
        tcp_pt[1] += robot_t[1];
        tcp_pt[2] += robot_t[2];

        // Apply the current h2e transformation
        ceres::AngleAxisToRotationMatrix(h2e, rot_mat_h2e);
        cv::Mat h2e_mat = (cv::Mat_<double>(4,4) <<
                rot_mat_h2e[0], rot_mat_h2e[3], rot_mat_h2e[6], h2e[3],
                rot_mat_h2e[1], rot_mat_h2e[4], rot_mat_h2e[7], h2e[4],
                rot_mat_h2e[2], rot_mat_h2e[5], rot_mat_h2e[8], h2e[5],
                0, 0, 0, 1);

        cv::Mat h2e_mat_inv = h2e_mat.inv();

        rot_mat_h2e_inv[0] = h2e_mat_inv.at<double>(0,0);
        rot_mat_h2e_inv[1] = h2e_mat_inv.at<double>(1,0);
        rot_mat_h2e_inv[2] = h2e_mat_inv.at<double>(2,0);
        rot_mat_h2e_inv[3] = h2e_mat_inv.at<double>(0,1);
        rot_mat_h2e_inv[4] = h2e_mat_inv.at<double>(1,1);
        rot_mat_h2e_inv[5] = h2e_mat_inv.at<double>(2,1);
        rot_mat_h2e_inv[6] = h2e_mat_inv.at<double>(0,2);
        rot_mat_h2e_inv[7] = h2e_mat_inv.at<double>(1,2);
        rot_mat_h2e_inv[8] = h2e_mat_inv.at<double>(2,2);

        ceres::RotationMatrixToAngleAxis(rot_mat_h2e_inv, h2e_vec_inv);

        h2e_tras_vec_inv[0] = h2e_mat_inv.at<double>(0,3);
        h2e_tras_vec_inv[1] = h2e_mat_inv.at<double>(1,3);
        h2e_tras_vec_inv[2] = h2e_mat_inv.at<double>(2,3);

        ceres::AngleAxisRotatePoint(h2e_vec_inv, tcp_pt, cam_pt);

        cam_pt[0] += h2e_tras_vec_inv[0];
        cam_pt[1] += h2e_tras_vec_inv[1];
        cam_pt[2] += h2e_tras_vec_inv[2];


        // Projection.
        double proj_pt[2];
        cam_model.project(cam_pt, proj_pt);

        residuals[0] = (proj_pt[0] - double(observed_pt(0)));
        residuals[1] = (proj_pt[1] - double(observed_pt(1)));
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create( const PinholeCameraModel &cam_model,
                                        const Eigen::Vector3d &robot_r_vec,
                                        const Eigen::Vector3d &robot_t_vec,
                                        const Eigen::Vector3d &pattern_pt,
                                        const Eigen::Vector2d &observed_pt )
    {
        return (new ceres::NumericDiffCostFunction<EyeInHandCalibReprojectionError, ceres::CENTRAL, 2, 6, 6>(
                new EyeInHandCalibReprojectionError( cam_model, robot_r_vec, robot_t_vec, pattern_pt, observed_pt )));
    }

    const PinholeCameraModel &cam_model;
    Eigen::Vector3d robot_r_vec;
    Eigen::Vector3d robot_t_vec;
    Eigen::Vector3d pattern_pt;
    Eigen::Vector2d observed_pt;
};




struct MultiCalibReprojectionError
{
    MultiCalibReprojectionError( const PinholeCameraModel &cam_model,
                                 const Eigen::Vector3d &robot_r_vec,
                                 const Eigen::Vector3d &robot_t_vec,
                                 const Eigen::Vector3d &pattern_pt,
                                 const Eigen::Vector2d &observed_pt ) :
            cam_model(cam_model),
            robot_r_vec(robot_r_vec),
            robot_t_vec(robot_t_vec),
            pattern_pt(pattern_pt),
            observed_pt( observed_pt) {}

    template <typename T>
    bool operator()(const double* const board2tcp,
                    const double* const h2e,
                    const double* const cam2cam,
                    T* residuals) const
    {
        T ptn_pt[3] = {T(pattern_pt(0)), T(pattern_pt(1)), T(pattern_pt(2))},
                tcp_pt[3], base_pt[3], cam_pt[3], cam2_pt[3];

        // Apply the current board2tcp transformation
        ceres::AngleAxisRotatePoint(board2tcp, ptn_pt, tcp_pt);

        tcp_pt[0] += board2tcp[3];
        tcp_pt[1] += board2tcp[4];
        tcp_pt[2] += board2tcp[5];


        T robot_r[3] = {T(robot_r_vec(0)), T(robot_r_vec(1)), T(robot_r_vec(2))},
                robot_t[3] = {T(robot_t_vec(0)), T(robot_t_vec(1)), T(robot_t_vec(2))};

        // Apply the robot position transformation
        ceres::AngleAxisRotatePoint(robot_r, tcp_pt, base_pt);

        base_pt[0] += robot_t[0];
        base_pt[1] += robot_t[1];
        base_pt[2] += robot_t[2];

        // Apply the current h2e transformation
        ceres::AngleAxisRotatePoint(h2e, base_pt, cam_pt);

        cam_pt[0] += h2e[3];
        cam_pt[1] += h2e[4];
        cam_pt[2] += h2e[5];


        // Apply the current cam2cam transformation
        ceres::AngleAxisRotatePoint(cam2cam, cam_pt, cam2_pt);

        cam2_pt[0] += cam2cam[3];
        cam2_pt[1] += cam2cam[4];
        cam2_pt[2] += cam2cam[5];

        // Projection.
        T proj_pt[2];
        cam_model.project(cam2_pt, proj_pt);

        // The error is the difference between the predicted and observed position.
        residuals[0] = proj_pt[0] - T(observed_pt(0));
        residuals[1] = proj_pt[1] - T(observed_pt(1));

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create( const PinholeCameraModel &cam_model,
                                        const Eigen::Vector3d &robot_r_vec,
                                        const Eigen::Vector3d &robot_t_vec,
                                        const Eigen::Vector3d &pattern_pt,
                                        const Eigen::Vector2d &observed_pt )
    {
        return (new ceres::NumericDiffCostFunction<MultiCalibReprojectionError, ceres::CENTRAL, 2, 6, 6, 6>(
                new MultiCalibReprojectionError( cam_model, robot_r_vec, robot_t_vec, pattern_pt, observed_pt )));
    }

    const PinholeCameraModel &cam_model;
    Eigen::Vector3d robot_r_vec;
    Eigen::Vector3d robot_t_vec;
    Eigen::Vector3d pattern_pt;
    Eigen::Vector2d observed_pt;
};



struct MultiEyeInHandCalibReprojectionError
{
    MultiEyeInHandCalibReprojectionError( const PinholeCameraModel &cam_model,
                                          const Eigen::Vector3d &robot_r_vec,
                                          const Eigen::Vector3d &robot_t_vec,
                                          const Eigen::Vector3d &pattern_pt,
                                          const Eigen::Vector2d &observed_pt ) :
            cam_model(cam_model),
            robot_r_vec(robot_r_vec),
            robot_t_vec(robot_t_vec),
            pattern_pt(pattern_pt),
            observed_pt( observed_pt) {}

    bool operator()(const double* const b2w,
                    const double* const h2e,
                    const double* const c2c,
                    double* residuals) const
    {
        double ptn_pt[3] = {double(pattern_pt(0)), double(pattern_pt(1)), double(pattern_pt(2))},
                tcp_pt[3], base_pt[3], cam_pt[3], cam2_pt[3], rot_mat[9], rot_mat_inv[9], rot_vec[3], tras_vec[3];

        // Apply the current board2tcp transformation
        ceres::AngleAxisRotatePoint(b2w, ptn_pt, base_pt);

        base_pt[0] += b2w[3];
        base_pt[1] += b2w[4];
        base_pt[2] += b2w[5];


        double robot_r[3] = {double(robot_r_vec(0)), double(robot_r_vec(1)), double(robot_r_vec(2))},
                robot_t[3] = {double(robot_t_vec(0)), double(robot_t_vec(1)), double(robot_t_vec(2))};

        // Apply the robot position transformation
        ceres::AngleAxisRotatePoint(robot_r, base_pt, tcp_pt);

        tcp_pt[0] += robot_t[0];
        tcp_pt[1] += robot_t[1];
        tcp_pt[2] += robot_t[2];





        //###########################################################################################

        ceres::AngleAxisToRotationMatrix(h2e, rot_mat);

        cv::Mat temp_g = (cv::Mat_<double>(4,4) << rot_mat[0], rot_mat[3], rot_mat[6], h2e[3],
                rot_mat[1], rot_mat[4], rot_mat[7], h2e[4],
                rot_mat[2], rot_mat[5], rot_mat[8], h2e[5],
                0, 0, 0, 1);

        cv::Mat temp_g_inv = temp_g.inv();

        rot_mat_inv[0] = temp_g_inv.at<double>(0,0);
        rot_mat_inv[1] = temp_g_inv.at<double>(1,0);
        rot_mat_inv[2] = temp_g_inv.at<double>(2,0);
        rot_mat_inv[3] = temp_g_inv.at<double>(0,1);
        rot_mat_inv[4] = temp_g_inv.at<double>(1,1);
        rot_mat_inv[5] = temp_g_inv.at<double>(2,1);
        rot_mat_inv[6] = temp_g_inv.at<double>(0,2);
        rot_mat_inv[7] = temp_g_inv.at<double>(1,2);
        rot_mat_inv[8] = temp_g_inv.at<double>(2,2);

        ceres::RotationMatrixToAngleAxis(rot_mat_inv, rot_vec);

        tras_vec[0] = temp_g_inv.at<double>(0,3);
        tras_vec[1] = temp_g_inv.at<double>(1,3);
        tras_vec[2] = temp_g_inv.at<double>(2,3);

        //###########################################################################################


        // Apply the current h2e transformation
        ceres::AngleAxisRotatePoint(rot_vec, tcp_pt, cam_pt);

        cam_pt[0] += tras_vec[0];
        cam_pt[1] += tras_vec[1];
        cam_pt[2] += tras_vec[2];


        // Apply the current cam2cam transformation
        ceres::AngleAxisRotatePoint(c2c, cam_pt, cam2_pt);

        cam2_pt[0] += c2c[3];
        cam2_pt[1] += c2c[4];
        cam2_pt[2] += c2c[5];

        // Projection.
        double proj_pt[2];
        cam_model.project(cam2_pt, proj_pt);

        // The error is the difference between the predicted and observed position.
        residuals[0] = proj_pt[0] - double(observed_pt(0));
        residuals[1] = proj_pt[1] - double(observed_pt(1));

        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create( const PinholeCameraModel &cam_model,
                                        const Eigen::Vector3d &robot_r_vec,
                                        const Eigen::Vector3d &robot_t_vec,
                                        const Eigen::Vector3d &pattern_pt,
                                        const Eigen::Vector2d &observed_pt )
    {
        return (new ceres::NumericDiffCostFunction<MultiEyeInHandCalibReprojectionError, ceres::CENTRAL, 2, 6, 6, 6>(
                new MultiEyeInHandCalibReprojectionError( cam_model, robot_r_vec, robot_t_vec, pattern_pt, observed_pt )));
    }

    const PinholeCameraModel &cam_model;
    Eigen::Vector3d robot_r_vec;
    Eigen::Vector3d robot_t_vec;
    Eigen::Vector3d pattern_pt;
    Eigen::Vector2d observed_pt;
};

// Class for multi camera inverse
struct MultiCalibReprojectionErrorInverse
{
    MultiCalibReprojectionErrorInverse( const PinholeCameraModel &cam_model,
                                        const Eigen::Vector3d &robot_r_vec,
                                        const Eigen::Vector3d &robot_t_vec,
                                        const Eigen::Vector3d &pattern_pt,
                                        const Eigen::Vector2d &observed_pt ) :
            cam_model(cam_model),
            robot_r_vec(robot_r_vec),
            robot_t_vec(robot_t_vec),
            pattern_pt(pattern_pt),
            observed_pt( observed_pt) {}

    //template <typename T>
    bool operator()(const double* const board2tcp,
                    const double* const h2e,
                    const double* const cam2cam,
                    double* residuals) const
    {
        double ptn_pt[3] = {double(pattern_pt(0)), double(pattern_pt(1)), double(pattern_pt(2))},
                tcp_pt[3], base_pt[3], cam_pt[3], cam2_pt[3], tras_vec[3], rot_vec[3], rot_mat[9], rot_mat_inv[9];

        // Apply the current board2tcp transformation
        ceres::AngleAxisRotatePoint(board2tcp, ptn_pt, tcp_pt);

        tcp_pt[0] += board2tcp[3];
        tcp_pt[1] += board2tcp[4];
        tcp_pt[2] += board2tcp[5];


        double robot_r[3] = {double(robot_r_vec(0)), double(robot_r_vec(1)), double(robot_r_vec(2))},
                robot_t[3] = {double(robot_t_vec(0)), double(robot_t_vec(1)), double(robot_t_vec(2))};

        // Apply the robot position transformation
        ceres::AngleAxisRotatePoint(robot_r, tcp_pt, base_pt);

        base_pt[0] += robot_t[0];
        base_pt[1] += robot_t[1];
        base_pt[2] += robot_t[2];

        // Apply the current h2e transformation
        ceres::AngleAxisRotatePoint(h2e, base_pt, cam_pt);

        cam_pt[0] += h2e[3];
        cam_pt[1] += h2e[4];
        cam_pt[2] += h2e[5];

        //###########################################################################################
        cv::Mat temp_variable;

        cv::Mat cam2cam_mat_inv;
        cv::Mat cam2cam_mat = (cv::Mat_<double>(1,3) << cam2cam[0], cam2cam[1], cam2cam[2]);
        ceres::AngleAxisToRotationMatrix(cam2cam, rot_mat);

        cv::Mat temp_g = (cv::Mat_<double>(4,4) <<
                                                rot_mat[0], rot_mat[3], rot_mat[6], cam2cam[3],
                rot_mat[1], rot_mat[4], rot_mat[7], cam2cam[4],
                rot_mat[2], rot_mat[5], rot_mat[8], cam2cam[5],
                0, 0, 0, 1);

        cv::Mat temp_g_inv = temp_g.inv();

        rot_mat_inv[0] = temp_g_inv.at<double>(0,0);
        rot_mat_inv[1] = temp_g_inv.at<double>(1,0);
        rot_mat_inv[2] = temp_g_inv.at<double>(2,0);
        rot_mat_inv[3] = temp_g_inv.at<double>(0,1);
        rot_mat_inv[4] = temp_g_inv.at<double>(1,1);
        rot_mat_inv[5] = temp_g_inv.at<double>(2,1);
        rot_mat_inv[6] = temp_g_inv.at<double>(0,2);
        rot_mat_inv[7] = temp_g_inv.at<double>(1,2);
        rot_mat_inv[8] = temp_g_inv.at<double>(2,2);

        ceres::RotationMatrixToAngleAxis(rot_mat_inv, rot_vec);

        tras_vec[0] = temp_g_inv.at<double>(0,3);
        tras_vec[1] = temp_g_inv.at<double>(1,3);
        tras_vec[2] = temp_g_inv.at<double>(2,3);

        //###########################################################################################

        ceres::AngleAxisRotatePoint(rot_vec, cam_pt, cam2_pt);

        cam2_pt[0] += tras_vec[0];
        cam2_pt[1] += tras_vec[1];
        cam2_pt[2] += tras_vec[2];

        // Projection.
        double proj_pt[2];
        cam_model.project(cam2_pt, proj_pt);

        // The error is the difference between the predicted and observed position.
        residuals[0] = proj_pt[0] - double(observed_pt(0));
        residuals[1] = proj_pt[1] - double(observed_pt(1));

        return true;
    }

    static ceres::CostFunction* Create( const PinholeCameraModel &cam_model,
                                        const Eigen::Vector3d &robot_r_vec,
                                        const Eigen::Vector3d &robot_t_vec,
                                        const Eigen::Vector3d &pattern_pt,
                                        const Eigen::Vector2d &observed_pt )
    {
        return (new ceres::NumericDiffCostFunction<MultiCalibReprojectionErrorInverse, ceres::CENTRAL, 2, 6, 6, 6>(
                new MultiCalibReprojectionErrorInverse( cam_model, robot_r_vec, robot_t_vec, pattern_pt, observed_pt )));
    }

    const PinholeCameraModel &cam_model;
    Eigen::Vector3d robot_r_vec;
    Eigen::Vector3d robot_t_vec;
    Eigen::Vector3d pattern_pt;
    Eigen::Vector2d observed_pt;
};


// Class for multi camera inverse
struct MultiEyeInHandCalibReprojectionErrorInverse
{
    MultiEyeInHandCalibReprojectionErrorInverse( const PinholeCameraModel &cam_model,
                                        const Eigen::Vector3d &robot_r_vec,
                                        const Eigen::Vector3d &robot_t_vec,
                                        const Eigen::Vector3d &pattern_pt,
                                        const Eigen::Vector2d &observed_pt ) :
            cam_model(cam_model),
            robot_r_vec(robot_r_vec),
            robot_t_vec(robot_t_vec),
            pattern_pt(pattern_pt),
            observed_pt( observed_pt) {}

    //template <typename T>
    bool operator()(const double* const b2w,
                    const double* const h2e,
                    const double* const c2c,
                    double* residuals) const
    {
        double ptn_pt[3] = {double(pattern_pt(0)), double(pattern_pt(1)), double(pattern_pt(2))},
                tcp_pt[3], base_pt[3], cam_pt[3], cam2_pt[3], tras_vec[3], rot_vec[3], rot_mat[9], rot_mat_inv[9];

        // Apply the current board2tcp transformation
        ceres::AngleAxisRotatePoint(b2w, ptn_pt, base_pt);

        base_pt[0] += b2w[3];
        base_pt[1] += b2w[4];
        base_pt[2] += b2w[5];


        double robot_r[3] = {double(robot_r_vec(0)), double(robot_r_vec(1)), double(robot_r_vec(2))},
                robot_t[3] = {double(robot_t_vec(0)), double(robot_t_vec(1)), double(robot_t_vec(2))};

        // Apply the robot position transformation
        ceres::AngleAxisRotatePoint(robot_r, base_pt, tcp_pt);

        tcp_pt[0] += robot_t[0];
        tcp_pt[1] += robot_t[1];
        tcp_pt[2] += robot_t[2];

        // Apply the current h2e transformation
        ceres::AngleAxisRotatePoint(h2e, tcp_pt, cam_pt);

        cam_pt[0] += h2e[3];
        cam_pt[1] += h2e[4];
        cam_pt[2] += h2e[5];

        //###########################################################################################
        cv::Mat temp_variable;

        cv::Mat cam2cam_mat_inv;
        cv::Mat cam2cam_mat = (cv::Mat_<double>(1,3) << c2c[0], c2c[1], c2c[2]);
        ceres::AngleAxisToRotationMatrix(c2c, rot_mat);

        cv::Mat temp_g = (cv::Mat_<double>(4,4) <<
                rot_mat[0], rot_mat[3], rot_mat[6], c2c[3],
                rot_mat[1], rot_mat[4], rot_mat[7], c2c[4],
                rot_mat[2], rot_mat[5], rot_mat[8], c2c[5],
                0, 0, 0, 1);

        cv::Mat temp_g_inv = temp_g.inv();

        rot_mat_inv[0] = temp_g_inv.at<double>(0,0);
        rot_mat_inv[1] = temp_g_inv.at<double>(1,0);
        rot_mat_inv[2] = temp_g_inv.at<double>(2,0);
        rot_mat_inv[3] = temp_g_inv.at<double>(0,1);
        rot_mat_inv[4] = temp_g_inv.at<double>(1,1);
        rot_mat_inv[5] = temp_g_inv.at<double>(2,1);
        rot_mat_inv[6] = temp_g_inv.at<double>(0,2);
        rot_mat_inv[7] = temp_g_inv.at<double>(1,2);
        rot_mat_inv[8] = temp_g_inv.at<double>(2,2);

        ceres::RotationMatrixToAngleAxis(rot_mat_inv, rot_vec);

        tras_vec[0] = temp_g_inv.at<double>(0,3);
        tras_vec[1] = temp_g_inv.at<double>(1,3);
        tras_vec[2] = temp_g_inv.at<double>(2,3);

        //###########################################################################################

        ceres::AngleAxisRotatePoint(rot_vec, cam_pt, cam2_pt);

        cam2_pt[0] += tras_vec[0];
        cam2_pt[1] += tras_vec[1];
        cam2_pt[2] += tras_vec[2];

        // Projection.
        double proj_pt[2];
        cam_model.project(cam2_pt, proj_pt);

        // The error is the difference between the predicted and observed position.
        residuals[0] = proj_pt[0] - double(observed_pt(0));
        residuals[1] = proj_pt[1] - double(observed_pt(1));

        return true;
    }

    static ceres::CostFunction* Create( const PinholeCameraModel &cam_model,
                                        const Eigen::Vector3d &robot_r_vec,
                                        const Eigen::Vector3d &robot_t_vec,
                                        const Eigen::Vector3d &pattern_pt,
                                        const Eigen::Vector2d &observed_pt )
    {
        return (new ceres::NumericDiffCostFunction<MultiEyeInHandCalibReprojectionErrorInverse, ceres::CENTRAL, 2, 6, 6, 6>(
                new MultiEyeInHandCalibReprojectionErrorInverse( cam_model, robot_r_vec, robot_t_vec, pattern_pt, observed_pt )));
    }

    const PinholeCameraModel &cam_model;
    Eigen::Vector3d robot_r_vec;
    Eigen::Vector3d robot_t_vec;
    Eigen::Vector3d pattern_pt;
    Eigen::Vector2d observed_pt;
};


#endif //METRIC_CALIBRATOR_HANDEYE_CALIBRATOR_H
