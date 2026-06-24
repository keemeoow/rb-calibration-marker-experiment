//
// Created by davide on 12/09/23.
//

#include "Calibrator.h"

int MAX_IMAGES = 300;


void Calibrator::calibration() {

    std::cout << ">> Start the calibration!" << std::endl;

    //------------------------------ DATA READING ------------------------------
    // Create the Calibration Info structure
    CalibrationInfo calib_info;
    
    // Read the provided data
    Reader reader(data_);
    reader.readCalibrationInfo(calib_info);
    int calibration_setup = calib_info.getCalibSetup();

    const int number_of_cameras = calib_info.getNumberOfCams();
    // int total_images = countImages(data_ + "/" + calib_info.getCamFolderPref() + "1/image");
    int total_images = countFolderImages(data_, calib_info.getCamFolderPref());
    int number_of_waypoints;
    int start_index = 0;
    int end_index = total_images;
    
    number_of_waypoints = total_images;
    start_index = 0;
    
    // Check reliability of the provided data
    checkData(data_, calib_info.getCamFolderPref(), number_of_cameras);

    // Create the Camera Network Info structure
    std::vector <CameraInfo> camera_network_info(number_of_cameras);
    for (int i = 0; i < camera_network_info.size(); i++) {
        camera_network_info[i].setParameters(data_ + calib_info.getCamFolderPref() + std::to_string(i + 1),
                                             calib_info.getResizeFactor());
    }

    // Read the image and pose collections
    std::vector <std::vector<cv::Mat>> images_collected = reader.readImages(number_of_cameras, calib_info.getResizeFactor(), start_index, end_index);
    std::vector <std::vector<cv::Mat>> poses_collected = reader.readRobotPoses(number_of_cameras, start_index, end_index);


    // ------------------------------ DATA COLLECTION ------------------------------
    // Initialize the vectors of correct images, poses, corners and the cross observation matrix
    std::vector <std::vector<cv::Mat>> correct_images(number_of_cameras), correct_poses(number_of_cameras);
    std::vector < std::vector < std::vector < cv::Point2f>>> correct_corners_only(number_of_cameras), correct_corners(number_of_waypoints, std::vector<std::vector<cv::Point2f >> (number_of_cameras));
    std::vector <std::vector<int>> cross_observation_matrix(number_of_waypoints, std::vector<int>(number_of_cameras, 0));
    std::vector <std::vector<cv::Mat>> rvec_all(number_of_waypoints, std::vector<cv::Mat>(number_of_cameras)), tvec_all(number_of_waypoints, std::vector<cv::Mat>(number_of_cameras)), rvec_used(number_of_cameras, std::vector<cv::Mat>(number_of_waypoints)), tvec_used(number_of_cameras, std::vector<cv::Mat>(number_of_waypoints));

    // Detect the calibration pattern
    Detector detector(calib_info, camera_network_info, number_of_waypoints);
    detector.patternDetection(images_collected, poses_collected, correct_images, correct_poses, correct_corners_only, correct_corners, cross_observation_matrix, rvec_all, tvec_all);


    // Convert robot poses with double values
    for (int j = 0; j < number_of_cameras; j++) {
        for (int i = 0; i < poses_collected[0].size(); i++) {
            poses_collected[j][i].convertTo(poses_collected[j][i], CV_64F);
        }
    }


    std::vector <std::vector<cv::Mat>> rototras_vec(number_of_cameras), rototras_all(number_of_cameras);
    for (int i = 0; i < number_of_cameras; i++) {
        std::vector <cv::Mat> rototras_camera(number_of_waypoints);
        std::vector <cv::Mat> rototras_camera_push;
        for (int j = 0; j < number_of_waypoints; j++) {
            if (cross_observation_matrix[j][i]) {
                cv::Mat rototras;
                cv::Mat rotation;
                cv::Rodrigues(rvec_all[j][i], rotation);
                getRotoTras<double>(rotation, tvec_all[j][i], rototras);

                getTras<double>(rototras, tvec_used[i][j]);
                cv::Mat rotation_temp;
                getRoto<double>(rototras, rotation_temp);
                cv::Rodrigues(rotation_temp, rvec_used[i][j]);

                rototras_camera[j] = rototras;
                rototras_camera_push.push_back(rototras);
            }
        }
        rototras_vec[i] = rototras_camera;
        rototras_all[i] = rototras_camera_push;
    }

    // Get 3D checkerboard points
    std::vector <cv::Point3f> object_points;
    detector.getObjectPoints(object_points);

    // Set initial guess
    std::vector <cv::Mat> h2e_initial_guess_vec(number_of_cameras);
    cv::Mat b2ee_initial_guess;
    setInitialGuess(h2e_initial_guess_vec, b2ee_initial_guess, rototras_all, correct_poses, calib_info.getCalibSetup());


    // ------------------------------------------ Calibration process ------------------------------------------
    std::vector <std::vector<cv::Mat>> multi_c2c_optimal_vec(number_of_cameras, std::vector<cv::Mat>(number_of_cameras));
    std::vector <cv::Mat> multi_h2e_optimal_vec(number_of_cameras);
    cv::Mat multi_b2ee_optimal;
    std::vector <cv::Mat> multi_b2ee_optimal_vec;

    // Initialize the calibrator object
    MultiHandEyeCalibrator multi_calibrator(number_of_waypoints, number_of_cameras, object_points, poses_collected[0],
                                            h2e_initial_guess_vec, b2ee_initial_guess, cross_observation_matrix,
                                            rvec_used, tvec_used, rototras_vec);


    switch (calib_info.getCalibSetup()) {
        case 0:
            multi_calibrator.eyeInHandCalibration(camera_network_info, correct_corners, multi_h2e_optimal_vec,
                                                  multi_b2ee_optimal, multi_c2c_optimal_vec, images_collected);
            break;
        case 1:
            multi_calibrator.eyeOnBaseCalibration(camera_network_info, correct_corners, multi_h2e_optimal_vec,
                                                  multi_b2ee_optimal, multi_c2c_optimal_vec);

            break;
    }

    multi_b2ee_optimal_vec.push_back(multi_b2ee_optimal);


    for (int i = 0; i < number_of_waypoints; i++){
        poses_collected[0][i].convertTo(poses_collected[0][i], CV_64F);
    }

    // ------------------------------------------ Metric evaluation ------------------------------------------
    Metrics metric(object_points, multi_h2e_optimal_vec, multi_b2ee_optimal_vec, camera_network_info, calib_info, data_, cross_observation_matrix);

    // Multi camera calibration results
    createFolder(data_ +  "/results");

    // Initialize the metric object for each camera
    for (int i = 0; i < number_of_cameras; i++) {
        std::cout << ">> Camera " << i + 1 << " calibration results" << std::endl;
        if (calib_info.getCalibSetup()==0){
            std::cout << "Eye-in-hand" << std::endl;
            std::cout << "Camera with respect to the Hand: " << multi_h2e_optimal_vec[i] << std::endl;
            std::cout << "Board with respect to the Robot: " << multi_b2ee_optimal_vec[0] << std::endl;
        }
        if (calib_info.getCalibSetup()==1){
            std::cout << "Eye-on-base" << std::endl;
            std::cout << "Camera with respect to the Robot: " << multi_h2e_optimal_vec[i].inv() << std::endl;
            std::cout << "Board with respect to the Hand: " << multi_b2ee_optimal_vec[0] << std::endl;
        }
    }

    // Reproject corners
    std::vector<std::vector<std::vector<cv::Point2f>>> corner_points_reprojected(number_of_cameras, std::vector<std::vector<cv::Point2f>>(number_of_waypoints));
    metric.projectCorners(correct_corners, poses_collected[0], images_collected, corner_points_reprojected);

    // Save reprojection error
    metric.reprojectionError(correct_corners, corner_points_reprojected);
}
