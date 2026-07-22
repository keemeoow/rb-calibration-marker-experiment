//
// Created by davide on 12/09/23.
//

#include <fstream>
#include "utils.h"

namespace fs = std::filesystem;

void checkData(std::string data_folder, std::string prefix, int number_of_cameras){
    int folder_count = 0;
    for (const auto& entry: fs::directory_iterator(data_folder)){
        if (entry.path().filename().string().find(prefix) == 0){
            folder_count ++;
        }
    }

    // Stop the program if the number of selected cameras is different from the number of provided folders
    if (number_of_cameras != folder_count) {
        std::cerr << "The number of selected cameras does not coincide with the number of provided folders!" << std::endl;
        exit(EXIT_FAILURE);
    }
}

int countImages(const std::string& path) {
    int count = 0;
    for (const auto& entry : fs::directory_iterator(path)) {
        if (entry.is_regular_file()) {
            auto ext = entry.path().extension().string();
            
            // Convert extension to lowercase to make the comparison case-insensitive
            std::transform(ext.begin(), ext.end(), ext.begin(),
                           [](unsigned char c){ return std::tolower(c); });
            if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
                count++;
            }
        }
    }
    return count;
}

int countFolderImages(const std::string& base_path, const std::string& cam_prefix) {
    for (const auto& entry : fs::directory_iterator(base_path)) {
        if (entry.is_directory()) {
            const std::string folder_name = entry.path().filename().string();
            if (folder_name.rfind(cam_prefix, 0) == 0) {
                std::string image_folder = entry.path().string() + "/image";
                if (fs::exists(image_folder) && fs::is_directory(image_folder)) {
                    return countImages(image_folder);
                }
            }
        }
    }
    throw std::runtime_error("No matching camera*/image folder found.");
}

bool isFolderNotEmpty(const std::string& folder_path) {
    if (!fs::exists(folder_path)) {
        // The folder does not exist
        std::cerr << "The folder does not exist!" << std::endl;
        return false;
    }

    for (const auto& entry : fs::directory_iterator(folder_path)) {
        // If there is at least one entry in the folder, it's not empty
        return true;
    }

    // The folder is empty
    std::cerr << "The folder is empty!" << std::endl;
    return false;
}


bool compareFilenames(const fs::directory_entry& a, const fs::directory_entry& b) {
    return a.path().filename().string() < b.path().filename().string();
}



void readPoseFromCSV(const std::string& input_path, cv::Mat& out_mat, char delim)
{
    std::ifstream inputfile(input_path);
    std::string current_line;
    std::vector<std::vector<float> > all_data;
    while(getline(inputfile, current_line)){
        std::vector<float> values;
        std::stringstream temp(current_line);
        std::string single_value;
        while(getline(temp,single_value,delim)){
            float f = std::stof(single_value.c_str());
            values.push_back(f);
        }
        all_data.push_back(values);
    }

    out_mat = cv::Mat::zeros((int)all_data.size(), (int)all_data[0].size(), CV_32FC1);
    for(int rows = 0; rows < (int)all_data.size(); rows++){
        for(int cols= 0; cols< (int)all_data[0].size(); cols++){
            out_mat.at<float>(rows,cols) = all_data[rows][cols];
        }
    }
}


void setInitialGuess(std::vector<cv::Mat> &h2e_initial_guess_vec, cv::Mat &b2ee_initial_guess, const std::vector<std::vector<cv::Mat>> rototras_vec, const std::vector<std::vector<cv::Mat>> correct_poses, const int calibration_setup){

    if (calibration_setup==1) {
        b2ee_initial_guess = (cv::Mat_<double>(4, 4) <<
                                                    1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1);

        for (int i = 0; i < h2e_initial_guess_vec.size(); i++) {
            h2e_initial_guess_vec[i] = (cv::Mat_<double>(4, 4) <<
                                                               1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1);

            cv::Mat double_matrix1, double_matrix2, double_matrix3;
            rototras_vec[i][0].convertTo(double_matrix1, CV_64F);
            b2ee_initial_guess.convertTo(double_matrix2, CV_64F);
            correct_poses[i][0].convertTo(double_matrix3, CV_64F);
            h2e_initial_guess_vec[i] = double_matrix1 * double_matrix2 * double_matrix3.inv();
        }
    }
    else{

        for (int i = 0; i < h2e_initial_guess_vec.size(); i++) {
            h2e_initial_guess_vec[i] = (cv::Mat_<double>(4, 4) <<
                                                               1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1);

        }

        /*b2ee_initial_guess = (cv::Mat_<double>(4, 4) <<
                                                     1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1);*/
        cv::Mat double_matrix1, double_matrix2, double_matrix3;
        rototras_vec[0][0].convertTo(double_matrix1, CV_64F);
        h2e_initial_guess_vec[0].convertTo(double_matrix2, CV_64F);
        correct_poses[0][0].convertTo(double_matrix3, CV_64F);
        b2ee_initial_guess = double_matrix3 * double_matrix2 * double_matrix1;
    }
};

bool filterRansac(std::vector<cv::Point3f> object_points, std::vector<cv::Point2f> corners, CameraInfo camera_info, cv::Mat& rvec, cv::Mat& tvec){

    //cv::Mat rvec, tvec;
    std::vector<int> inliers;
    cv::solvePnPRansac(object_points, corners, camera_info.getCameraMatrix(),camera_info.getDistCoeff(), rvec, tvec, false, 50, 8, 0.99, inliers);
    //cv::solvePnP(object_points, corners, camera_info.getCameraMatrix(), camera_info.getDistCoeff(), rvec, tvec);
    if (inliers.size() == object_points.size())
        return true;
    else
        return false;
    //return true;

}

void createFolder(std::string folder_name){
    if (!fs::is_directory(folder_name)) {
        try {
            fs::create_directories(folder_name);
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Error creating the folder: " << e.what() << std::endl;
        }
    }
}

int linesNumber(const std::string file_path){
    std::vector<std::string> existing_data;
    std::ifstream input_file(file_path);
    if (input_file.is_open()) {
        std::string line;
        while (std::getline(input_file, line)) {
            existing_data.push_back(line);
        }
        input_file.close();
    }

    return existing_data.size();
}
