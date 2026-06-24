//
// Created by Davide on 12/09/24.
//

#ifndef METRIC_CALIBRATOR_CALIBRATOR_H
#define METRIC_CALIBRATOR_CALIBRATOR_H

#include <iostream>
#include <string>
#include <chrono>

#include "Reader.h"
#include "Detector.h"
#include "MultiHandEyeCalibrator.h"
#include "utils.h"
#include "Metrics.h"

#include <Eigen/Dense>

class Calibrator{
private:
    std::string data_;
    int number_of_waypoints_;
    int start_index_;

public:
    Calibrator(std::string data_folder, int number_of_waypoints, int start_index){data_ = data_folder; number_of_waypoints_ = number_of_waypoints; start_index_ = start_index;};
    void calibration();
};

#endif //METRIC_CALIBRATOR_CALIBRATOR_H
