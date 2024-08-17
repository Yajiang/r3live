// Include Files
#pragma once
#include <math.h>
#include <cmath>
#include "kd_tree/ikd_Tree.h"
#include <Eigen/Core>
#include <algorithm>

#define eps_value 1e-6

struct PlaneType{
    Eigen::Vector3d p[4];
};

class FOVChecker{
public:

    FOVChecker();
    ~FOVChecker();
    void SetEnv(BoxPointType env_param);
    void SetBoxLength(double boxLenParam);
    void CheckFov(Eigen::Vector3d curPose, Eigen::Vector3d axis, double theta, double depth, vector<BoxPointType> &boxes);
    auto CheckBox(Eigen::Vector3d curPose, Eigen::Vector3d axis, double theta, double depth, const BoxPointType box) -> bool;
    auto CheckLine(Eigen::Vector3d curPose, Eigen::Vector3d axis, double theta, double depth, Eigen::Vector3d lineP, Eigen::Vector3d lineVec) ->bool;
    bool CheckSurface(Eigen::Vector3d curPose, Eigen::Vector3d axis,  double theta, double depth, PlaneType plane);
    bool CheckPoint(Eigen::Vector3d curPose, Eigen::Vector3d axis, double theta, double depth, Eigen::Vector3d point);
    bool CheckBoxInEnv(BoxPointType box);    
private:
    BoxPointType env;
    double box_length;
};

