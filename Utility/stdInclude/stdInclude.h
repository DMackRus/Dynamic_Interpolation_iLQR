//
// Created by davem on 21/02/2022.
//

#ifndef MUJOCO_SANDBOX_STDINCLUDE_H
#define MUJOCO_SANDBOX_STDINCLUDE_H

#include <iostream>
#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <stdio.h>
#include <string>
#include <vector>
#include <math.h>
#include <omp.h>
#include <random>

using namespace Eigen;
using namespace std;
using namespace std::chrono;

#define PI 3.141519265
#define MUJ_STEPS_HORIZON_LENGTH 2500
#define MUJOCO_DT 0.004

#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2

enum VecPos{
    x = 0,
    y = 1,
    z = 2
};

typedef Matrix<double, 3, 1> m_point;
typedef Matrix<double, 4, 1> m_quat;
typedef Matrix<double, 6, 1> m_pose;


#endif //MUJOCO_SANDBOX_STDINCLUDE_H
