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
#define MUJ_STEPS_HORIZON_LENGTH 2000
#define MUJOCO_DT 0.004

enum VecPos{
    x,
    y,
    z
};

typedef Matrix<double, 3, 1> m_point;
typedef Matrix<double, 4, 1> m_quat;
typedef Matrix<double, 6, 1> m_pose;


#endif //MUJOCO_SANDBOX_STDINCLUDE_H
