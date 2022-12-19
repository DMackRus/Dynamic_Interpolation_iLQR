//
// Created by dave on 19/12/22.
//

#ifndef MUJOCO_ACROBOT_CONTROL_STOMP_H
#define MUJOCO_ACROBOT_CONTROL_STOMP_H

#include "mujoco.h"
#include "glfw3.h"
#include "../Utility/stdInclude/stdInclude.h"
#include "../modelTranslator/modelTranslator.h"
#include "../Utility/MujocoController/MujocoUI.h"
#include <iterator>
#include <random>

#define ROLLOUTS_PER_ITERATION        100

#define VISUALISE_ROLLOUTS_STOMP      0

class STOMP{
public:
    STOMP(mjModel* m, mjData* d, taskTranslator* _modelTranslator);

    mjModel* model;
    mjData* mdata = NULL;
    mjData* d_init = NULL;
    taskTranslator *modelTranslator;
    std::vector<m_ctrl> U_best;
    int numIterations = 0;
    int maxIterations = 10;
    float epsConverge = 0.02;


    mjData *d_Array[ROLLOUTS_PER_ITERATION];

    double noiseProfile[NUM_CTRL] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};

    void initialise(mjData *_d_init);
    std::vector<m_ctrl> optimise(std::vector<m_ctrl> U_init);
    double rolloutTrajectory(mjData *d, std::vector<m_ctrl> U);
    std::vector<m_ctrl> generateAndEvaluateNoisyTrajectories(std::vector<m_ctrl> U_best, double oldBestCost, bool &betterTrajecFound, double &newBestCost);
    std::vector<m_ctrl> generateNoisyTrajectory(std::vector<m_ctrl> U_best);
    bool checkForConvergence(double oldCost, double newCost);
private:
};

#endif //MUJOCO_ACROBOT_CONTROL_STOMP_H
