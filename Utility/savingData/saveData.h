//
// Created by dave on 27/01/23.
//

#ifndef MUJOCO_ACROBOT_CONTROL_SAVEDATA_H
#define MUJOCO_ACROBOT_CONTROL_SAVEDATA_H

#define NUM_TRAJECTORIES_DATA_COLLECTION 50
#define NUM_TESTS_EXTRAPOLATION  6

#include "../stdInclude/stdInclude.h"
#include "../../modelTranslator/modelTranslator.h"
#include "../../iLQR/iLQR_dataCentric.h"

typedef Matrix<double, (2), 1> m_cube;

class saveData{
public:

    saveData();
    ofstream outputFile;
    std::string filename = "finalTrajectory.csv";

    ofstream outputMatrices;
    ofstream outputBMatrices;
    std::string matricesFileName = "matrices_A.csv";

    ofstream outputDataCollection;

    std::vector<m_state_state> A_matrices;
    std::vector<m_state_ctrl> B_matrices;
    std::vector<m_state> savedStates;
    std::vector<m_ctrl> savedControls;

    std::vector<m_state> testing_X0;
    std::vector<m_state> testing_X_Desired;

    std::vector<std::vector<float>> initCosts;
    std::vector<std::vector<float>> finalsCosts;
    std::vector<std::vector<float>> iLQRTime;
    std::vector<std::vector<float>> avgLinTime;
    std::vector<std::vector<int>>   numIterations;
    std::vector<std::vector<int>>   avgNumEvaluations;
    std::vector<std::vector<double>> avgEvalsVariance;
    std::vector<m_cube> cubeTermPos;
    std::string DataCollecNames[3] = {"Pendulum_Data.csv", "Reaching_Data.csv", "Pushing_Data.csv"};
    int scalingLevels[NUM_TESTS_EXTRAPOLATION] = {1, 2, 5, 10, 20, 50};

    // --------------------------------------------------------------------------------------------------------
    //                                      Utility writing to files
    // --------------------------------------------------------------------------------------------------------
    void saveMatricesToCSV();
    void saveDataCollectionToCSV(int taskNumber);
    void saveTestingData(int numTestingData, const std::string& name);
    void saveTrajecToCSV(iLQR* optimiser);
    void saveCylinderDiffToCSV(std::vector<double> cubeDiffs, std::string fileName);

};

#endif //MUJOCO_ACROBOT_CONTROL_SAVEDATA_H
