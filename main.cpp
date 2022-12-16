#include "Utility/MujocoController/MujocoUI.h"
#include "iLQR/iLQR_dataCentric.h"
#include "modelTranslator/modelTranslator.h"
#include "Utility/stdInclude/stdInclude.h"

#include "mujoco.h"

// Different operating modes for my code
#define RUN_ILQR                1       // RUN_ILQR - runs a simple iLQR optimisation for given task
#define GENERATE_A_B            0       // GENERATE_A_B - runs initial trajectory and generates and saves A, B matrices over that trajectory for every timestep
#define ILQR_DATA_COLLECTION    0       // ILQR_DATA_COLLECTION - runs iLQR for many trajectories and saves useful data to csv file
#define MAKE_TESTING_DATA       0       // MAKE_TESTING_DATA - Creates a set number of valid starting and desired states for a certain task and saves them to a .csv file for later use.

#define NUM_TRAJECTORIES_DATA_COLLECTION 50
#define NUM_TESTS_EXTRAPOLATION  6

#define READ_TASK_FROM_FILE     1

extern MujocoController *globalMujocoController;
extern mjModel* model;						// MuJoCo model
extern mjData* mdata;						// MuJoCo data

extern iLQR* optimiser;
taskTranslator* modelTranslator;

typedef Matrix<double, (2), 1> m_cube;

std::vector<m_state> X_dyn;
std::vector<m_state> X_lin;

int scalingLevels[NUM_TESTS_EXTRAPOLATION] = {1, 2, 5, 10, 20, 50};

std::vector<std::vector<float>> initCosts;
std::vector<std::vector<float>> finalsCosts;
std::vector<std::vector<float>> iLQRTime;
std::vector<std::vector<float>> avgLinTime;
std::vector<std::vector<int>>   numIterations;
std::vector<std::vector<int>>   avgNumEvaluations;
std::vector<std::vector<double>> avgEvalsVariance;
std::vector<m_cube> cubeTermPos;

std::vector<m_state_state> A_matrices;
std::vector<m_state_ctrl> B_matrices;
std::vector<m_state> savedStates;
std::vector<m_ctrl> savedControls;

std::vector<m_state> testing_X0;
std::vector<m_state> testing_X_Desired;

m_state X0;
m_state X_desired;

ofstream outputDiffDyn;
std::string diffDynFilename = "diffDyn.csv";
ofstream outputFile;
std::string filename = "finalTrajectory.csv";

ofstream outputMatrices;
ofstream outputBMatrices;
std::string matricesFileName = "matrices_A.csv";

ofstream outputDataCollection;

extern mjvCamera cam;                   // abstract camera
extern mjvScene scn;                    // abstract scene
extern mjvOption opt;			        // visualization options
extern mjrContext con;				    // custom GPU context
extern GLFWwindow *window;

extern std::vector<m_ctrl> initControls;
extern std::vector<bool> grippersOpen;
extern mjData* d_init;

std::string DataCollecNames[3] = {"Pendulum_Data.csv", "Reaching_Data.csv", "Pushing_Data.csv"};

void saveStates();
void saveTrajecToCSV();

void simpleTest();

void saveMatricesToCSV();
void saveStatesToCSV();
void saveDataCollectionToCSV();
void saveTestingData(int numTestingData, const std::string& name);

void readStartAndGoalFromFile(int requiredRow);

int main() {
    // Create a new modelTranslator, initialise mujoco and then pass mujoco model to modelTranslator
    modelTranslator = new taskTranslator();
    initMujoco(modelTranslator->taskNumber, 0.004);
    modelTranslator->init(model);

    if(RUN_ILQR){

        // Initialise optimiser
        optimiser = new iLQR(model, mdata, modelTranslator, globalMujocoController);
        optimiser->makeDataForOptimisation();

        // For pushing - screenshots are from trajectory 2.
        // 2 is a good example of decent initilisation + final trajec
        // 3 is a good example of a bad initialisation
        // ----- For reaching ------
        X0 = modelTranslator->setupTask(d_init, false, 2);
        cout << "X desired: " << X_desired << endl;



        for(int i = 0; i < 1; i++){
            initControls.clear();
            auto iLQRStart = high_resolution_clock::now();

            optimiser->updateNumStepsPerDeriv(5);

            initControls = modelTranslator->initControls(mdata, d_init, X0);
            optimiser->resetInitialStates(d_init, X0);
            optimiser->setInitControls(initControls, grippersOpen);

//            optimiser->rollOutTrajectory();
//            optimiser->getDerivativesStatically();
//            optimiser->copyDerivatives();
//
//            for(int j = 0; j < MUJ_STEPS_HORIZON_LENGTH; j++){
//                A_matrices.push_back(m_state_state());
//                A_matrices[j] = optimiser->f_x[j].replicate(1, 1);
//
//                B_matrices.push_back(m_state_ctrl());
//                B_matrices[j] = optimiser->f_u[j].replicate(1, 1);
//
//                savedStates.push_back(m_state());
//                savedStates[j] = optimiser->X_old[j].replicate(1,1);
//
//                savedControls.push_back(m_ctrl());
//                savedControls[j] = optimiser->U_new[j].replicate(1,1);
//            }


            optimiser->optimise();

//            for(int j = 0; j < MUJ_STEPS_HORIZON_LENGTH; j++){
//                A_matrices.push_back(m_state_state());
//                A_matrices[(MUJ_STEPS_HORIZON_LENGTH) + j] = optimiser->f_x[j].replicate(1, 1);
//
//                B_matrices.push_back(m_state_ctrl());
//                B_matrices[(MUJ_STEPS_HORIZON_LENGTH) + j] = optimiser->f_u[j].replicate(1, 1);
//
//                savedStates.push_back(m_state());
//                savedStates[MUJ_STEPS_HORIZON_LENGTH + j] = optimiser->X_old[j].replicate(1,1);
//
//                savedControls.push_back(m_ctrl());
//                savedControls[MUJ_STEPS_HORIZON_LENGTH + j] = optimiser->U_new[j].replicate(1,1);
//            }
//            saveMatricesToCSV();

            auto iLQRStop = high_resolution_clock::now();
            auto iLQRDur = duration_cast<microseconds>(iLQRStop - iLQRStart);

            float milliiLQRTime = iLQRDur.count()/1000;
//            iLQRTime.push_back(milliiLQRTime/1000);
//            finalsCosts.push_back(optimiser->finalCost);
//            avgLinTime.push_back(optimiser->avgLinTime);
//            numIterations.push_back(optimiser->numIterations);

        }

//        for(int i = 0; i < 1; i++){
//            cout << "----------------------------------------------------" << endl;
//
//            cout << "iLQR convergence time: " << iLQRTime[i] << endl;
//            cout << "Final cost: " << finalsCosts[i] << endl;
//            cout << "Num Iterations: " << numIterations[i] << endl;
//            cout << "Average time linearising: " << avgLinTime[i] << endl;
//
//            cout << "-----------------------------------------------------" << endl;
//        }

        render();
    }
    else if(GENERATE_A_B){
        optimiser = new iLQR(model, mdata, modelTranslator, globalMujocoController);
        optimiser->makeDataForOptimisation();

        int validTrajectories = 0;

        while(validTrajectories < 1000){
            initControls.clear();
            optimiser->updateNumStepsPerDeriv(1);
            // updates X0, X_desired and d_init_test
            X_desired = modelTranslator->setupTask(mdata, true, 0);
            optimiser->resetInitialStates(d_init, X0);
            initControls = modelTranslator->initControls(mdata, d_init, X0);
            optimiser->setInitControls(initControls, grippersOpen);

            optimiser->rollOutTrajectory();

            if(optimiser->trajecCollisionFree == true){
                optimiser->getDerivativesStatically();

                for(int j = 0; j < MUJ_STEPS_HORIZON_LENGTH; j++){
                    A_matrices.push_back(m_state_state());
                    A_matrices[(validTrajectories * MUJ_STEPS_HORIZON_LENGTH) + j] = optimiser->A[j].replicate(1, 1);

                    B_matrices.push_back(m_state_ctrl());
                    B_matrices[(validTrajectories * MUJ_STEPS_HORIZON_LENGTH) + j] = optimiser->B[j].replicate(1, 1);

                    savedStates.push_back(m_state());
                    savedStates[(validTrajectories * MUJ_STEPS_HORIZON_LENGTH) + j] = optimiser->X_old[j].replicate(1,1);

                    savedControls.push_back(m_ctrl());
                    savedControls[(validTrajectories * MUJ_STEPS_HORIZON_LENGTH) + j] = optimiser->U_new[j].replicate(1,1);
                }

                cout << "Trajectory " << validTrajectories << " completed" << endl;
                validTrajectories++;
            }
            else{
                cout << "bad initial trajectory, encountered collision, DISCARDING" << endl;
            }

        }
        saveMatricesToCSV();

    }
    else if(ILQR_DATA_COLLECTION){
        // Only need to be done once - initialise optimiser and make data for optimisation
        optimiser = new iLQR(model, mdata, modelTranslator, globalMujocoController);
        optimiser->makeDataForOptimisation();

        if(!DYNAMIC_LINEAR_DERIVS){
            for(int i = 0; i < NUM_TRAJECTORIES_DATA_COLLECTION; i++){

                // randomly generate start and goal state, reinitialise initial state data object
                initControls.clear();
                X_desired = modelTranslator->setupTask(mdata, false, i);
                initControls = modelTranslator->initControls(mdata, d_init, X0);

                std::vector<float> _initCosts;
                std::vector<float> _finalsCosts;
                std::vector<float> _iLQRTime;
                std::vector<float> _avgLinTime;
                std::vector<int>   _numIterations;

                // Run trajectory optimisation for the different steps between derivatives defined (1, 2, 5, 10, etc...)
                for(int j = 0; j < NUM_TESTS_EXTRAPOLATION; j++){
                    auto iLQRStart = high_resolution_clock::now();
                    optimiser->updateNumStepsPerDeriv(scalingLevels[j]);
                    optimiser->resetInitialStates(d_init, X0);
                    optimiser->setInitControls(initControls, grippersOpen);

                    optimiser->optimise();

                    auto iLQRStop = high_resolution_clock::now();
                    auto iLQRDur = duration_cast<microseconds>(iLQRStop - iLQRStart);

                    float milliiLQRTime = iLQRDur.count()/1000;
                    _iLQRTime.push_back(milliiLQRTime/1000);
                    _initCosts.push_back(optimiser->initCost);
                    _finalsCosts.push_back(optimiser->finalCost);
                    _avgLinTime.push_back(optimiser->avgLinTime);
                    _numIterations.push_back(optimiser->numIterations);

                }

                initCosts.push_back(_initCosts);
                finalsCosts.push_back(_finalsCosts);
                iLQRTime.push_back(_iLQRTime);
                avgLinTime.push_back(_avgLinTime);
                numIterations.push_back(_numIterations);

            }

            cout << "final costs: " << finalsCosts[1][0] << endl;
        }
        else{
            for(int i = 0; i < NUM_TRAJECTORIES_DATA_COLLECTION; i++) {

                // randomly generate start and goal state, reinitialise initial state data object
                initControls.clear();
                X_desired = modelTranslator->setupTask(mdata, false, i);
                initControls = modelTranslator->initControls(mdata, d_init, X0);

                std::vector<float> _initCosts;
                std::vector<float> _finalsCosts;
                std::vector<float> _iLQRTime;
                std::vector<float> _avgLinTime;
                std::vector<int> _numIterations;
                std::vector<int> _avgNumEvaluations;
                std::vector<double> _variance;

                auto iLQRStart = high_resolution_clock::now();
                optimiser->updateNumStepsPerDeriv(5);
                optimiser->resetInitialStates(d_init, X0);
                optimiser->setInitControls(initControls, grippersOpen);

                optimiser->optimise();

                auto iLQRStop = high_resolution_clock::now();
                auto iLQRDur = duration_cast<microseconds>(iLQRStop - iLQRStart);

                float milliiLQRTime = iLQRDur.count() / 1000;
                _iLQRTime.push_back(milliiLQRTime / 1000);
                _initCosts.push_back(optimiser->initCost);
                _finalsCosts.push_back(optimiser->finalCost);
                _avgLinTime.push_back(optimiser->avgLinTime);
                _numIterations.push_back(optimiser->numIterations);
                _avgNumEvaluations.push_back(optimiser->avgNumEvals);
                _variance.push_back(optimiser->avgVariance);

                initCosts.push_back(_initCosts);
                finalsCosts.push_back(_finalsCosts);
                iLQRTime.push_back(_iLQRTime);
                avgLinTime.push_back(_avgLinTime);
                numIterations.push_back(_numIterations);
                avgNumEvaluations.push_back(_avgNumEvaluations);
                avgEvalsVariance.push_back(_variance);
            }

        }
        saveDataCollectionToCSV();

    }
    else if(MAKE_TESTING_DATA){
        int numTestingData = 50;

        for(int i = 0; i < numTestingData; i++){

            testing_X0.push_back(m_state());
            testing_X_Desired.push_back(m_state());

            testing_X0[i] = modelTranslator->generateRandomStartState(mdata);
            testing_X_Desired[i] = modelTranslator->generateRandomGoalState(testing_X0[i], mdata);

        }

        saveTestingData(numTestingData, modelTranslator->names[modelTranslator->taskNumber]);
    }
    else{

        optimiser = new iLQR(model, mdata, modelTranslator, globalMujocoController);
        X0 = modelTranslator->setupTask(mdata, false, 2);
        cpMjData(model, d_init, mdata);

        simpleTest();
        optimiser->resetInitialStates(d_init, X0);

        render_simpleTest();
    }

    return 0;
}

#ifdef REACHING
void initControls(){

    m_ctrl lastControl;
    lastControl.setZero();
    int jerkLimit = 1;
    for(int i = 0; i <= MUJ_STEPS_HORIZON_LENGTH; i++){

        m_state X_diff;
        m_state X = modelTranslator->returnState(mdata);
        m_ctrl nextControl;
        X_diff = X_desired - X;
        int K[7] = {200, 200, 200, 200, 50, 50, 50};

        testInitControls.push_back(m_ctrl());
        grippersOpen.push_back(bool());
        grippersOpen[i] = false;

        nextControl(0) = X_diff(0) * K[0];
        nextControl(1) = X_diff(1) * K[1];
        nextControl(2) = X_diff(2) * K[2];
        nextControl(3) = X_diff(3) * K[3];
        nextControl(4) = X_diff(4) * K[4];
        nextControl(5) = X_diff(5) * K[5];
        nextControl(6) = X_diff(6) * K[6];


        for(int k = 0; k < NUM_CTRL; k++){

            if(nextControl(k) - lastControl(k) > jerkLimit){
                nextControl(k) = lastControl(k) + jerkLimit;
            }

            if(nextControl(k) - lastControl(k) < -jerkLimit){
                nextControl(k) = lastControl(k) - jerkLimit;
            }

            if(nextControl(k) > modelTranslator->torqueLims[k]) nextControl(k) = modelTranslator->torqueLims[k];
            if(nextControl(k) < -modelTranslator->torqueLims[k]) nextControl(k) = -modelTranslator->torqueLims[k];

            //nextControl(k) = X_diff(k) * K[i];
            testInitControls[i](k) = nextControl(k);
            mdata->ctrl[k] = testInitControls[i](k);

        }

        lastControl = nextControl.replicate(1,1);

//        cout << "x_diff[i]" << X_diff << endl;
//        cout << "next control: " << nextControl << endl;
//        cout << "testInitControls[i]" << testInitControls[i] << endl;

        for(int j = 0; j < 1; j++){
            mj_step(model, mdata);
        }
    }
}

#endif

#ifdef REACHING_CLUTTER
void initControls(){
    cpMjData(model, mdata, d_init_test);

    int grippersOpenIndex = 1000;
    const std::string endEffecName = "end_effector";
    //panda0_leftfinger
    const std::string EE_Name = "panda0_leftfinger";
    const std::string goalName = "goal";

    int pos_EE_id = mj_name2id(model, mjOBJ_SITE, endEffecName.c_str());
    int EE_id = mj_name2id(model, mjOBJ_BODY, EE_Name.c_str());
    int goal_id = mj_name2id(model, mjOBJ_BODY, goalName.c_str());

    cout << "EE id: " << EE_id << endl;
    cout << "pos_EE_id id: " << pos_EE_id << endl;

    m_point startPoint = modelTranslator->returnEE_point(mdata);
            //globalMujocoController->returnSitePoint(model, mdata, pos_EE_id);
    m_quat startQuat = globalMujocoController->returnBodyQuat(model, mdata, EE_id);
    float endPointX = X_desired(7) + 0.1;
    float endPointY = X_desired(8);

    endPointY += 0.3;

    cout << "start pose: " << startPoint << endl;
    cout << "start quart: " << startQuat << endl;

    float x_diff = endPointX - startPoint(0);
    float y_diff = endPointY - startPoint(1);

    m_point initPath[MUJ_STEPS_HORIZON_LENGTH];
    initPath[0](0) = startPoint(0);
    initPath[0](1) = startPoint(1);
    initPath[0](2) = startPoint(2);

    for (int i = 0; i < MUJ_STEPS_HORIZON_LENGTH - 1; i++) {
        initPath[i + 1](0) = initPath[i](0) + (x_diff / (MUJ_STEPS_HORIZON_LENGTH));
        initPath[i + 1](1) = initPath[i](1) + (y_diff / (MUJ_STEPS_HORIZON_LENGTH));
        initPath[i + 1](2) = initPath[i](2);
    }

    for (int i = 0; i <= MUJ_STEPS_HORIZON_LENGTH; i++) {

        m_point currentEEPoint = globalMujocoController->returnSitePoint(model, mdata, pos_EE_id);
        m_quat currentQuat = globalMujocoController->returnBodyQuat(model, mdata, EE_id);

        m_quat invQuat = globalMujocoController->invQuat(currentQuat);
        m_quat quatDiff = globalMujocoController->multQuat(startQuat, invQuat);

        m_point axisDiff = globalMujocoController->quat2Axis(quatDiff);

        m_pose differenceFromPath;
        float gains[6] = {1000, 1000, 1000, 80, 80, 80};
        for (int j = 0; j < 3; j++) {
            differenceFromPath(j) = initPath[i](j) - currentEEPoint(j);
            differenceFromPath(j + 3) = axisDiff(j);
        }

//        cout << "current path point: " << initPath[i] << endl;S
//        cout << "currentEE Pose: " << currentEEPose << endl;
//        cout << "diff from path: " << differenceFromPath << endl;

        m_pose desiredEEForce;

        for (int j = 0; j < 6; j++) {
            desiredEEForce(j) = differenceFromPath(j) * gains[j];
        }

        //cout << "desiredEEForce " << desiredEEForce << endl;

        MatrixXd Jac = globalMujocoController->calculateJacobian(model, mdata, EE_id);

        MatrixXd Jac_t = Jac.transpose();
        MatrixXd Jac_inv = Jac.completeOrthogonalDecomposition().pseudoInverse();

        m_ctrl desiredControls;

        MatrixXd jacobianControls = Jac_inv * desiredEEForce;

        testInitControls.push_back(m_ctrl());
        grippersOpen.push_back(bool());

        jacobianControls(6) = 0;


        for (int k = 0; k < NUM_CTRL; k++) {

            testInitControls[i](k) = jacobianControls(k) + mdata->qfrc_bias[k];

            mdata->ctrl[k] = testInitControls[i](k);
        }

        if(i < grippersOpenIndex){
            grippersOpen[i] = false;

        }
        else if(i > grippersOpenIndex and i < 2500){
            grippersOpen[i] = true;
        }
        else{
            grippersOpen[i] = false;
        }

        if(grippersOpen[i]){
            mdata->ctrl[7] = GRIPPERS_OPEN;
            mdata->ctrl[8] = GRIPPERS_OPEN;
        }
        else{
            mdata->ctrl[7] = GRIPPERS_CLOSED;
            mdata->ctrl[8] = GRIPPERS_CLOSED;
        }

        for (int j = 0; j < 1; j++) {
            mj_step(model, mdata);
        }
    }
}
#endif

void simpleTest(){
    initControls = modelTranslator->initControls(mdata, d_init, X0);
}

void saveTrajecToCSV(){

    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
        outputFile << i << ",";

        for(int j = 0; j < NUM_CTRL; j++){
            outputFile << optimiser->finalControls[i](j) << ",";
        }

        for(int j = 0; j < NUM_CTRL; j++){
            outputFile << optimiser->initControls[i](j) << ",";
        }

        for(int j = 0; j < DOF; j++){
            outputFile << optimiser->X_final[i](j) << ",";
        }

        for(int j = 0; j < DOF; j++){
            outputFile << optimiser->X_final[i](j+DOF) << ",";
        }
        outputFile << endl;
    }

    outputFile.close();
}

void saveStates(){

    cout << "X_dyn[end]: " << X_dyn[MUJ_STEPS_HORIZON_LENGTH] << endl;
    cout << "X_lin[end]: " << X_lin[MUJ_STEPS_HORIZON_LENGTH] << endl;

    cout << "X_dyn[0]: " << X_dyn[0] << endl;
    cout << "X_lin[0]: " << X_lin[0] << endl;


    outputDiffDyn.open(diffDynFilename);
    outputDiffDyn << "Joint 0 dyn" << "," << "Joint 0 lin" << "," << "Joint 0 diff" << "," << "Joint 1 dyn" << "," << "Joint 1 lin" << "," << "Joint 1 diff" << ",";
    outputDiffDyn << "Joint 2 dyn" << "," << "Joint 2 lin" << "," << "Joint 2 diff" << "," << "Joint 3 dyn" << "," << "Joint 3 lin" << "," << "Joint 3 diff" << ",";
    outputDiffDyn << "Joint 4 dyn" << "," << "Joint 4 lin" << "," << "Joint 4 diff" << "," << "Joint 5 dyn" << "," << "Joint 5 lin" << "," << "Joint 5 diff" << ",";
    outputDiffDyn << "Joint 6 dyn" << "," << "Joint 6 lin" << "," << "Joint 6 diff" << ",";
    outputDiffDyn << "Cube X dyn" << "," << "Cube X lin" << "," << "Cube X diff" << "," << "Cube Y dyn" << "," << "Cube Y lin" << "," << "Cube Y diff" << "," << "Cube rot dyn" << "," << "Cube rot lin" << "," << "Cube rot diff" << ",";
    outputDiffDyn << "Joint 0 vel dyn" << "," << "Joint 0 vel lin" << "," << "Joint 0 vel diff" << ",";
    outputDiffDyn << "Joint 1 vel dyn" << "," << "Joint 1 vel lin" << "," << "Joint 1 vel diff" << "," << "Joint 2 vel dyn" << "," << "Joint 2 vel lin" << "," << "Joint 2 vel diff" << ",";
    outputDiffDyn << "Joint 3 vel dyn" << "," << "Joint 3 vel lin" << "," << "Joint 3 vel diff" << "," << "Joint 4 vel dyn" << "," << "Joint 4 vel lin" << "," << "Joint 4 vel diff" << ",";
    outputDiffDyn << "Joint 5 vel dyn" << "," << "Joint 5 vel lin" << "," << "Joint 5 vel diff" << "," << "Joint 6 vel dyn" << "," << "Joint 6 vel lin" << "," << "Joint 6 vel diff" << ",";
    outputDiffDyn << "Cube X  vel dyn" << "," << "Cube X vel lin" << "," << "Cube X vel diff" << "," << "Cube Y vel dyn" << "," << "Cube Y vel lin" << "," << "Cube Y vel diff" << "," << "Cube rot vel dyn" << "," << "Cube rot vel lin" << "," << "Cube rot vel diff" << endl;

    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
        for(int j = 0; j < (2 * DOF); j++){
            float val;
            val = X_dyn[i](j);
            outputDiffDyn << val << ",";
            val = X_lin[i](j);
            outputDiffDyn << val << ",";
            val = X_lin[i](j) - X_dyn[i](j);
            outputDiffDyn << val << ",";
        }
        outputDiffDyn << endl;
    }

    outputDiffDyn.close();
}

void saveMatricesToCSV(){
    int size = A_matrices.size();
    cout << "size is" << size << endl;
    outputMatrices.open(matricesFileName);
    for(int i = 0; i < size; i++){
        for(int j = 0; j < (DOF); j++){
            for(int k = 0; k < (2 * DOF); k++){
                outputMatrices << A_matrices[i](j + DOF, k);
                if(k == (2 * DOF) - 1){
                    outputMatrices << endl;
                }
                else{
                    outputMatrices << ",";
                }
            }

        }
    }
    outputMatrices.close();

    outputBMatrices.open("matrices_B.csv");

    for(int i = 0; i < size; i++){
        for(int j = 0; j < (DOF); j++){
            for(int k = 0; k < NUM_CTRL; k++){
                outputBMatrices << B_matrices[i](j + DOF, k);
                if(k == NUM_CTRL - 1){
                    outputBMatrices << endl;
                }
                else{
                    outputBMatrices << ",";
                }
            }
        }
    }

    outputBMatrices.close();

    outputMatrices.open("savedStates.csv");
    for(int i = 0; i < size; i++){
        for(int j = 0; j < (DOF * 2); j++)
        {
            outputMatrices << savedStates[i](j);
            if(j == (2 * DOF) - 1){
                outputMatrices << endl;
            }
            else{
                outputMatrices << ",";
            }
        }
    }

    outputBMatrices.close();

    outputBMatrices.open("savedControls.csv");

    for(int i = 0; i < size; i++){
        for(int j = 0; j < NUM_CTRL; j++) {
            outputBMatrices << savedControls[i](j);

            if(j == NUM_CTRL - 1){
                outputBMatrices << endl;
            }
            else{
                outputBMatrices << ",";
            }
        }
    }

    outputBMatrices.close();
}

void saveDataCollectionToCSV(){

    std::string root;
    std::string filePath;

    if(COPYING_DERIVS){
        root = "Testing_Data/Copying/";
    }
    else if(LINEAR_INTERP_DERIVS){
        root = "Testing_Data/Linear_Interpolation/";
    }
    else if(DYNAMIC_LINEAR_DERIVS){
        root = "Testing_Data/Dynamic_Linear_Interpolation/";
    }

    filePath = root + DataCollecNames[modelTranslator->taskNumber];
    cout << "filepath: " << filePath << endl;

    outputDataCollection.open(filePath);

    if(!DYNAMIC_LINEAR_DERIVS){
        for(int i = 0; i < NUM_TESTS_EXTRAPOLATION; i++){
            outputDataCollection << "num Iterations" << ",";
        }
        for(int i = 0; i < NUM_TESTS_EXTRAPOLATION; i++){
            outputDataCollection << "optimisation time" << ",";
        }
        for(int i = 0; i < NUM_TESTS_EXTRAPOLATION; i++){
            outputDataCollection << "cost reduction" << ",";
        }
        for(int i = 0; i < NUM_TESTS_EXTRAPOLATION; i++){
            outputDataCollection << "Mean Time Linearising" << ",";
        }

        outputDataCollection << endl;

        for(int i = 0; i < 4; i++){
            for(int j = 0; j < NUM_TESTS_EXTRAPOLATION; j++){
                outputDataCollection << scalingLevels[j] << ",";
            }
        }
        outputDataCollection << endl;

        for(int i = 0; i < NUM_TRAJECTORIES_DATA_COLLECTION; i++){
            for(int j = 0; j < NUM_TESTS_EXTRAPOLATION; j++){
                outputDataCollection << numIterations[i][j] << ",";
            }

            for(int j = 0; j < NUM_TESTS_EXTRAPOLATION; j++){
                outputDataCollection << iLQRTime[i][j] << ",";
            }

            for(int j = 0; j < NUM_TESTS_EXTRAPOLATION; j++){
                float costReduction = 0.0f;
                costReduction = 1 - (finalsCosts[i][j]/initCosts[i][j]);
                outputDataCollection << costReduction << ",";
            }

            for(int j = 0; j < NUM_TESTS_EXTRAPOLATION; j++){
                outputDataCollection << avgLinTime[i][j] << ",";
            }

            outputDataCollection << endl;
        }
    }
    else{

        outputDataCollection << "num Iterations" << ",";
        outputDataCollection << "optimisation time" << ",";
        outputDataCollection << "cost reduction" << ",";
        outputDataCollection << "Mean Linearising Time" << ",";
        outputDataCollection << "Mean Number of Evals" << ",";
        outputDataCollection << "Mean Variance of EVals" << ",";
        outputDataCollection << endl;

        for(int i = 0; i < NUM_TRAJECTORIES_DATA_COLLECTION; i++){
            outputDataCollection << numIterations[i][0] << ",";

            outputDataCollection << iLQRTime[i][0] << ",";

            float costReduction = 0.0f;
            costReduction = 1 - (finalsCosts[i][0]/initCosts[i][0]);
            outputDataCollection << costReduction << ",";


            outputDataCollection << avgLinTime[i][0] << ",";
            outputDataCollection << avgNumEvaluations[i][0] << ",";

            outputDataCollection << avgEvalsVariance[i][0] << ",";


            outputDataCollection << endl;
        }
    }



    outputDataCollection.close();
}

void saveTestingData(int numTestingData, const std::string& name){
    outputDataCollection.open(name);

    for(int i = 0; i < numTestingData; i++){
        for(int j = 0; j < (2 * DOF); j++){
            outputDataCollection << testing_X0[i](j) << ",";
        }
        for(int j = 0; j < (2 * DOF); j++){
            outputDataCollection << testing_X_Desired[i](j) << ",";
        }
        outputDataCollection << endl;
    }

    outputDataCollection.close();
}
