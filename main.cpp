#include "Utility/MujocoController/MujocoUI.h"
#include "iLQR/iLQR_dataCentric.h"
#include "STOMP/STOMP.h"
#include "modelTranslator/modelTranslator.h"
#include "Utility/stdInclude/stdInclude.h"
#include "Utility/savingData/saveData.h"

#include "mujoco.h"

// Different operating modes for my code
#define RUN_ILQR                0       // RUN_ILQR - runs a simple iLQR optimisation for given task and renders on repeat
#define RUN_STOMP               0       // RUN_STOMP - runs a simple stomp optimiser for given task and renders on repeat
#define MPC_TESTING             0       // MPC_TESTING - Runs an MPC controller to achieve desired task and renders controls live
#define GENERATE_A_B            0       // GENERATE_A_B - runs initial trajectory and generates and saves A, B matrices over that trajectory for every timestep
#define ILQR_DATA_COLLECTION    0       // ILQR_DATA_COLLECTION - runs iLQR for many trajectories and saves useful data to csv file
#define MAKE_TESTING_DATA       0       // MAKE_TESTING_DATA - Creates a set number of valid starting and desired states for a certain task and saves them to a .csv file for later use.

#define READ_TASK_FROM_FILE     1

extern MujocoController *globalMujocoController;
extern mjModel* model;						// MuJoCo model
extern mjData* mdata;						// MuJoCo data
//mjData *opt_mdata;                        // Optimisation data
extern mjData* d_initMPC;

extern iLQR* optimiser;
extern STOMP* optimiser_stomp;
extern taskTranslator* modelTranslator;
saveData* dataSaver;

m_state X0;

extern std::vector<m_ctrl> initControls;
extern std::vector<m_ctrl> finalControls;
extern std::vector<m_ctrl> MPCControls;
extern std::vector<bool> grippersOpen;
extern mjData* d_init;
extern mjData* d_init_master;

void simpleTest();
void MPCControl(int taskRow);

int main() {
    // Create a new modelTranslator, initialise mujoco and then pass mujoco model to modelTranslator
    modelTranslator = new taskTranslator();
    dataSaver = new saveData();
    const char *fileName;
    int taskRow = 5;        // done testing with taskRow = 3

    //------------------------------------//
    m_point eulerAngle;
    eulerAngle << 0, 0, 0;

    m_quat quat = globalMujocoController->eul2Quat(eulerAngle);
    cout << "Quat: " << quat << endl;

    eulerAngle << PI, 0, 0;

    quat = globalMujocoController->eul2Quat(eulerAngle);
    cout << "Quat: " << quat << endl;

    eulerAngle << 0, PI, 0;

    quat = globalMujocoController->eul2Quat(eulerAngle);
    cout << "Quat: " << quat << endl;

    eulerAngle << 0, 0, PI;

    quat = globalMujocoController->eul2Quat(eulerAngle);
    cout << "Quat: " << quat << endl;

    //------------------------------------//

    // Acrobot model
    if(modelTranslator->taskNumber == 0){
        fileName = "franka_emika/Acrobot.xml";
    }
    // General franka_emika arm reaching model
    else if(modelTranslator->taskNumber == 1){
        fileName = "franka_emika/reaching.xml";
    }
    // Franka arm plus a cylinder to push along ground
    else if(modelTranslator->taskNumber == 2){
        fileName = "franka_emika/object_pushing.xml";
        //"/home/davidrussell/catkin_ws/src/realRobotExperiments_TrajOpt/Dynamic_Interpolation_iLQR/franka_emika/object_pushing.xml"
    }
    // Franka arm reaches through mild clutter to goal object
    else if(modelTranslator->taskNumber == 3){
        const char *fileName = "franka_emika/reaching_through_clutter.xml";
    }
    else{
        std::cout << "Valid task number not supplied, program, exiting" << std::endl;
    }

    initMujoco(0.004, fileName);

    // cant be done in modelTranslator constructor as model is not initialised yet
    modelTranslator->init(model);

//    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
//        initControls.push_back(m_ctrl());
//        finalControls.push_back(m_ctrl());
//    }

    if(RUN_ILQR){

        // Initialise optimiser - creates all the data objects
        optimiser = new iLQR(model, mdata, modelTranslator);

        // Setup task - sets up desired state and initial state
        X0 = modelTranslator->setupTask(d_init, false, taskRow);
        cout << "X desired: " << modelTranslator->X_desired << endl;

        // Set up number of steps per derivative
        optimiser->updateNumStepsPerDeriv(5);

        // Set up initial controls
        initControls = modelTranslator->initOptimisationControls(mdata, d_init, X0);

        // Optimise controls
        finalControls = optimiser->optimise(d_init, initControls, 10);

        render();
    }
    else if(RUN_STOMP){
        // Initialise optimiser
        optimiser_stomp = new STOMP(model, mdata, modelTranslator);

        X0 = modelTranslator->setupTask(d_init, false, taskRow);
        cout << "X desired: " << modelTranslator->X_desired << endl;

        initControls = modelTranslator->initOptimisationControls(mdata, d_init, X0);

        optimiser_stomp->initialise(d_init);
        finalControls = optimiser_stomp->optimise(initControls);

        render();
    }
    else if(MPC_TESTING){
        MPCControl(taskRow);
    }
    else if(GENERATE_A_B){
        optimiser = new iLQR(model, mdata, modelTranslator);

        int validTrajectories = 0;

        while(validTrajectories < 1){
            initControls.clear();
            optimiser->updateNumStepsPerDeriv(1);
            // updates X0, X_desired and d_init_test
            X0 = modelTranslator->setupTask(d_init, false, taskRow);
            cpMjData(model, optimiser->d_init, d_init);
            initControls = modelTranslator->initOptimisationControls(mdata, d_init, X0);
            optimiser->setInitControls(initControls, grippersOpen);

            optimiser->rollOutTrajectory();

            if(optimiser->trajecCollisionFree == true){
                optimiser->getDerivativesStatically();

                for(int j = 0; j < MUJ_STEPS_HORIZON_LENGTH; j++){
                    dataSaver->A_matrices.push_back(m_state_state());
                    dataSaver->A_matrices[(validTrajectories * MUJ_STEPS_HORIZON_LENGTH) + j] = optimiser->A[j].replicate(1, 1);

                    dataSaver->B_matrices.push_back(m_state_ctrl());
                    dataSaver->B_matrices[(validTrajectories * MUJ_STEPS_HORIZON_LENGTH) + j] = optimiser->B[j].replicate(1, 1);

                    dataSaver->savedStates.push_back(m_state());
                    dataSaver->savedStates[(validTrajectories * MUJ_STEPS_HORIZON_LENGTH) + j] = optimiser->X_old[j].replicate(1,1);

                    dataSaver->savedControls.push_back(m_ctrl());
                    dataSaver->savedControls[(validTrajectories * MUJ_STEPS_HORIZON_LENGTH) + j] = optimiser->U_new[j].replicate(1,1);
                }

                cout << "Trajectory " << validTrajectories << " completed" << endl;
                validTrajectories++;
            }
            else{
                cout << "bad initial trajectory, encountered collision, DISCARDING" << endl;
            }

        }
        dataSaver->saveMatricesToCSV();

    }
    else if(ILQR_DATA_COLLECTION){
        // Only need to be done once - initialise optimiser and make data for optimisation
        optimiser = new iLQR(model, mdata, modelTranslator);
        optimiser->makeDataForOptimisation();

        if(!DYNAMIC_LINEAR_DERIVS){
            for(int i = 0; i < NUM_TRAJECTORIES_DATA_COLLECTION; i++){

                // randomly generate start and goal state, reinitialise initial state data object
                initControls.clear();
                modelTranslator->X_desired = modelTranslator->setupTask(mdata, false, i);
                initControls = modelTranslator->initOptimisationControls(mdata, d_init, X0);

                std::vector<float> _initCosts;
                std::vector<float> _finalsCosts;
                std::vector<float> _iLQRTime;
                std::vector<float> _avgLinTime;
                std::vector<int>   _numIterations;

                // Run trajectory optimisation for the different steps between derivatives defined (1, 2, 5, 10, etc...)
                for(int j = 0; j < NUM_TESTS_EXTRAPOLATION; j++){
                    auto iLQRStart = high_resolution_clock::now();
                    optimiser->updateNumStepsPerDeriv(dataSaver->scalingLevels[j]);
                    optimiser->resetInitialStates(d_init, X0);
                    optimiser->setInitControls(initControls, grippersOpen);

                    finalControls = optimiser->optimise(d_init, initControls, 10);

                    auto iLQRStop = high_resolution_clock::now();
                    auto iLQRDur = duration_cast<microseconds>(iLQRStop - iLQRStart);

                    float milliiLQRTime = iLQRDur.count()/1000;
                    _iLQRTime.push_back(milliiLQRTime/1000);
                    _initCosts.push_back(optimiser->initCost);
                    _finalsCosts.push_back(optimiser->finalCost);
                    _avgLinTime.push_back(optimiser->avgLinTime);
                    _numIterations.push_back(optimiser->numIterations);

                }

                dataSaver->initCosts.push_back(_initCosts);
                dataSaver->finalsCosts.push_back(_finalsCosts);
                dataSaver->iLQRTime.push_back(_iLQRTime);
                dataSaver->avgLinTime.push_back(_avgLinTime);
                dataSaver->numIterations.push_back(_numIterations);

            }

            cout << "final costs: " << dataSaver->finalsCosts[1][0] << endl;
        }
        else{
            for(int i = 0; i < NUM_TRAJECTORIES_DATA_COLLECTION; i++) {

                // randomly generate start and goal state, reinitialise initial state data object
                initControls.clear();
                modelTranslator->X_desired = modelTranslator->setupTask(mdata, false, i);
                initControls = modelTranslator->initOptimisationControls(mdata, d_init, X0);

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

                finalControls = optimiser->optimise(d_init, initControls, 10);

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

                dataSaver->initCosts.push_back(_initCosts);
                dataSaver->finalsCosts.push_back(_finalsCosts);
                dataSaver->iLQRTime.push_back(_iLQRTime);
                dataSaver->avgLinTime.push_back(_avgLinTime);
                dataSaver->numIterations.push_back(_numIterations);
                dataSaver->avgNumEvaluations.push_back(_avgNumEvaluations);
                dataSaver->avgEvalsVariance.push_back(_variance);
            }

        }
        dataSaver->saveDataCollectionToCSV(modelTranslator->taskNumber);

    }
    else if(MAKE_TESTING_DATA){
        int numTestingData = 50;

        for(int i = 0; i < numTestingData; i++){

            dataSaver->testing_X0.push_back(m_state());
            dataSaver->testing_X_Desired.push_back(m_state());

            dataSaver->testing_X0[i] = modelTranslator->generateRandomStartState(mdata);
            dataSaver->testing_X_Desired[i] = modelTranslator->generateRandomGoalState(dataSaver->testing_X0[i], mdata);

        }

        dataSaver->saveTestingData(numTestingData, modelTranslator->names[modelTranslator->taskNumber]);
    }
    else{

        optimiser = new iLQR(model, mdata, modelTranslator);
        X0 = modelTranslator->setupTask(mdata, false, taskRow);
        cpMjData(model, d_init_master, mdata);
        cpMjData(model, d_init, d_init_master);

        simpleTest();
        optimiser->resetInitialStates(d_init, X0);

        render_simpleTest();
    }

    return 0;
}

void MPCControl(int taskRow){
    // Initialise optimiser - creates all the data objects
    optimiser = new iLQR(model, mdata, modelTranslator);
    X0 = modelTranslator->setupTask(d_init_master, false, taskRow);
    cout << "X desired: " << modelTranslator->X_desired << endl;
    optimiser->updateNumStepsPerDeriv(5);
    initControls = modelTranslator->initSetupControls(mdata, d_init, X0);
    finalControls = optimiser->optimise(d_init, initControls, 2);
    bool taskComplete = false;
    int currentControlCounter = 0;
    int visualCounter = 0;
    int overallTaskCounter = 0;
    int reInitialiseCounter = 0;
    bool movingToStart = true;

    cpMjData(model, mdata, d_init);
    cpMjData(model, d_initMPC, d_init);

    auto MPCStart = high_resolution_clock::now();

    while(!taskComplete){

        if(movingToStart){

        }
        else{

        }


        m_ctrl nextControl = finalControls.at(0);
        // Delete control we have applied
        finalControls.erase(finalControls.begin());
        // add control to back - replicate last control for now
        finalControls.push_back(finalControls.at(finalControls.size() - 1));

        // Store applied control in a std::vector for re-playability
        MPCControls.push_back(nextControl);
        modelTranslator->setControls(mdata, nextControl, false);
        modelTranslator->stepModel(mdata, 1);
        currentControlCounter++;
        overallTaskCounter++;
        reInitialiseCounter++;

        // check if problem is solved?
        if(modelTranslator->taskCompleted(mdata)){
            cout << "task completed" << endl;
            taskComplete = true;
        }

        // timeout of problem solution
        if(overallTaskCounter > 3000){
            cout << "task timeout" << endl;
            taskComplete = true;
        }

        // State we predicted we would be at at this point. TODO - always true at the moment as no noise in system
        m_state predictedState = modelTranslator->returnState(mdata);
        //Check states mismatched
        bool replanNeeded = false;
        if(modelTranslator->predictiveStateMismatch(mdata, predictedState)){
            replanNeeded = true;
        }

        if(currentControlCounter > 300){
            replanNeeded = true;
            currentControlCounter = 0;

        }

        if(replanNeeded){
            cpMjData(model, d_init, mdata);
            if(modelTranslator->newControlInitialisationNeeded(d_init, reInitialiseCounter)){
                cout << "re initialise needed" << endl;
                reInitialiseCounter = 0;
                initControls = modelTranslator->initOptimisationControls(mdata, d_init, X0);
                finalControls = optimiser->optimise(d_init, initControls, 2);
            }
            else{
                finalControls = optimiser->optimise(d_init, finalControls, 2);
            }
        }

        visualCounter++;
        if(visualCounter >= 20){
            renderOnce(mdata);
            visualCounter = 0;
        }
    }

    auto MPCStop = high_resolution_clock::now();
    auto MPCDuration = duration_cast<microseconds>(MPCStop - MPCStart);
    float trajecTime = MPCControls.size() * MUJOCO_DT;
    cout << "duration of MPC was: " << MPCDuration.count()/1000 << " ms. Trajec length of " << trajecTime << " s" << endl;

    renderMPCAfter();
}

void simpleTest(){
    std::vector<m_ctrl> initControlsSetup = modelTranslator->initSetupControls(mdata, d_init, X0);
    cpMjData(model, d_init, mdata);
    std::vector<m_ctrl> initControlsOptimise = modelTranslator->initOptimisationControls(mdata, d_init, X0);
    cout << "initControlSetup " << initControlsSetup.size() << endl;
    initControls.insert(initControls.end(), initControlsSetup.begin(), initControlsSetup.end());
    initControls.insert(initControls.end(), initControlsOptimise.begin(), initControlsOptimise.end());
    cout << "initControl " << initControls.size() << endl;
    cout << "test" << endl;
}

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
