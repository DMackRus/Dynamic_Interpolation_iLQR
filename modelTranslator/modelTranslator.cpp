//
// Created by david on 03/05/22.
//

#include "modelTranslator.h"

extern MujocoController *globalMujocoController;

frankaModel::frankaModel(){

    for(int i = 0; i < 7; i++) {
        stateNames.push_back(std::string());
    }

#ifdef PENDULUM_TASK
    taskNumber = 0;

    stateNames[0] = "upper_arm";
    stateNames[1] = "lower_arm";

    armControlCosts = 0.05;
    double armStateCosts = 2;
    double armVelCosts = 0;

    for(int i = 0; i < NUM_CTRL; i++){
        R.diagonal()[i] = armControlCosts;
        J.diagonal()[i] = 0;
        stateIndexToStateName[i] = i;
        stateIndexToFreeJntIndex[i] = 0;
    }

    for(int i = 0; i < NUM_CTRL; i++){
        Q.diagonal()[i] = armStateCosts;
        Q.diagonal()[i + DOF] = armVelCosts;
    }

#endif

#ifdef REACHING_TASK
    taskNumber = 1;

    stateNames[0] = "panda0_link1";
    stateNames[1] = "panda0_link2";
    stateNames[2] = "panda0_link3";
    stateNames[3] = "panda0_link4";
    stateNames[4] = "panda0_link5";
    stateNames[5] = "panda0_link6";
    stateNames[6] = "panda0_link7";

    armControlCosts = 0;
    double armStateCosts = 1;
    double armVelCosts = 2;

    for(int i = 0; i < NUM_CTRL; i++){
        R.diagonal()[i] = armControlCosts;
        J.diagonal()[i] = 0;
        stateIndexToStateName[i] = i;
        stateIndexToFreeJntIndex[i] = 0;
    }

    for(int i = 0; i < NUM_CTRL; i++){
        Q.diagonal()[i] = armStateCosts;
        Q.diagonal()[i + DOF] = armVelCosts;
    }

#endif

#ifdef OBJECT_PUSHING_TASK
    taskNumber = 2;
    stateNames.push_back(std::string());
    stateNames[0] = "panda0_link1";
    stateNames[1] = "panda0_link2";
    stateNames[2] = "panda0_link3";
    stateNames[3] = "panda0_link4";
    stateNames[4] = "panda0_link5";
    stateNames[5] = "panda0_link6";
    stateNames[6] = "panda0_link7";
    stateNames[7] = "goal";

    armControlCosts = 0;
    double armStateCosts = 0;
    double armVelCosts = 0;

    for(int i = 0; i < NUM_CTRL; i++){
        R.diagonal()[i] = armControlCosts;
        J.diagonal()[i] = 0.0007;
        stateIndexToStateName[i] = i;
        stateIndexToFreeJntIndex[i] = 0;
    }

    for(int i = 0; i < NUM_CTRL; i++){
        Q.diagonal()[i] = armStateCosts;
        Q.diagonal()[i + DOF] = armVelCosts;
    }

    Q.diagonal()[7] = cubeXPosCost;
    Q.diagonal()[8] = cubeYPosCost;
    stateIndexToStateName[7] = 7;
    stateIndexToStateName[8] = 7;
    stateIndexToFreeJntIndex[7] = 0;
    stateIndexToFreeJntIndex[8] = 1;

    Q.diagonal()[7 + DOF] = cubeVelCosts;
    Q.diagonal()[8 + DOF] = cubeVelCosts;


#endif

#ifdef REACHING_CLUTTER_TASK
    taskNumber = 3;
    stateNames.push_back(std::string());
    stateNames.push_back(std::string());
    stateNames.push_back(std::string());
    stateNames.push_back(std::string());
    stateNames.push_back(std::string());
    stateNames[0] = "panda0_link1";
    stateNames[1] = "panda0_link2";
    stateNames[2] = "panda0_link3";
    stateNames[3] = "panda0_link4";
    stateNames[4] = "panda0_link5";
    stateNames[5] = "panda0_link6";
    stateNames[6] = "panda0_link7";
    stateNames[7] = "goal";
    stateNames[8] = "obstacle_1";
    stateNames[9] = "obstacle_2";
    stateNames[10] = "obstacle_3";
    stateNames[11] = "obstacle_4";

    armControlCosts = 0;
    double armStateCosts = 0;
    double armVelCosts = 0;

    for(int i = 0; i < NUM_CTRL; i++){
        R.diagonal()[i] = armControlCosts;
        J.diagonal()[i] = 0;
        stateIndexToStateName[i] = i;
        stateIndexToFreeJntIndex[i] = 0;
    }

    for(int i = 0; i < NUM_CTRL; i++){
        Q.diagonal()[i] = armStateCosts;
        Q.diagonal()[i + DOF] = armVelCosts;
    }

    for(int i = NUM_CTRL; i < 2 * DOF; i++){
        Q.diagonal()[i] = 0;
    }

    stateIndexToStateName[7] = 7;
    stateIndexToStateName[8] = 7;
    stateIndexToStateName[9] = 8;
    stateIndexToStateName[10] = 8;
    stateIndexToStateName[11] = 9;
    stateIndexToStateName[12] = 9;
    stateIndexToStateName[13] = 10;
    stateIndexToStateName[14] = 10;
    stateIndexToStateName[15] = 11;
    stateIndexToStateName[16] = 11;

    // 0 = x, 1 = y, 2 = z
    stateIndexToFreeJntIndex[7] = 0;
    stateIndexToFreeJntIndex[8] = 1;
    stateIndexToFreeJntIndex[9] = 0;
    stateIndexToFreeJntIndex[10] = 1;
    stateIndexToFreeJntIndex[11] = 0;
    stateIndexToFreeJntIndex[12] = 1;
    stateIndexToFreeJntIndex[13] = 0;
    stateIndexToFreeJntIndex[14] = 1;
    stateIndexToFreeJntIndex[15] = 0;
    stateIndexToFreeJntIndex[16] = 1;

#endif

    cout << "R: " << R.diagonal() << endl;
    cout << "Q: " << Q.diagonal() << endl;

}

void frankaModel::init(mjModel *m){
    model = m;
}

void frankaModel::setDesiredState(m_state _desiredState){
    X_desired = _desiredState.replicate(1, 1);
}

#ifdef DOUBLE_PENDULUM
// Given a set of mujoco data, what is the cost of its state and controls
double frankaModel::costFunction(mjData *d, int controlNum, int totalControls, m_ctrl lastControl, bool firstControl){
    double stateCost = 0;
    m_state X = returnState(d);
    m_ctrl U = returnControls(d);
    m_state X_diff;

    VectorXd temp(1);

    // actual - desired
    X_diff = X - X_desired;

    temp = ((X_diff.transpose() * Q * X_diff)) + (U.transpose() * R * U);

    stateCost = temp(0);

    return stateCost;

}

// Given a set of mujoco data, what are its cost derivates with respect to state and control
void frankaModel::costDerivatives(mjData *d, Ref<m_state> l_x, Ref<m_state_state> l_xx, Ref<m_ctrl> l_u, Ref<m_ctrl_ctrl> l_uu, int controlNum, int totalControls, m_ctrl lastU){
    m_state X_diff;
    m_state X;
    m_ctrl U;
    m_ctrl U_diff;

    X = returnState(d);
    U = returnControls(d);
    U_diff = U - lastU;

    // actual - desired
    X_diff = X - X_desired;

    l_x = 2 *  Q * X_diff;
    l_xx = 2 * Q;

    l_u = (2 * R * U) + (2 * J * U_diff);
    l_uu = (2 * R) + (2 * J);
}

// set the state of a mujoco data object as per this model
void frankaModel::setState(mjData *d, m_state X){
    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str() );
        globalMujocoController->set_qPosVal(model, d, bodyId, false, 0, X(i));
        globalMujocoController->set_qVelVal(model, d, bodyId, false, 0, X(i + DOF));
    }
}

// Return the state of a mujoco data model
m_state frankaModel::returnState(mjData *d){
    m_state state;

    // Firstly set all the required franka panda arm joints
    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        state(i) = globalMujocoController->return_qPosVal(model, d, bodyId, false, 0);
        state(i + DOF) = globalMujocoController->return_qVelVal(model, d, bodyId, false, 0);
    }

    return state;
}

m_dof frankaModel::returnVelocities(mjData *d){
    m_dof velocities;

    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        velocities(i) = globalMujocoController->return_qVelVal(model, d, bodyId, false, 0);
    }

    return velocities;
}

m_dof frankaModel::returnAccelerations(mjData *d){
    m_dof accelerations;

    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        accelerations(i) = globalMujocoController->return_qAccVal(model, d, bodyId, false, 0);
    }

    return accelerations;
}

m_state frankaModel::generateRandomStartState(mjData *d){
    m_state randState;

    float arm1Pos = randFloat(-2, 2);
    float arm2Pos = randFloat(-2, 2);

    randState << arm1Pos, arm2Pos, 0, 0;


    return randState;
}

m_state frankaModel::generateRandomGoalState( m_state startState, mjData *d){
    m_state randGoalState;

    float randNum = randFloat(0, 1);
    if(randNum > 0.5){
        // in stable down position
        randGoalState << 3.14, 0, 0, 0;
    }
    else{
        // unstable up position
        randGoalState << 0, 0, 0, 0;
    }


    return randGoalState;
}

#endif

#ifdef REACHING
// Given a set of mujoco data, what is the cost of its state and controls
double frankaModel::costFunction(mjData *d, int controlNum, int totalControls, m_ctrl lastControl, bool firstControl){
    double stateCost = 0;
    m_state X = returnState(d);
    m_ctrl U = returnControls(d);
    m_state X_diff;

    VectorXd temp(1);

    // actual - desired
    X_diff = X - X_desired;

    temp = ((X_diff.transpose() * Q * X_diff)) + (U.transpose() * R * U);

    stateCost = temp(0);

    return stateCost;

}

// Given a set of mujoco data, what are its cost derivates with respect to state and control
void frankaModel::costDerivatives(mjData *d, Ref<m_state> l_x, Ref<m_state_state> l_xx, Ref<m_ctrl> l_u, Ref<m_ctrl_ctrl> l_uu, int controlNum, int totalControls, m_ctrl lastU){
    m_state X_diff;
    m_state X;
    m_ctrl U;
    m_ctrl U_diff;

    X = returnState(d);
    U = returnControls(d);
    U_diff = U - lastU;

    // actual - desired
    X_diff = X - X_desired;

    l_x = 2 *  Q * X_diff;
    l_xx = 2 * Q;

    l_u = (2 * R * U) + (2 * J * U_diff);
    l_uu = (2 * R) + (2 * J);
}

void frankaModel::costDerivsControlsAnalytical(mjData *d, Ref<m_ctrl> l_u, Ref<m_ctrl_ctrl> l_uu, m_ctrl lastControl){
    m_ctrl U;
    m_ctrl U_diff;

    U = returnControls(d);
    U_diff = U - lastControl;

    l_u = (2 * R * U) + (2 * J * U_diff);
    l_uu = (2 * R) + (2 * J);
}

// set the state of a mujoco data object as per this model
void frankaModel::setState(mjData *d, m_state X){
    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str() );
        globalMujocoController->set_qPosVal(model, d, bodyId, false, 0, X(i));
        globalMujocoController->set_qVelVal(model, d, bodyId, false, 0, X(i + DOF));
    }
}

// Return the state of a mujoco data model
m_state frankaModel::returnState(mjData *d){
    m_state state;

    // Firstly set all the required franka panda arm joints
    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        state(i) = globalMujocoController->return_qPosVal(model, d, bodyId, false, 0);
        state(i + DOF) = globalMujocoController->return_qVelVal(model, d, bodyId, false, 0);
    }

    return state;
}

m_dof frankaModel::returnVelocities(mjData *d){
    m_dof velocities;

    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        velocities(i) = globalMujocoController->return_qVelVal(model, d, bodyId, false, 0);
    }

    return velocities;
}

m_dof frankaModel::returnAccelerations(mjData *d){
    m_dof accelerations;

    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        accelerations(i) = globalMujocoController->return_qAccVal(model, d, bodyId, false, 0);
    }

    return accelerations;
}

m_state frankaModel::generateRandomStartState(mjData *d){
    m_state randState;
    bool collisionFreeState = false;
    while(!collisionFreeState){
        for(int i = 0; i < 7; i++){
            float randNum = randFloat(jointLimsMin[i]/2, jointLimsMax[i]/2);
            randState(i) = randNum;
            randState(i + NUM_CTRL) = 0.0f;
        }
        if(!isStateInCollision(d, randState)){
            collisionFreeState = true;
        }
    }

//    randState << 1.19, 0.458, 0.653, -1.82, 0, 0, 0,
//                0, 0, 0, 0, 0, 0, 0;

    return randState;
}

m_state frankaModel::generateRandomGoalState(m_state startState, mjData *d){
    m_state randState;
    bool collisionFreeState = false;
    float midPoints[NUM_CTRL] = {0, 0, 0, -1.57, 0, 0.26, 0};
    while(!collisionFreeState){
        for(int i = 0; i < 7; i++){
            float randNum = randFloat(0.5, 1.5);

            // add on random stochastic variable
            if(startState(i) < midPoints[i]){
                randState(i) = startState(i) + randNum;
            }
                // subtract
            else{
                randState(i) = startState(i) - randNum;
            }

            //randState(i) = randNum;
            randState(i + NUM_CTRL) = 0.0f;
        }

        if(!isStateInCollision(d, randState)){
            collisionFreeState = true;
        }
    }

//    randState << -1.48, -0.513, -1.19, -1.49, 0, 0.684, 0,
//            0, 0, 0, 0, 0, 0, 0;
    return randState;
}

#endif

#ifdef OBJECT_PUSHING
double frankaModel::costFunction(mjData *d, int controlNum, int totalControls, m_ctrl lastControl, bool firstControl){
    double stateCost = 0;
    m_state X = returnState(d);
    m_ctrl U = returnControls(d);
    m_state X_diff;

    VectorXd temp(1);

    // actual - desired
    X_diff = X - X_desired;
    double percentageDone = (double)controlNum / (double)totalControls;
    double terminalScalar;
    if(percentageDone < 0.6){
        terminalScalar = 0;
    }
    else{
        terminalScalar = (terminalConstant * pow(percentageDone, 2)) + 1;
    }

    DiagonalMatrix<double, 2 * DOF> Q_scaled;
    for(int i = 0; i < DOF; i++){
        Q_scaled.diagonal()[i] = terminalScalar * Q.diagonal()[i];
        Q_scaled.diagonal()[i + DOF] = Q.diagonal()[i + DOF];
    }

    temp = ((X_diff.transpose() * Q_scaled * X_diff)) + (U.transpose() * R * U);

    stateCost = temp(0);

    return stateCost;

}

// Given a set of mujoco data, what are its cost derivates with respect to state and control
void frankaModel::costDerivatives(mjData *d, Ref<m_state> l_x, Ref<m_state_state> l_xx, Ref<m_ctrl> l_u, Ref<m_ctrl_ctrl> l_uu, int controlNum, int totalControls, m_ctrl lastU){
    m_state X_diff;
    m_state X;
    m_ctrl U;
    m_ctrl U_diff;

    X = returnState(d);
    U = returnControls(d);
    U_diff = U - lastU;

    // actual - desired
    X_diff = X - X_desired;
    double percentageDone = (double)controlNum / (double)totalControls;
    double terminalScalar;
    if(percentageDone < 0.7){
        terminalScalar = 0;
    }
    else{
        terminalScalar = (terminalConstant * pow(percentageDone, 2)) + 1;
    }

    DiagonalMatrix<double, 2 * DOF> Q_scaled;
    for(int i = 0; i < DOF; i++){
        Q_scaled.diagonal()[i] = terminalScalar * Q.diagonal()[i];
        Q_scaled.diagonal()[i + DOF] = Q.diagonal()[i + DOF];
    }

    l_x = 2 *  Q_scaled * X_diff;
    l_xx = 2 * Q_scaled;

    l_u = (2 * R * U) + (2 * J * U_diff);
    l_uu = (2 * R) + (2 * J);
}

// set the state of a mujoco data object as per this model
void frankaModel::setState(mjData *d, m_state X){

    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        globalMujocoController->set_qPosVal(model, d, bodyId, false, 0, X(i));
        globalMujocoController->set_qVelVal(model, d, bodyId, false, 0, X(i + DOF));
    }

    // Get mujoco Id's for items in the scene
    int goalBoxId = mj_name2id(model, mjOBJ_BODY, stateNames[7].c_str());

    // set item positions
    globalMujocoController->set_qPosVal(model, d, goalBoxId, true, 0, X(7));
    globalMujocoController->set_qPosVal(model, d, goalBoxId, true, 1, X(8));

    // set item velocities
    globalMujocoController->set_qVelVal(model, d, goalBoxId, true, 0, X(7 + DOF));
    globalMujocoController->set_qVelVal(model, d, goalBoxId, true, 1, X(8 + DOF));

}

// Return the state of a mujoco data model
m_state frankaModel::returnState(mjData *d){
    m_state state;

    // Firstly set all the required franka panda arm joints
    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        state(i) = globalMujocoController->return_qPosVal(model, d, bodyId, false, 0);
        state(i + DOF) = globalMujocoController->return_qVelVal(model, d, bodyId, false, 0);
    }

    // Get Mujoco Id's for items in the scene
    int goalBoxId = mj_name2id(model, mjOBJ_BODY, stateNames[7].c_str());

    state(7) = globalMujocoController->return_qPosVal(model, d, goalBoxId, true, 0);
    state(8) = globalMujocoController->return_qPosVal(model, d, goalBoxId, true, 1);

    state(7 + DOF) = globalMujocoController->return_qVelVal(model, d, goalBoxId, true, 0);
    state(8 + DOF) = globalMujocoController->return_qVelVal(model, d, goalBoxId, true, 1);

    return state;
}

m_dof frankaModel::returnVelocities(mjData *d){
    m_dof velocities;

    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        velocities(i) = globalMujocoController->return_qVelVal(model, d, bodyId, false, 0);
    }

    int goalBoxId = mj_name2id(model, mjOBJ_BODY, stateNames[7].c_str());

    velocities(NUM_CTRL) = globalMujocoController->return_qVelVal(model, d, goalBoxId, true, 0);
    velocities(NUM_CTRL + 1) = globalMujocoController->return_qVelVal(model, d, goalBoxId, true, 1);

    return velocities;
}

m_dof frankaModel::returnAccelerations(mjData *d){
    m_dof accelerations;

    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        accelerations(i) = globalMujocoController->return_qAccVal(model, d, bodyId, false, 0);
    }

    int goalBoxId = mj_name2id(model, mjOBJ_BODY, stateNames[7].c_str());

    accelerations(NUM_CTRL) = globalMujocoController->return_qAccVal(model, d, goalBoxId, true, 0);
    accelerations(NUM_CTRL + 1) = globalMujocoController->return_qAccVal(model, d, goalBoxId, true, 1);

    return accelerations;
}

m_state frankaModel::generateRandomStartState(mjData *d){
    m_state randState;

    float cubeX = randFloat(0.45, 0.55);
    float cubeY = randFloat(-0.1, 0.1);

//    cubeX = 0.5;
//    cubeY = 0;
    randState <<   0, -0.183, 0, -3.1, 0, 1.34, 0,
            cubeX, cubeY,
            0, 0, 0, 0, 0, 0, 0,
            0, 0;

    return randState;
}

m_state frankaModel::generateRandomGoalState(m_state startState, mjData *d){
    m_state randState;

    float desiredCubeX = randFloat(0.6, 0.8);
    float desiredCubeY = randFloat(-0.2, 0.2);

//    desiredCubeX = 0.7;
//    desiredCubeY = 0.1;
    randState << 0, 0, 0, 0, 0, 0, 0,
            desiredCubeX, desiredCubeY,
            0, 0, 0, 0, 0, 0, 0,
            0, 0;

    int visualGoalId = mj_name2id(model, mjOBJ_BODY, "display_goal");

    m_pose goalPose;
    goalPose.setZero();
    goalPose(0) = desiredCubeX;
    goalPose(1) = desiredCubeY;

    globalMujocoController->setBodyPose(model, d, visualGoalId, goalPose);

    return randState;
}

#endif

#ifdef REACHING_CLUTTER
double frankaModel::costFunction(mjData *d, int controlNum, int totalControls, m_ctrl lastControl, bool firstControl){
    double stateCost = 0;
    m_state X = returnState(d);
    m_ctrl U = returnControls(d);
    m_state X_diff;

    VectorXd temp(1);

    // actual - desired
    X_diff = X - X_desired;
    double percentageDone = (double)controlNum / (double)totalControls;
    double terminalScalar = 1;
//    if(percentageDone < 0.5){
//        terminalScalar = 0;
//    }
//    else{
//        terminalScalar = (terminalConstant * pow(percentageDone, 2)) + 1;
//    }
//
//    DiagonalMatrix<double, (2 * DOF) + 3> Q_scaled;
//    Q_scaled = Q;
//    for(int i = 0; i < 10; i++){
//        Q_scaled.diagonal()[i] = terminalScalar * Q_scaled.diagonal()[i];
//    }


    double distEEToGoal = EEDistToGoal(d, 7);
    cout << "dist to goal" << distEEToGoal << endl;

    temp = ((X_diff.transpose() * Q * X_diff)) + (U.transpose() * R * U);

    stateCost = temp(0) + distEEToGoal;

    return stateCost;

}

double frankaModel::EEDistToGoal(mjData *d, int goalObjectId){
    double dist;

    int goalObject_id = mj_name2id(model, mjOBJ_BODY, stateNames[goalObjectId].c_str());

    m_point EE_point = returnEE_point(d);
    m_pose goalObjectPose = globalMujocoController->returnBodyPose(model, d, goalObject_id);

//    double x_dist = EE_point(0) - goalObjectPose(0);
//    double y_dist = EE_point(1) - goalObjectPose(1);
    double x_dist = EE_point(0) - 0.8;
    double y_dist = EE_point(1) - 0;
    double z_dist = EE_point(1) - 0.3;

    dist = sqrt(pow(x_dist, 2) + pow(y_dist, 2) + pow(z_dist, 2));

    return dist;
}

m_point frankaModel::returnEE_point(mjData *d){
    m_point EE_point;
    const std::string EE = "end_effector";
    int EE_id = mj_name2id(model, mjOBJ_SITE, EE.c_str());

    EE_point = globalMujocoController->returnSitePoint(model, d, EE_id);
    cout << "EE_point" << EE_point << endl;

    return EE_point;
}

void frankaModel::costDerivsControlsAnalytical(mjData *d, Ref<m_ctrl> l_u, Ref<m_ctrl_ctrl> l_uu, m_ctrl lastControl){
    m_ctrl U = returnControls(d);
    m_ctrl U_diff = U - lastControl;

    l_u = (2 * R * U) + (2 * J * U_diff);
    l_uu = (2 * R) + (2 * J);
}

// Given a set of mujoco data, what are its cost derivates with respect to state and control
void frankaModel::costDerivatives(mjData *d, Ref<m_state> l_x, Ref<m_state_state> l_xx, Ref<m_ctrl> l_u, Ref<m_ctrl_ctrl> l_uu, int controlNum, int totalControls, m_ctrl lastU){
    m_state X_diff;
    m_state X;
    m_ctrl U;
    m_ctrl U_diff;

    X = returnState(d);
    U = returnControls(d);
    U_diff = U - lastU;

    // actual - desired
    X_diff = X - X_desired;
    double percentageDone = (double)controlNum / (double)totalControls;
    double terminalScalar = 1;
//    if(percentageDone < 0.5){
//        terminalScalar = 0;
//    }
//    else{
//        terminalScalar = (terminalConstant * pow(percentageDone, 2)) + 1;
//    }
//
//    DiagonalMatrix<double, 2 * DOF> Q_scaled;
//    for(int i = 0; i < DOF; i++){
//        Q_scaled.diagonal()[i] = terminalScalar * Q.diagonal()[i];
//        Q_scaled.diagonal()[i + DOF] = Q.diagonal()[i + DOF];
//    }

    //cout << "state: " << endl << X << endl;

    // End effector x pos
    double ex = X(2 * DOF);
    double ey = X((2 * DOF) + 1);
    double ez = X((2 * DOF) + 2);

    //cout << "ex: " << ex << endl;
    //cout << "ey: " << ey << endl;
    //cout << "ez: " << ez << endl;

    double gx = X(7);
    double gy = X(8);


    //cout << "gx: " << gx << endl;
    //cout << "gy: " << gy << endl;

    // cost with respect to endeffector x
    double l_ex = 2 * (ex - gx);
    double l_ey = 2 * (ey - gy);
    double l_ez = 2 * (ez - 0.2);
    double l_gx = -2 * (ex - gx);
    double l_gy = -2 * (ey - gy);

    l_x = 2 *  Q * X_diff;
    l_x((2 * DOF)) = l_ex;
    l_x((2 * DOF) + 1) = l_ey;
    l_x((2 * DOF) + 2) = l_ez;
    l_x(7) = l_gx;
    l_x(8) = l_gy;

    l_xx(7, 7) = 2;
    l_xx(8, 8) = 2;
    l_xx((2 * DOF), (2 * DOF)) = 2;
    l_xx((2 * DOF) + 1, (2 * DOF) + 1) = 2;
    l_xx((2 * DOF) + 2, (2 * DOF) + 2) = 2;

    l_xx(7, (2 * DOF)) = -2;
    l_xx(8, (2 * DOF) + 1) = -2;

    l_xx((2 * DOF), 7) = -2;
    l_xx((2 * DOF) + 1, 8) = -2;

    //cout << "l_x " << l_x << endl;

    l_xx = l_x * l_x.transpose();

    //cout << "l_xx " << l_xx << endl;

    //l_xx = 2 * Q;

    l_u = (2 * R * U) + (2 * J * U_diff);
    l_uu = (2 * R) + (2 * J);
}

// set the state of a mujoco data object as per this model
void frankaModel::setState(mjData *d, m_state X){

    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        globalMujocoController->set_qPosVal(model, d, bodyId, false, 0, X(i));
        globalMujocoController->set_qVelVal(model, d, bodyId, false, 0, X(i + DOF));
    }

    // Get mujoco Id's for items in the scene
    int goalBoxId = mj_name2id(model, mjOBJ_BODY, stateNames[7].c_str());
    int obstacleIds[4];

    // set goal object position
    globalMujocoController->set_qPosVal(model, d, goalBoxId, true, 0, X(7));
    globalMujocoController->set_qPosVal(model, d, goalBoxId, true, 1, X(8));

    // set goal object velocity
    globalMujocoController->set_qVelVal(model, d, goalBoxId, true, 0, X(7 + DOF));
    globalMujocoController->set_qVelVal(model, d, goalBoxId, true, 1, X(8 + DOF));

    // number of obstacles = 4
    for(int i = 0; i < 4; i++){
        obstacleIds[i] = mj_name2id(model, mjOBJ_BODY, stateNames[8 + i].c_str());
        // Set obstacle position
        globalMujocoController->set_qPosVal(model, d, obstacleIds[i], true, 0, X(9 + (2 * i)));
        globalMujocoController->set_qPosVal(model, d, obstacleIds[i], true, 1, X(10 + (2 * i)));

        // Set obstacle velocity
        globalMujocoController->set_qVelVal(model, d, obstacleIds[i], true, 0, X(9 + (2 * i) + DOF));
        globalMujocoController->set_qVelVal(model, d, obstacleIds[i], true, 1, X(10 + (2 * i) + DOF));
    }
}

// Return the state of a mujoco data model
m_state frankaModel::returnState(mjData *d){
    m_state state;

    // Firstly set all the required franka panda arm joints
    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        state(i) = globalMujocoController->return_qPosVal(model, d, bodyId, false, 0);
        state(i + DOF) = globalMujocoController->return_qVelVal(model, d, bodyId, false, 0);
    }

    // Get Mujoco Id's for items in the scene
    int goalBoxId = mj_name2id(model, mjOBJ_BODY, stateNames[7].c_str());
    int obstacleIds[4];

    state(7) = globalMujocoController->return_qPosVal(model, d, goalBoxId, true, 0);
    state(8) = globalMujocoController->return_qPosVal(model, d, goalBoxId, true, 1);

    state(7 + DOF) = globalMujocoController->return_qVelVal(model, d, goalBoxId, true, 0);
    state(8 + DOF) = globalMujocoController->return_qVelVal(model, d, goalBoxId, true, 1);

    // number of obstacles = 4
    for(int i = 0; i < 4; i++){
        obstacleIds[i] = mj_name2id(model, mjOBJ_BODY, stateNames[8 + i].c_str());
        // Set obstacle position
        state(9 + (2 * i)) = globalMujocoController->return_qPosVal(model, d, obstacleIds[i], true, 0);
        state(10 + (2 * i)) = globalMujocoController->return_qPosVal(model, d, obstacleIds[i], true, 1);

        // Set obstacle velocity
        state(9 + (2 * i) + DOF) = globalMujocoController->return_qVelVal(model, d, obstacleIds[i], true, 0);
        state(10 + (2 * i) + DOF) = globalMujocoController->return_qVelVal(model, d, obstacleIds[i], true, 1);
    }

    m_point EE_point = returnEE_point(d);


    return state;
}

m_dof frankaModel::returnVelocities(mjData *d){
    m_dof velocities;

    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        velocities(i) = globalMujocoController->return_qVelVal(model, d, bodyId, false, 0);
    }

    int goalBoxId = mj_name2id(model, mjOBJ_BODY, stateNames[7].c_str());
    int obstacleIds[4];

    velocities(NUM_CTRL) = globalMujocoController->return_qVelVal(model, d, goalBoxId, true, 0);
    velocities(NUM_CTRL + 1) = globalMujocoController->return_qVelVal(model, d, goalBoxId, true, 1);

    for(int i = 0; i < 4; i++){
        obstacleIds[i] = mj_name2id(model, mjOBJ_BODY, stateNames[8 + i].c_str());

        // Set obstacle velocity
        velocities(9 + (2 * i)) = globalMujocoController->return_qVelVal(model, d, obstacleIds[i], true, 0);
        velocities(10 + (2 * i)) = globalMujocoController->return_qVelVal(model, d, obstacleIds[i], true, 1);
    }

    return velocities;
}

m_dof frankaModel::returnAccelerations(mjData *d){
    m_dof accelerations;

    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        accelerations(i) = globalMujocoController->return_qAccVal(model, d, bodyId, false, 0);
    }

    int goalBoxId = mj_name2id(model, mjOBJ_BODY, stateNames[7].c_str());
    int obstacleIds[4];

    accelerations(NUM_CTRL) = globalMujocoController->return_qAccVal(model, d, goalBoxId, true, 0);
    accelerations(NUM_CTRL + 1) = globalMujocoController->return_qAccVal(model, d, goalBoxId, true, 1);

    for(int i = 0; i < 4; i++){
        obstacleIds[i] = mj_name2id(model, mjOBJ_BODY, stateNames[8 + i].c_str());

        // Set obstacle velocity
        accelerations(9 + (2 * i)) = globalMujocoController->return_qAccVal(model, d, obstacleIds[i], true, 0);
        accelerations(10 + (2 * i)) = globalMujocoController->return_qAccVal(model, d, obstacleIds[i], true, 1);
    }

    return accelerations;
}

m_state frankaModel::generateRandomStartState(mjData *d){
    m_state randState;

    randState <<   0.119, 0.568, 0, -2.55, -2.97, -0.01, 0,
            //0.8, 0, 0.6, 0.1, 0.6, -0.1, 0.7, 0.2, 0.7, -0.2,
            1.8, 0, 1.6, 0.1, 1.6, -0.1, 1.7, 0.2, 1.7, -0.2,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    return randState;
}

m_state frankaModel::generateRandomGoalState(m_state startState, mjData *d){
    m_state randState;

    randState <<   0.119, 0.568, 0, -2.55, -2.97, -0.0848, 0,
            0.8, 0, 0.6, 0.1, 0.6, -0.1, 0.7, 0.2, 0.7, -0.2,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    return randState;
}

#endif

// Set the controls of a mujoco data object
void frankaModel::setControls(mjData *d, m_ctrl U, bool grippersOpen){
    for(int i = 0; i < NUM_CTRL; i++){
        d->ctrl[i] = U(i);
    }

#ifndef DOUBLE_PENDULUM
    if(grippersOpen){
        d->ctrl[7] = GRIPPERS_OPEN;
        d->ctrl[8] = GRIPPERS_OPEN;
    }
    else{
        d->ctrl[7] = GRIPPERS_CLOSED;
        d->ctrl[8] = GRIPPERS_CLOSED;
    }
#endif
}

// Return the controls of a mujoco data object
m_ctrl frankaModel::returnControls(mjData *d){
    m_ctrl controls;
    for(int i = 0; i < NUM_CTRL; i++){
        controls(i) = d->ctrl[i];
    }

    return controls;
}

void frankaModel::perturbVelocity(mjData *perturbedData, mjData *origData, int stateIndex, double eps){
    int stateNameIndex = stateIndexToStateName[stateIndex];
    int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[stateNameIndex].c_str());
    bool freeJoint;
    int freeJntIndex = stateIndexToFreeJntIndex[stateIndex];

    if(stateIndex <= 6){
        freeJoint = false;
    }
    else{
        freeJoint = true;
    }

    double origVelocity = globalMujocoController->return_qVelVal(model, origData, bodyId, freeJoint, freeJntIndex);
    double perturbedVel = origVelocity + eps;
    globalMujocoController->set_qVelVal(model, perturbedData, bodyId, freeJoint, freeJntIndex, perturbedVel);

}

void frankaModel::perturbPosition(mjData *perturbedData, mjData *origData, int stateIndex, double eps){
    int stateNameIndex = stateIndexToStateName[stateIndex];
    int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[stateNameIndex].c_str());
    bool freeJoint;
    int freeJntIndex = stateIndexToFreeJntIndex[stateIndex];
    bool quatMath = false;

    if(stateIndex <= 6){
        freeJoint = false;
    }
    else{
        freeJoint = true;
    }

    if(!quatMath){
        double origPos = globalMujocoController->return_qPosVal(model, origData, bodyId, freeJoint, freeJntIndex);
        double perturbedPos = origPos + eps;
        globalMujocoController->set_qPosVal(model, perturbedData, bodyId, freeJoint, freeJntIndex, perturbedPos);
    }
    else{

        m_quat origQuat = globalMujocoController->returnBodyQuat(model, origData, bodyId);
        m_point origAxis = globalMujocoController->quat2Axis(origQuat);

        m_point perturbedAxis;
        perturbedAxis = origAxis.replicate(1,1);
        perturbedAxis(1) += eps;

        m_quat perturbedQuat = globalMujocoController->axis2Quat(perturbedAxis);
        globalMujocoController->setBodyQuat(model, perturbedData, bodyId, perturbedQuat);

    }
}

void frankaModel::stepModel(mjData *d, int numSteps){
    for(int i = 0; i < numSteps; i++){
        mj_step(model, d);
    }
}

bool frankaModel::isStateInCollision(mjData *d, m_state state){
    bool collision = false;
    mjData *resetData;
    resetData = mj_makeData(model);
    cpMjData(model, resetData, d);

    setState(d, state);
    mj_step1(model, d);

    int numberOfCollisions = globalMujocoController->getRobotNumberOfCollisions(d);

    cpMjData(model, d, resetData);
    mj_deleteData(resetData);

    if (numberOfCollisions > 0) {
        collision = true;
    }

    return collision;
}


//// Given a set of mujoco data, what is the cost of its state and controls
//double frankaModel::costFunction(int controlNum, int totalControls, m_state X, m_ctrl U, m_ctrl lastControl, bool firstControl){
//    double stateCost = 0;
//    m_state X_diff;
//
//    VectorXd temp(1);
//
//    // actual - desired
//    X_diff = X - X_desired;
//
//    // percentage done is a value between 0 and 1
//    double percentageDone = (float)controlNum / (float)totalControls;
//    double terminalScalar;
//    if(percentageDone < 0.5){
//        terminalScalar = 0;
//    }
//    else{
//        terminalScalar = (terminalConstant * pow(percentageDone, 2)) + 1;
//    }
//
//    double obstacleDistCost = 0.0f;
////    double objectsDiffX = X(9) - alphax;
////    double objectsDiffY = X(10) - alphay;
////
////    obstacleDistCost = (A * exp(-(pow(objectsDiffX,2)/sigma))) + (A * exp(-(pow(objectsDiffY,2)/sigma)));
////    if(percentageDone > 0.2){
////        obstacleDistCost = (5 * exp(-(pow(objectsDiffX,2)/0.05))) + (5 * exp(-(pow(objectsDiffY,2)/0.03)));
////        //obstacleDistCost = (0.1 / (pow(objectsDiffX,2) + 0.1)) + (0.1 / (pow(objectsDiffY, 2) + 0.1));
////        // 0.03
////    }
////    else{
////        obstacleDistCost = (0.01 * exp(-(pow(objectsDiffX,2)/0.03))) + (0.01 * exp(-(pow(objectsDiffY,2)/0.03)));
////    }
//
//
//    DiagonalMatrix<double, 2 * DOF> Q_scaled;
//    for(int i = 0; i < DOF; i++){
//        Q_scaled.diagonal()[i] = terminalScalar * Q.diagonal()[i];
//        Q_scaled.diagonal()[i + DOF] = Q.diagonal()[i + DOF];
//    }
//
//    double jerkCost = 0.0f;
//    VectorXd jerkTemp(1);
//    if(!firstControl){
//        m_ctrl U_diff = U - lastControl;
//        jerkTemp = U_diff.transpose() * J * U_diff;
//
//        jerkCost = jerkTemp(0);
//    }
//
//    temp = ((X_diff.transpose() * Q_scaled * X_diff)) + (U.transpose() * R * U);
//
//    stateCost = temp(0) + jerkCost + (obstacleDistCost);
//
//    return stateCost;
//}

//m_dof frankaModel::returnPositions(mjData *d){
//    m_dof positions;
//
//    for(int i = 0; i < NUM_CTRL; i++){
//        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
//        positions(i) = globalMujocoController->return_qPosVal(model, d, bodyId, false, 0);
//    }
//
//    int boxId = mj_name2id(model, mjOBJ_BODY, stateNames[7].c_str());
//    positions(NUM_CTRL) = globalMujocoController->return_qPosVal(model, d, boxId, true, 0);
//    positions(NUM_CTRL + 1) = globalMujocoController->return_qPosVal(model, d, boxId, true, 1);
//
//
//    m_quat bodyQuat = globalMujocoController->returnBodyQuat(model, d, boxId);
//    m_point bodyAxis = globalMujocoController->quat2Axis(bodyQuat);
//
//    positions(NUM_CTRL + 2) = bodyAxis(1);
//
//    return positions;
//}

//m_dof frankaModel::returnVelocities(mjData *d){
//    m_dof velocities;
//
//    for(int i = 0; i < NUM_CTRL; i++){
//        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
//        velocities(i) = globalMujocoController->return_qVelVal(model, d, bodyId, false, 0);
//    }
//
//    int goalBoxId = mj_name2id(model, mjOBJ_BODY, stateNames[7].c_str());
//    //int boxObstacle1Id = mj_name2id(model, mjOBJ_BODY, stateNames[8].c_str());
//
//    velocities(NUM_CTRL) = globalMujocoController->return_qVelVal(model, d, goalBoxId, true, 0);
//    velocities(NUM_CTRL + 1) = globalMujocoController->return_qVelVal(model, d, goalBoxId, true, 1);
//
//    //velocities(NUM_CTRL + 2) = globalMujocoController->return_qVelVal(model, d, boxObstacle1Id, true, 0);
//    //velocities(NUM_CTRL + 3) = globalMujocoController->return_qVelVal(model, d, boxObstacle1Id, true, 1);
//
//    return velocities;
//}
//
//m_dof frankaModel::returnAccelerations(mjData *d){
//    m_dof accelerations;
//
//    for(int i = 0; i < NUM_CTRL; i++){
//        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
//        accelerations(i) = globalMujocoController->return_qAccVal(model, d, bodyId, false, 0);
//    }
//
//    int goalBoxId = mj_name2id(model, mjOBJ_BODY, stateNames[7].c_str());
//    //int boxObstacle1Id = mj_name2id(model, mjOBJ_BODY, stateNames[8].c_str());
//
//    accelerations(NUM_CTRL) = globalMujocoController->return_qAccVal(model, d, goalBoxId, true, 0);
//    accelerations(NUM_CTRL + 1) = globalMujocoController->return_qAccVal(model, d, goalBoxId, true, 1);
//
//    //accelerations(NUM_CTRL + 2) = globalMujocoController->return_qAccVal(model, d, boxObstacle1Id, true, 0);
//    //accelerations(NUM_CTRL + 3) = globalMujocoController->return_qAccVal(model, d, boxObstacle1Id, true, 1);
//
//
//    return accelerations;
//}
