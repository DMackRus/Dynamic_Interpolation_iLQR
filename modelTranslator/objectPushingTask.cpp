//
// Created by davem on 14/12/2022.
//

#include "modelTranslator.h"

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
