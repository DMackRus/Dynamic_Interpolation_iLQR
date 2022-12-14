//
// Created by davem on 14/12/2022.
//
#include "modelTranslator.h"

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