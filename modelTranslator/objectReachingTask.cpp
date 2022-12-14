//
// Created by davem on 14/12/2022.
//

#include "modelTranslator.h"


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
