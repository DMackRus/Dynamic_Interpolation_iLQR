//
// Created by davem on 14/12/2022.
//

#include "modelTranslator.h"

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
