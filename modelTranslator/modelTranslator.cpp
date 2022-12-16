//
// Created by david on 03/05/22.
//

#include "modelTranslator.h"

taskTranslator::taskTranslator(){

    for(int i = 0; i < 7; i++) {
        stateNames.push_back(std::string());
    }

#ifdef PENDULUM_TASK
    taskNumber = 0;

    stateNames[0] = "upper_arm";
    stateNames[1] = "lower_arm";

    armControlCosts = 0.01;
    double armStateCosts = 2;
    double armVelCosts = 1;

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
    double armVelCosts = 1;

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
//    stateNames[0] = "link1";
//    stateNames[1] = "link2";
//    stateNames[2] = "link3";
//    stateNames[3] = "link4";
//    stateNames[4] = "link5";
//    stateNames[5] = "link6";
//    stateNames[6] = "link7";
//    stateNames[7] = "tin";

    armControlCosts = 0;
    double armStateCosts = 0;
    double armVelCosts = 0.1;

    for(int i = 0; i < NUM_CTRL; i++){
        R.diagonal()[i] = armControlCosts;
        if(TORQUE_CONTROL){
            J.diagonal()[i] = 0.0007;
        }
        else{
            J.diagonal()[i] = 0.02;
        }

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

void taskTranslator::init(mjModel *m){
    model = m;
}

// Set the controls of a mujoco data object
void taskTranslator::setControls(mjData *d, m_ctrl U, bool grippersOpen){

#ifdef OBJECT_PUSHING
    if(TORQUE_CONTROL){
        for(int i = 0; i < NUM_CTRL; i++){
            d->ctrl[i] = U(i) + d->qfrc_bias[i];
        }
    }
    else{
        for(int i = 0; i < NUM_CTRL; i++){
            d->ctrl[i] = U(i);
        }
    }

#else
    for(int i = 0; i < NUM_CTRL; i++){
        d->ctrl[i] = U(i);
    }
#endif

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
m_ctrl taskTranslator::returnControls(mjData *d){
    m_ctrl controls;
    for(int i = 0; i < NUM_CTRL; i++){
        controls(i) = d->ctrl[i];
    }

    return controls;
}

void taskTranslator::perturbVelocity(mjData *perturbedData, mjData *origData, int stateIndex, double eps){
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

void taskTranslator::perturbPosition(mjData *perturbedData, mjData *origData, int stateIndex, double eps){
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

void taskTranslator::stepModel(mjData *d, int numSteps){
    for(int i = 0; i < numSteps; i++){
        mj_step(model, d);
    }
}

bool taskTranslator::isStateInCollision(mjData *d, m_state state){
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
