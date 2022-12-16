//
// Created by davem on 14/12/2022.
//

#include "modelTranslator.h"

extern m_point intermediatePoint;

extern mjvCamera cam;                   // abstract camera
extern mjvScene scn;                    // abstract scene
extern mjvOption opt;			        // visualization options
extern mjrContext con;				    // custom GPU context
extern GLFWwindow *window;

#ifdef OBJECT_PUSHING
double taskTranslator::costFunction(mjData *d, int controlNum, int totalControls, m_ctrl lastControl){
    double stateCost = 0;
    m_state X = returnState(d);
    m_ctrl U = returnControls(d);
    m_state X_diff;

    VectorXd temp(1);

    // actual - desired
    X_diff = X - X_desired;
    double percentageDone = (double)controlNum / (double)totalControls;
    double terminalScalar = 1.0;
//    if(percentageDone < 0.6){
//        terminalScalar = 0;
//    }
//    else{
//        terminalScalar = (terminalConstant * pow(percentageDone, 2)) + 1;
//    }

    DiagonalMatrix<double, 2 * DOF> Q_scaled;
    for(int i = 0; i < DOF; i++){
        if(controlNum == (totalControls - 1)){
            cout << "terminal" << endl;
            cout << "controlNum: " << controlNum << " - total controls: " << totalControls << endl;
            Q_scaled.diagonal()[i] = TERMINAL_STATE_MULT * Q.diagonal()[i];
        }
        else{
            Q_scaled.diagonal()[i] = Q.diagonal()[i];
        }

        Q_scaled.diagonal()[i + DOF] = Q.diagonal()[i + DOF];
    }

    temp = ((X_diff.transpose() * Q_scaled * X_diff)) + (U.transpose() * R * U);

    stateCost = temp(0);

    return stateCost;

}

// Given a set of mujoco data, what are its cost derivates with respect to state and control
void taskTranslator::costDerivatives(mjData *d, Ref<m_state> l_x, Ref<m_state_state> l_xx, Ref<m_ctrl> l_u, Ref<m_ctrl_ctrl> l_uu, int controlNum, int totalControls, m_ctrl lastU){
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
    double terminalScalar = 1.0;
//    if(percentageDone < 0.6){
//        terminalScalar = 0;
//    }
//    else{
//        terminalScalar = (terminalConstant * pow(percentageDone, 2)) + 1;
//    }

    DiagonalMatrix<double, 2 * DOF> Q_scaled;
    for(int i = 0; i < DOF; i++){
        if(controlNum == (totalControls - 1)){
            cout << "terminal" << endl;
            cout << "controlNum: " << controlNum << " - total controls: " << totalControls << endl;
            Q_scaled.diagonal()[i] = TERMINAL_STATE_MULT * Q.diagonal()[i];
        }
        else{
            Q_scaled.diagonal()[i] = Q.diagonal()[i];
        }
        Q_scaled.diagonal()[i + DOF] = Q.diagonal()[i + DOF];
    }

    l_x = 2 *  Q_scaled * X_diff;
    l_xx = 2 * Q_scaled;

    l_u = (2 * R * U) + (2 * J * U_diff);
    l_uu = (2 * R) + (2 * J);
}

// set the state of a mujoco data object as per this model
void taskTranslator::setState(mjData *d, m_state X){

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
m_state taskTranslator::returnState(mjData *d){
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

m_dof taskTranslator::returnVelocities(mjData *d){
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

m_dof taskTranslator::returnAccelerations(mjData *d){
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

m_state taskTranslator::generateRandomStartState(mjData *d){
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

m_state taskTranslator::generateRandomGoalState(m_state startState, mjData *d){
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

m_state taskTranslator::setupTask(mjData *d, bool randomTask, int taskRow){
    m_state X0;

    if(!randomTask){

        cout << "Reading task from file" << endl;

        std::string nameOfFile = names[taskNumber];

        fstream fin;

        fin.open(nameOfFile, ios::in);
        std::vector<string> row;
        std::string line, word, temp;

        int counter = 0;

        while(fin >> temp){
            row.clear();

            getline(fin, line);

            stringstream s(temp);

            while(getline(s, word, ',')){
                row.push_back(word);
            }

            if(counter == taskRow){
                for(int i = 0; i < (2 * DOF); i++){
                    X0(i) = stod(row[i]);
                    X_desired(i) = stod(row[i + (2 * DOF)]);
                }
            }

            counter++;
        }

    //    X0(5) += PI/2;
    //    X0(6) += PI/4;


    }
    else{
        // Generate start and desired state randomly
        X0 = generateRandomStartState(d);
        //cout << "-------------- random init state ----------------" << endl << X0 << endl;
        //X_desired = generateRandomGoalState(X0, mdata);
        //cout << "-------------- random desired state ----------------" << endl << X_desired << endl;
    }

    setState(d, X0);

    if(taskNumber == 2){
        int visualGoalId = mj_name2id(model, mjOBJ_BODY, "display_goal");

        m_pose goalPose;
        goalPose.setZero();
        goalPose(0) = X_desired(7);
        goalPose(1) = X_desired(8);

        globalMujocoController->setBodyPose(model, d, visualGoalId, goalPose);
    }

    // smooth any random pertubations for starting data
    stepModel(d, 2);

    return X0;
}

// Generate initial trajectory
std::vector<m_ctrl> taskTranslator::initControls(mjData *d, mjData *d_init, m_state X0) {

    std::vector<m_ctrl> initControls;
    cpMjData(model, d, d_init);

    const std::string EE_Name = "franka_gripper";
    const std::string goalName = "goal";
    //panda0_leftfinger

//    const std::string EE_Name = "right_finger";
//    const std::string goalName = "tin";

    int EE_id = mj_name2id(model, mjOBJ_BODY, EE_Name.c_str());
    int goal_id = mj_name2id(model, mjOBJ_BODY, goalName.c_str());

    m_pose startPose = globalMujocoController->returnBodyPose(model, d, EE_id);
    m_quat startQuat = globalMujocoController->returnBodyQuat(model, d, EE_id);

    // TODO hard coded - get it programmatically? - also made it slightly bigger so trajectory has room to improve
    float cylinder_radius = 0.04;
    float x_cylinder0ffset = cylinder_radius * sin(PI/4);
    float y_cylinder0ffset = cylinder_radius * cos(PI/4);

    float endPointX = X_desired(7) - x_cylinder0ffset;
    float endPointY;
    if(X_desired(8) - startPose(0) > 0){
        endPointY = X_desired(8) - y_cylinder0ffset;
    }
    else{
        endPointY = X_desired(8) + y_cylinder0ffset;
    }

    float cylinderObjectX = X0(7);
    float cylinderObjectY = X0(8);
    float intermediatePointX;
    float intermediatePointY;

    float angle = atan2(endPointY - cylinderObjectY, endPointX - cylinderObjectX);

    if(TORQUE_CONTROL){
        if(endPointY > cylinderObjectY){
            angle += 0.4;
        }
        else{
            angle -= 0.4;
        }
    }
    else{
        if(endPointY > cylinderObjectY){
            angle += 0.1;
        }
        else{
            angle -= 0.1;
        }
    }

    float h = 0.15;

    float deltaX = h * cos(angle);
    float deltaY = h * sin(angle);

    // Calculate intermediate waypoint to position end effector behind cube such that it can be pushed to desired goal position
//    if(X_desired(8) - startPose(0) > 0){
//        intermediatePointY = cylinderObjectY + deltaY;
//    }
//    else{
//        intermediatePointY = cylinderObjectY - deltaY;
//    }
    intermediatePointY = cylinderObjectY - deltaY;
    intermediatePointX = cylinderObjectX - deltaX;

    intermediatePoint(0) = intermediatePointX;
    intermediatePoint(1) = intermediatePointY;

//    if(endPointY - cylinderObjectY > 0){
//        intermediatePointY -= 0.1;
//    }
//    else{
//        intermediatePointY += 0.1;
//    }

    float x_diff = intermediatePointX - startPose(0);
    float y_diff = intermediatePointY - startPose(1);

    m_point initPath[MUJ_STEPS_HORIZON_LENGTH];
    initPath[0](0) = startPose(0);
    initPath[0](1) = startPose(1);
    initPath[0](2) = startPose(2);

    int splitIndex = 500;

    for (int i = 0; i < 1000; i++) {
        initPath[i + 1](0) = initPath[i](0) + (x_diff / splitIndex);
        initPath[i + 1](1) = initPath[i](1) + (y_diff / splitIndex);
        initPath[i + 1](2) = initPath[i](2);
    }

    x_diff = endPointX - intermediatePointX;
    y_diff = endPointY - intermediatePointY;

    // Deliberately make initial;isation slightly worse so trajectory optimiser can do something
    for (int i = splitIndex; i < MUJ_STEPS_HORIZON_LENGTH - 1; i++) {
        initPath[i + 1](0) = initPath[i](0) + (x_diff / (MUJ_STEPS_HORIZON_LENGTH - splitIndex));
        initPath[i + 1](1) = initPath[i](1) + (y_diff / (MUJ_STEPS_HORIZON_LENGTH - splitIndex));
        initPath[i + 1](2) = initPath[i](2);
    }

    m_ctrl desiredControls;

    if(!TORQUE_CONTROL){
        m_state startState = returnState(d);
        for(int i = 0; i < NUM_CTRL; i++){
            desiredControls(i) = startState(i);
        }
    }

    cout << "start state" << desiredControls << endl;

    for (int i = 0; i <= MUJ_STEPS_HORIZON_LENGTH; i++) {

        m_pose currentEEPose = globalMujocoController->returnBodyPose(model, d, EE_id);
        m_quat currentQuat = globalMujocoController->returnBodyQuat(model, d, EE_id);

        m_quat invQuat = globalMujocoController->invQuat(currentQuat);
        m_quat quatDiff = globalMujocoController->multQuat(startQuat, invQuat);

        m_point axisDiff = globalMujocoController->quat2Axis(quatDiff);

        m_pose differenceFromPath;
        float gains[6] = {10000, 10000, 10000, 10, 10, 10};
        for (int j = 0; j < 3; j++) {
            differenceFromPath(j) = initPath[i](j) - currentEEPose(j);
            differenceFromPath(j + 3) = axisDiff(j);
        }
//
//        cout << "current path point: " << initPath[i] << endl;
//        cout << "currentEE Pose: " << currentEEPose << endl;
//        cout << "diff from path: " << differenceFromPath << endl;

        // Calculate jacobian inverse
        MatrixXd Jac = globalMujocoController->calculateJacobian(model, d, EE_id);
        MatrixXd Jac_t = Jac.transpose();
        MatrixXd Jac_inv = Jac.completeOrthogonalDecomposition().pseudoInverse();

        m_pose desiredEEForce;


        if(TORQUE_CONTROL){
            for (int j = 0; j < 6; j++) {
                desiredEEForce(j) = differenceFromPath(j) * gains[j];
            }
            desiredControls = Jac_inv * desiredEEForce;
        }
        else{
            // Position control
            for (int j = 0; j < 6; j++) {
                desiredEEForce(j) = differenceFromPath(j) * gains[j] * 0.000001;
            }
            desiredControls += Jac_inv * desiredEEForce;
            //cout << "desired controls: " << desiredControls << endl;
        }


        //cout << "desiredEEForce " << desiredEEForce << endl;


        initControls.push_back(m_ctrl());


        for (int k = 0; k < NUM_CTRL; k++) {

            initControls[i](k) = desiredControls(k);
        }

        setControls(d, initControls[i], false);

        stepModel(d, 1);

    }

    return initControls;
}

#endif
