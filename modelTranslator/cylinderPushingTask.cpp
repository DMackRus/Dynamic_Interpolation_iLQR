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

#ifdef CYLINDER_PUSHING_TASK

void initControls_MainWayPoints_Setup(mjData *d, mjModel *model, double angle_Push, m_point objectStart, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming);
void initControls_MainWayPoints_Optimise(mjData *d, mjModel *model, m_point desiredObjectEnd, double angle_EE_push, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming);
std::vector<m_point> initControls_createAllWayPoints(std::vector<m_point> mainWayPoints, std::vector<int> wayPointsTiming);
std::vector<m_ctrl> initControls_generateAllControls(mjData *d, mjModel *model,std::vector<m_point> initPath, double angle_EE_push);

double taskTranslator::costFunction(mjData *d, int controlNum, int totalControls, mjData *d_last){
    double stateCost = 0;
    m_state X = returnState(d);
    m_ctrl U = returnControls(d);
    m_dof velocities = returnVelocities(d);
    m_dof velocities_last = returnVelocities(d_last);
    m_ctrl armVels_now;
    m_ctrl armVels_last;
    for(int i = 0; i < NUM_CTRL; i++){
        armVels_now(i) = velocities(i);
        armVels_last(i) = velocities_last(i);
    }
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

    bool terminal = false;
    if(controlNum == (totalControls - 1)){
        terminal = true;
        //cout << "terminal" << endl;
    }

    DiagonalMatrix<double, 2 * DOF> Q_scaled;
    for(int i = 0; i < DOF; i++){
        if(terminal){
            Q_scaled.diagonal()[i] = TERMINAL_STATE_MULT * Q.diagonal()[i];
        }
        else{
            Q_scaled.diagonal()[i] = Q.diagonal()[i];
        }

        Q_scaled.diagonal()[i + DOF] = Q.diagonal()[i + DOF];
    }

    //m_ctrl controlDiff = U - lastControl;
    m_ctrl velDiff = armVels_now - armVels_last;

    temp = ((X_diff.transpose() * Q_scaled * X_diff)) + (U.transpose() * R * U) + (velDiff.transpose() * J * velDiff);

    stateCost = temp(0);

    return stateCost;

}

// Given a set of mujoco data, what are its cost derivates with respect to state and control
void taskTranslator::costDerivatives(mjData *d, Ref<m_state> l_x, Ref<m_state_state> l_xx, Ref<m_ctrl> l_u, Ref<m_ctrl_ctrl> l_uu, int controlNum, int totalControls, mjData *d_last){
    m_state X_diff;
    m_state X;
    m_ctrl U;
    m_ctrl U_diff;
    m_dof velocities = returnVelocities(d);
    m_dof velocities_last = returnVelocities(d_last);
    m_ctrl armVels_now;
    m_ctrl armVels_last;
    for(int i = 0; i < NUM_CTRL; i++){
        armVels_now(i) = velocities(i);
        armVels_last(i) = velocities_last(i);
    }

    X = returnState(d);
    U = returnControls(d);
    m_ctrl lastU = returnControls(d_last);
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

    bool terminal = false;
    if(controlNum == (totalControls - 1)){
        terminal = true;
    }

    DiagonalMatrix<double, 2 * DOF> Q_scaled;
    for(int i = 0; i < DOF; i++){
        if(terminal){
            Q_scaled.diagonal()[i] = TERMINAL_STATE_MULT * Q.diagonal()[i];
        }
        else{
            Q_scaled.diagonal()[i] = Q.diagonal()[i];
        }
        Q_scaled.diagonal()[i + DOF] = Q.diagonal()[i + DOF];
    }

    m_ctrl velDiff = armVels_now - armVels_last;

    l_x = 2 *  Q_scaled * X_diff;
    l_xx = 2 * Q_scaled;

    l_u = (2 * R * U) + (2 * J * velDiff);
    l_uu.setZero();
    for(int i = 0; i < NUM_CTRL; i++){
        l_uu(i, i) = (2 * R.diagonal()[i]) + (2 * J.diagonal()[i]);
    }

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

//    const std::string EE_Name = "franka_gripper";
//    X0(5) += PI/4;
//    setState(d, X0);
//    int EE_id = mj_name2id(model, mjOBJ_BODY, EE_Name.c_str());
//    m_quat quat = globalMujocoController->returnBodyQuat(model, d, EE_id);
//    cout << "Quat: " << quat << endl;
//
//    m_point eul = globalMujocoController->quat2Eul(quat);
//
//    cout << "Eul: " << eul << endl;

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

void taskTranslator::setX_Desired(m_state _X_desired, mjData *d){
    X_desired = _X_desired.replicate(1, 1);

    if(taskNumber == 2){
        int visualGoalId = mj_name2id(model, mjOBJ_BODY, "display_goal");

        m_pose goalPose;
        goalPose.setZero();
        goalPose(0) = X_desired(7);
        goalPose(1) = X_desired(8);

        globalMujocoController->setBodyPose(model, d, visualGoalId, goalPose);
    }
}

// Generate initial controls to set up Task
std::vector<m_ctrl> taskTranslator::initSetupControls(mjData *d, mjData *d_init){
    std::vector<m_ctrl> initControls;
    cpMjData(model, d, d_init);

    // Main waypoints to follow and how long to take to get there
    std::vector<m_point> mainWayPoints;
    std::vector<int> wayPoints_timings;

    m_state X0 = returnState(d_init);

    m_point desiredObjectEnd;
    desiredObjectEnd(0) = X_desired(7);
    desiredObjectEnd(1) = X_desired(8);
    m_point objectStart;
    objectStart(0) = X0(7);
    objectStart(1) = X0(8);

//    cout << "desiredObjectEnd: " << desiredObjectEnd << endl;
//    cout << "objectStart: " << objectStart << endl;

    double angle_EE_push = atan2(desiredObjectEnd(1) - objectStart(1), desiredObjectEnd(0) - objectStart(0));

    initControls_MainWayPoints_Setup(d, model, angle_EE_push, objectStart, mainWayPoints, wayPoints_timings);

    std::vector<m_point> initPath = initControls_createAllWayPoints(mainWayPoints, wayPoints_timings);

    initControls = initControls_generateAllControls(d, model, initPath, angle_EE_push);

    return initControls;
}

void initControls_MainWayPoints_Setup(mjData *d, mjModel *model, double angle_Push, m_point objectStart, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming){
    const std::string goalName = "goal";

    const std::string EE_name = "franka_gripper";
    int EE_id = mj_name2id(model, mjOBJ_BODY, EE_name.c_str());

    m_pose startPose = globalMujocoController->returnBodyPose(model, d, EE_id);

    m_point mainWayPoint;
    mainWayPoint << startPose(0), startPose(1), startPose(2);
    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(0);

    float cylinderObjectX = objectStart(0);
    float cylinderObjectY = objectStart(1);

    // TODO hard coded - get it programmatically? - also made it slightly bigger so trajectory has room to improve
//    float cylinder_radius = 0.08;
    float cylinder_radius = 0.1;

    float intermediatePointX;
    float intermediatePointY;

    float h = 0.15;

    float deltaX = h * cos(angle_Push);
    float deltaY = h * sin(angle_Push);

    intermediatePointY = cylinderObjectY - deltaY;
    intermediatePointX = cylinderObjectX - deltaX;

    // Setting this up so we can visualise where the intermediate point is located
    intermediatePoint(0) = intermediatePointX;
    intermediatePoint(1) = intermediatePointY;

    mainWayPoint(0) = intermediatePoint(0);
    mainWayPoint(1) = intermediatePoint(1);
    mainWayPoint(2) = 0.27f;

    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(750);
}

// Generate initial controls to be optimised
std::vector<m_ctrl> taskTranslator::initOptimisationControls(mjData *d, mjData *d_init) {

    std::vector<m_ctrl> initControls;
    cpMjData(model, d, d_init);

    std::vector<m_point> mainWayPoints;
    std::vector<int> wayPoints_timings;

    m_state X0 = returnState(d_init);

    m_point desiredObjectEnd;
    desiredObjectEnd(0) = X_desired(7);
    desiredObjectEnd(1) = X_desired(8);
    m_point objectStart;
    objectStart(0) = X0(7);
    objectStart(1) = X0(8);

    double angle_EE_push = atan2(desiredObjectEnd(1) - objectStart(1), desiredObjectEnd(0) - objectStart(0));
    cout << "angle_EE_push: " << angle_EE_push << endl;

    initControls_MainWayPoints_Optimise(d, model, desiredObjectEnd, angle_EE_push, mainWayPoints, wayPoints_timings);
    std::vector<m_point> initPath = initControls_createAllWayPoints(mainWayPoints, wayPoints_timings);

    initControls = initControls_generateAllControls(d, model, initPath, angle_EE_push);

    return initControls;
}

void initControls_MainWayPoints_Optimise(mjData *d, mjModel *model, m_point desiredObjectEnd, double angle_EE_push, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming){
    const std::string goalName = "goal";

    const std::string EE_name = "franka_gripper";

    int EE_id = mj_name2id(model, mjOBJ_BODY, EE_name.c_str());
    int goal_id = mj_name2id(model, mjOBJ_BODY, goalName.c_str());

    m_pose startPose = globalMujocoController->returnBodyPose(model, d, EE_id);

    m_point mainWayPoint;
    mainWayPoint << startPose(0), startPose(1), startPose(2);
    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(0);

    // TODO hard coded - get it programmatically? - also made it slightly bigger so trajectory has room to improve
//    float cylinder_radius = 0.08;
    float cylinder_radius = 0.1;
    float x_cylinder0ffset = cylinder_radius * cos(angle_EE_push);
    float y_cylinder0ffset = cylinder_radius * sin(angle_EE_push);

    float desired_endPointX = desiredObjectEnd(0) - x_cylinder0ffset;
    float desired_endPointY;

    float endPointX;
    float endPointY;
    if(desiredObjectEnd(1) - startPose(1) > 0){
        desired_endPointY = desiredObjectEnd(1) + y_cylinder0ffset;
    }
    else{
        desired_endPointY = desiredObjectEnd(1) - y_cylinder0ffset;
    }

    float intermediatePointY = startPose(1);
    float intermediatePointX = startPose(0);

    // Setting this up so we can visualise where the intermediate point is located
    intermediatePoint(0) = intermediatePointX;
    intermediatePoint(1) = intermediatePointY;

    float maxDistTravelled = 0.05 * ((5.0f/6.0f) * MUJ_STEPS_HORIZON_LENGTH * MUJOCO_DT);
//    cout << "max EE travel dist: " << maxDistTravelled << endl;
    float desiredDistTravelled = sqrt(pow((desired_endPointX - intermediatePointX),2) + pow((desired_endPointY - intermediatePointY),2));
    float proportionOfDistTravelled = maxDistTravelled / desiredDistTravelled;
//    cout << "proportion" << proportionOfDistTravelled << endl;
    if(proportionOfDistTravelled > 1){
        endPointX = desired_endPointX;
        endPointY = desired_endPointY;
    }
    else{
        endPointX = intermediatePointX + ((desired_endPointX - intermediatePointX) * proportionOfDistTravelled);
        endPointY = intermediatePointY + ((desired_endPointY - intermediatePointY) * proportionOfDistTravelled);
    }

    mainWayPoint(0) = endPointX;
    mainWayPoint(1) = endPointY;
    mainWayPoint(2) = 0.27f;

    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(1750);

}

std::vector<m_point> initControls_createAllWayPoints(std::vector<m_point> mainWayPoints, std::vector<int> wayPointsTiming){
    int numMainWayPoints = mainWayPoints.size();
    std::vector<m_point> initPath;

    initPath.push_back(mainWayPoints[0]);
    wayPointsTiming[0]--;

    // should only be MUJ_STEPS_HORIZON_LENGTH number of controls
    int counter = 1;
    for(int i = 0; i < numMainWayPoints - 1; i++){
        float x_diff = mainWayPoints[i + 1](0) - mainWayPoints[i](0);
        float y_diff = mainWayPoints[i + 1](1) - mainWayPoints[i](1);
        float z_diff = mainWayPoints[i + 1](2) - mainWayPoints[i](2);
        for(int j = 0; j < wayPointsTiming[i + 1]; j++){
            initPath.push_back(m_point());
            initPath[counter](0) = initPath[counter - 1](0) + (x_diff / wayPointsTiming[i + 1]);
            initPath[counter](1) = initPath[counter - 1](1) + (y_diff / wayPointsTiming[i + 1]);
            initPath[counter](2) = initPath[counter - 1](2) + (z_diff / wayPointsTiming[i + 1]);

            counter++;
            if(counter > MUJ_STEPS_HORIZON_LENGTH){
                cout << "ERROR, TOO MANY POINTS IN INIT PATH" << endl;
            }
        }
    }

    return initPath;
}

std::vector<m_ctrl> initControls_generateAllControls(mjData *d, mjModel *model, std::vector<m_point> initPath, double angle_EE_push){
    std::vector<m_ctrl> initControls;

    taskTranslator tempModelTranslator;
    tempModelTranslator.init(model);
    int EE_id = mj_name2id(model, mjOBJ_BODY, tempModelTranslator.EE_name.c_str());
    m_quat startQuat = globalMujocoController->returnBodyQuat(model, d, EE_id);

    m_pose startPose = globalMujocoController->returnBodyPose(model, d, EE_id);
    globalMujocoController->quat2RotMat(startQuat);


//    cout << "converted angle is: " << convertedAngle << endl;
    if(angle_EE_push < 0){
        angle_EE_push = angle_EE_push + (2*PI);
//        cout << "converted angle is: " << convertedAngle << endl;
    }

    double convertedAngle = angle_EE_push - (PI/4);


    m_point xAxis, yAxis, zAxis;
    xAxis << cos(convertedAngle), sin(convertedAngle), 0;
    zAxis << 0, 0, -1;
    yAxis = globalMujocoController->crossProduct(zAxis, xAxis);



    Eigen::Matrix3d rotMat;
    rotMat << xAxis(0), yAxis(0), zAxis(0),
            xAxis(1), yAxis(1), zAxis(1),
            xAxis(2), yAxis(2), zAxis(2);

    m_quat desiredQuat = globalMujocoController->rotMat2Quat(rotMat);
    //cout << "angle EE push is: " << angle_EE_push << endl;
    //cout << "x contribution:" << cos(convertedAngle) << "y contribution: " << sin(convertedAngle) << endl;
    //cout << "yAxis: " << yAxis << endl;

    //cout << "desired quat: " << desiredQuat << endl;

    m_ctrl desiredControls;

    // Calculate the initial position control to be the starting position
    if(!TORQUE_CONTROL){
        m_state startState = tempModelTranslator.returnState(d);
        for(int i = 0; i < NUM_CTRL; i++){
            desiredControls(i) = startState(i);
        }
    }

    for (int i = 0; i < initPath.size(); i++) {

        m_pose currentEEPose = globalMujocoController->returnBodyPose(model, d, EE_id);
        m_quat currentQuat = globalMujocoController->returnBodyQuat(model, d, EE_id);
        m_quat invQuat = globalMujocoController->invQuat(currentQuat);
        m_quat quatDiff = globalMujocoController->multQuat(desiredQuat, invQuat);

        m_point axisDiff = globalMujocoController->quat2Axis(quatDiff);

        m_pose differenceFromPath;
        float gains[6] = {10000, 10000, 30000, 5000, 5000, 5000};
        for (int j = 0; j < 3; j++) {
            differenceFromPath(j) = initPath[i](j) - currentEEPose(j);
            differenceFromPath(j + 3) = axisDiff(j);
        }

        // Calculate jacobian inverse
        MatrixXd Jac = globalMujocoController->calculateJacobian(model, d, EE_id);
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

        tempModelTranslator.setControls(d, initControls[i], false);

        tempModelTranslator.stepModel(d, 1);

//        if (i % 10 == 0) {
//            mjrRect viewport = {0, 0, 0, 0};
//            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
//
//            // update scene and render
//            mjv_updateScene(model, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
//            mjr_render(viewport, &scn, &con);
//
//            // swap OpenGL buffers (blocking call due to v-sync)
//            glfwSwapBuffers(window);
//
//            // process pending GUI events, call GLFW callbacks
//            glfwPollEvents();
//        }

    }

    return initControls;
}

bool taskTranslator::taskCompleted(mjData *d){
    bool taskComplete = false;

    m_state currentState = returnState(d);

    // for the pushing task, the task is complete if the object is within a certain range of the destination
    float diffX = X_desired(7) - currentState(7);
    float diffY = X_desired(8) - currentState(8);
    float dist = sqrt(pow(diffX, 2) + pow(diffY, 2));

    // if object distance to goal is below some threshold, then task complete
    if(dist < 0.03){
        taskComplete = true;
    }

    return taskComplete;
}

bool taskTranslator::taskFailed(mjData *d){
    bool taskFailed = false;

    return taskFailed;
}

bool taskTranslator::predictiveStateMismatch(mjData *d_predicted, mjData *d_real){
    bool stateMismatch = false;

    double cumError = 0.0f;
    m_state actualState = returnState(d_real);
    m_state predictedState = returnState(d_predicted);
    double errorGains[DOF] = {0, 0, 0, 0, 0, 0, 0, 1, 1};
    double errors[DOF];

//    for(int i = 0; i < 2; i++){
//        cumError += abs(actualState(i + NUM_CTRL) - predictedState(i + NUM_CTRL));
//    }
    double X_diff = actualState(7) - predictedState(7);
    double Y_diff = actualState(8) - predictedState(8);
    double cubeDiff = sqrt(X_diff*X_diff + Y_diff*Y_diff);

    for(int i = 0; i < DOF; i++){
        errors[i] = errorGains[i] * (abs(actualState(i) - predictedState(i)));
        cumError += errors[i];
    }

    //cout << "cum error: " << cumError << endl;

    // if cumulative error is greater than some threshold
    if(cubeDiff > 0.1){
        stateMismatch = true;

        cout << "errors: " << errors[0] << ", " << errors[1] << ", " << errors[2] << ", " << errors[3] << ", " << errors[4] << ", " << errors[5] << ", " << errors[6] << ", " << errors[7] << ", " << errors[8] << endl;

        cout << "cube x predicted: " << predictedState(7) << endl;
        cout << "cube x real: " << actualState(7) << endl;
        cout << "cube y predicted: " << predictedState(8) << endl;
        cout << "cube y real: " << actualState(8) << endl;
    }

    return stateMismatch;
}

// Be careful with the math in this function, kind of changed x and Y around due to mujoco x being forwards from robot perspective
bool taskTranslator::newControlInitialisationNeeded(mjData *d, int counterSinceLastControlInitialisation){

    // Get End effector position

    if(counterSinceLastControlInitialisation > 100){
        const std::string goalName = "goal";

        int EE_id = mj_name2id(model, mjOBJ_BODY, EE_name.c_str());
        int goal_id = mj_name2id(model, mjOBJ_BODY, goalName.c_str());

        m_pose EE_Pose = globalMujocoController->returnBodyPose(model, d, EE_id);
        // Get object position
        m_point objectPos;
        m_state currentState = returnState(d);
        objectPos(0) = currentState(7);
        objectPos(1) = currentState(8);

        // calculate angle between goal and EE
        float angle_goal_EE = atan2(X_desired(7) - EE_Pose(0), X_desired(8) - EE_Pose(1));

        // calculate angle between goal and object
        float angle_goal_object = atan2(X_desired(7) - objectPos(0), X_desired(8) - objectPos(1));

        float angleDiff;
        angleDiff = angle_goal_object - angle_goal_EE;
        if(angleDiff > 0.5){
            cout << "reinitialise needed, angle diff: " << angleDiff << "\n";
            return true;
        }
        if(angleDiff < -0.5){
            cout << "reinitialise needed, angle diff: " << angleDiff << "\n";
            return true;
        }


        // if the end effector has gotten to high
        if(EE_Pose(2) > 0.4){
            cout << "reinitialise needed, EE too high \n";
            return true;
        }
//        if(X_desired(8) < objectPos(1)){
//
//        }
//        else{
//            if(angleDiff < -0.1){
//                return true;
//            }
//        }
    }
    else{
        if(counterSinceLastControlInitialisation > 1000){
            return true;
        }
        else{
            return false;
        }
    }


    // compare this angle and if greater than some threshold, return true


//    if(angleDiff > (PI/8)){
//        cout << "EE not behind object enough, REINITIALISE CONTROLS \n";
//        return true;
//    }

    return false;
}

#endif
