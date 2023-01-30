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

void initControls_findMainWaypoints(mjData *d, mjModel *model, m_point desiredObjectEnd, m_point objectStart, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming);
std::vector<m_point> initControls_createAllWayPoints(std::vector<m_point> mainWayPoints, std::vector<int> wayPointsTiming);
std::vector<m_ctrl> initControls_generateAllControls(mjData *d, mjModel *model,std::vector<m_point> initPath);

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

// Generate initial trajectory
std::vector<m_ctrl> taskTranslator::initControls(mjData *d, mjData *d_init, m_state X0) {

    std::vector<m_ctrl> initControls;
    cpMjData(model, d, d_init);

    std::vector<m_point> mainWayPoints;
    std::vector<int> wayPoints_timings;

    m_point desiredObjectEnd;
    desiredObjectEnd(0) = X_desired(7);
    desiredObjectEnd(1) = X_desired(8);
    m_point objectStart;
    objectStart(0) = X0(7);
    objectStart(1) = X0(8);

    initControls_findMainWaypoints(d, model, desiredObjectEnd, objectStart, mainWayPoints, wayPoints_timings);
    for(int i = 0; i < mainWayPoints.size(); i++){
        cout << "main waypoint " << i << ": " << mainWayPoints[i] << endl;
        cout << "timing " << wayPoints_timings[i] << endl;
    }
    std::vector<m_point> initPath = initControls_createAllWayPoints(mainWayPoints, wayPoints_timings);

    cout << "created initPath, length: " << initPath.size() << endl;
    cout << "init path points: " << initPath[498] << endl;
    cout << "init path points: " << initPath[499] << endl;
    cout << "init path points: " << initPath[500] << endl;
    cout << "--------------------------------------------" << endl;
    cout << "init path points: " << initPath[2996] << endl;
    cout << "init path points: " << initPath[2997] << endl;
    cout << "init path points: " << initPath[2998] << endl;

    initControls = initControls_generateAllControls(d, model, initPath);

    cout << "created init controls;" << endl;

    return initControls;
}

void initControls_findMainWaypoints(mjData *d, mjModel *model, m_point desiredObjectEnd, m_point objectStart, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming){
    const std::string EE_Name = "franka_gripper";
    const std::string goalName = "goal";

    int EE_id = mj_name2id(model, mjOBJ_BODY, EE_Name.c_str());
    int goal_id = mj_name2id(model, mjOBJ_BODY, goalName.c_str());

    m_pose startPose = globalMujocoController->returnBodyPose(model, d, EE_id);

    m_point mainWayPoint;
    mainWayPoint(0) = startPose(0);
    mainWayPoint(1) = startPose(1);
    mainWayPoint(2) = startPose(2);
    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(0);

    // TODO hard coded - get it programmatically? - also made it slightly bigger so trajectory has room to improve
    float cylinder_radius = 0.08;
    float x_cylinder0ffset = cylinder_radius * sin(PI/4);
    float y_cylinder0ffset = cylinder_radius * cos(PI/4);

    float desired_endPointX = desiredObjectEnd(0) - x_cylinder0ffset;
    float desired_endPointY;

    float endPointX;
    float endPointY;
    if(desiredObjectEnd(1) - startPose(0) > 0){
        desired_endPointY = desiredObjectEnd(1) - y_cylinder0ffset;
    }
    else{
        desired_endPointY = desiredObjectEnd(1) + y_cylinder0ffset;
    }

    float cylinderObjectX = objectStart(0);
    float cylinderObjectY = objectStart(1);
    float intermediatePointX;
    float intermediatePointY;

    float angle = atan2(desired_endPointY - cylinderObjectY, desired_endPointX - cylinderObjectX);

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

    // Setting this up so we can visualise where the intermediate point is located
    intermediatePoint(0) = intermediatePointX;
    intermediatePoint(1) = intermediatePointY;

    mainWayPoint(0) = intermediatePoint(0);
    mainWayPoint(1) = intermediatePoint(1);

    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(MUJ_STEPS_HORIZON_LENGTH / 6.0f);

    float maxDistTravelled = 0.05 * ((5.0f/6.0f) * MUJ_STEPS_HORIZON_LENGTH * MUJOCO_DT);
    cout << "max EE travel dist: " << maxDistTravelled << endl;
    float desiredDistTravelled = sqrt(pow((desired_endPointX - intermediatePointX),2) + pow((desired_endPointY - intermediatePointY),2));
    float proportionOfDistTravelled = maxDistTravelled / desiredDistTravelled;
    cout << "proportion" << proportionOfDistTravelled << endl;
    if(proportionOfDistTravelled > 1){
        endPointX = desired_endPointX;
        endPointY = desired_endPointY;
    }
    else{
        endPointX = intermediatePointX + ((desired_endPointX - intermediatePointX) * proportionOfDistTravelled);
        endPointY = intermediatePointY + ((desired_endPointY - intermediatePointY) * proportionOfDistTravelled);
    }

    cout << "desired X: " << desired_endPointX << endl;
    cout << "desired Y: " << desired_endPointY << endl;
    cout << "actual X: " << endPointX << endl;
    cout << "actual Y: " << endPointY << endl;
    cout << "inter X: " << intermediatePointX << endl;
    cout << "inter Y: " << intermediatePointY << endl;

    mainWayPoint(0) = endPointX;
    mainWayPoint(1) = endPointY;

    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(MUJ_STEPS_HORIZON_LENGTH - 1 - wayPointsTiming[1]);

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
        for(int j = 0; j < wayPointsTiming[i + 1]; j++){
            initPath.push_back(m_point());
            initPath[counter](0) = initPath[counter - 1](0) + (x_diff / wayPointsTiming[i + 1]);
            initPath[counter](1) = initPath[counter - 1](1) + (y_diff / wayPointsTiming[i + 1]);
            initPath[counter](2) = initPath[0](2);

            counter++;
            if(counter > MUJ_STEPS_HORIZON_LENGTH){
                cout << "ERROR, TOO MANY POINTS IN INIT PATH" << endl;
            }
        }
    }

    return initPath;
}

std::vector<m_ctrl> initControls_generateAllControls(mjData *d, mjModel *model, std::vector<m_point> initPath){
    std::vector<m_ctrl> initControls;

    const std::string EE_Name = "franka_gripper";
    int EE_id = mj_name2id(model, mjOBJ_BODY, EE_Name.c_str());
    taskTranslator tempModelTranslator;
    tempModelTranslator.init(model);
    m_quat startQuat = globalMujocoController->returnBodyQuat(model, d, EE_id);

    m_ctrl desiredControls;

    if(!TORQUE_CONTROL){
        m_state startState = tempModelTranslator.returnState(d);
        for(int i = 0; i < NUM_CTRL; i++){
            desiredControls(i) = startState(i);
        }
    }

    for (int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++) {

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

        tempModelTranslator.setControls(d, initControls[i], false);

        tempModelTranslator.stepModel(d, 1);

//        if (i % 40 == 0) {
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
    if(dist < 0.05){
        taskComplete = true;
    }

    return taskComplete;
}

bool taskTranslator::taskFailed(mjData *d){
    bool taskFailed = false;

    return taskFailed;
}

bool taskTranslator::predictiveStateMismatch(mjData *d, m_state predictedState){
    bool stateMismatch = false;

    double cumError = 0.0f;
    m_state actualState = returnState(d);

    for(int i = 0; i < DOF; i++){
        cumError += abs(actualState(i) - predictedState(i));
    }

    // if cumulative error is greater than some threshold
    if(cumError > 0.1){
        stateMismatch = true;
    }

    return stateMismatch;
}

// Be careful with the math in this function, kind of changed x and Y around due to mujoco x being forwards from robot perspective
bool taskTranslator::newControlInitialisationNeeded(mjData *d){

    // Get End effector position
    const std::string EE_Name = "franka_gripper";
    const std::string goalName = "goal";

    int EE_id = mj_name2id(model, mjOBJ_BODY, EE_Name.c_str());
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
    cout << "angle diff " << angleDiff << "\n";
    if(X_desired(8) < objectPos(1)){
        if(angleDiff > (PI / 10)){
            return true;
        }
    }
    else{
        if(angleDiff < -(PI / 10)){
            return true;
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
