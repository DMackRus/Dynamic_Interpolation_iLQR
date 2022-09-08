//
// Created by davem on 20/01/2022.
//
#include "MujocoController.h"

mjModel* model;						// MuJoCo model
mjData* mdata;						// MuJoCo data
mjvCamera cam;						// abstract camera
mjvOption opt;						// visualization options
mjvScene scn;						// abstract scene
mjrContext con;						// custom GPU context
mjvPerturb pert;
GLFWwindow* window;
MujocoController* globalMujocoController;



ofstream saveControlsFile;

std::string saveControlsFilename = "lastControls.csv";

MujocoController::MujocoController(){

    saveControlsFile.open(saveControlsFilename);
}

void MujocoController::step(){
    mj_step(_model, _data);
}

mjModel* MujocoController::returnModel() {
    return _model;
}

void MujocoController::saveMujocoState(){
    mjData *newData = mj_makeData(_model);
    mj_copyData(newData, _model, _data);
    _mujocoStates.push_back(newData);
}

void MujocoController::deleteLastMujocoState(){
    mj_deleteData(_mujocoStates[1]);
    _mujocoStates.pop_back();
}

//void MujocoController::setBodyState(int bodyId, const Ref<const m_pose> pose, const Ref<const m_pose> velocities){
//
//    setBodyPose(bodyId, pose);
//    setBodyVel(bodyId, velocities);
//
//}

//
//void MujocoController::setBodyVel(int  bodyId, const Ref<const m_pose> vel){
//    for(int i = 0; i < 3; i++){
//        _data->qvel[((bodyId - 1) * 6) + i] = vel(i);
//    }
//}
//
//void MujocoController::setBodyAccelerations(int bodyId, const Ref<const m_pose> acc){
//    for(int i = 0; i < 3; i++){
//        _data->qacc[((bodyId) * 6) + i] = acc(i);
//    }
//}
//
//void MujocoController::setBodyForces(int bodyId, const Ref<const m_pose> frc){
//    for(int i = 0; i < 3; i++){
//        _data->xfrc_applied[((bodyId) * 6) + i] = frc(i);
//    }
//}

//m_pose MujocoController::returnBodyState(int bodyId){
//    m_pose bodyPose;
//
//    // TODO extend this to work with rotations also, whether its quaternions or euler angles
//    for(int i = 0; i < 3; i++){
//        bodyPose(i) = _data->qpos[((bodyId - 1) * 7) + i];
//    }
//
//    return bodyPose;
//}
//
//m_pose MujocoController::returnBodyVelocities(int bodyId){
//    m_pose bodyVels;
//
//    for(int i = 0; i < 3; i++){
//        bodyVels(i) = _data->qvel[((bodyId - 1) * 6) + i];
//    }
//
//    return bodyVels;
//}
//
//m_pose MujocoController::returnBodyAcceleration(int bodyId){
//    m_pose bodyAcc;
//
//    for(int i = 0; i < 3; i++){
//        bodyAcc(i) = _data->qacc[((bodyId - 1) * 6) + i];
//    }
//
//    return bodyAcc;
//}
//
//m_pose MujocoController::returnBodyForces(int bodyId){
//    m_pose bodyFrc;
//
//    for(int i = 0; i < 3; i++){
//        bodyFrc(i) = _data->xfrc_applied[((bodyId) * 6) + i];
//    }
//
//    return bodyFrc;
//
//}

//void MujocoController::setRobotConfiguration(const Ref<const m_ctrl> configuration) {
//
//    for (int i = 0; i < NUM_JOINTS; i++) {
//        _data->qpos[i] = configuration(i);
//    }
//    mj_forward(model, _data);
//}
//
//m_ctrl MujocoController::returnRobotConfiguration(){
//    m_ctrl robotConfig;
//
//    for(int i = 0; i < NUM_JOINTS; i++){
//        robotConfig(i) = _data->qpos[i];
//    }
//    return robotConfig;
//}
//
//void MujocoController::setRobotVelocities(const Ref<const m_ctrl> jointVelocities){
//    for (int i = 0; i < NUM_JOINTS; i++) {
//        _data->qvel[i] = jointVelocities(i);
//    }
//}
//
//m_ctrl MujocoController::returnRobotVelocities(){
//    m_ctrl robotVelocities;
//    for(int i = 0; i < NUM_JOINTS; i++){
//        robotVelocities(i) = _data->qvel[i];
//    }
//    return robotVelocities;
//}

//void MujocoController::setRobotAccelerations(const Ref<const m_ctrl> jointAccelerations){
//    for (int i = 0; i < NUM_JOINTS; i++) {
//        _data->qacc[i] = jointAccelerations(i);
//    }
//}
//
//m_ctrl MujocoController::returnRobotAccelerations(){
//    m_ctrl jointAccelerations;
//    for(int i = 0; i < NUM_JOINTS; i++){
//        jointAccelerations(i) = _data->qacc[i];
//    }
//
//
//    return jointAccelerations;
//}
//
//bool MujocoController::isConfigInCollision(m_ctrl configuration) {
//    bool collision = false;
//    int originalResetValue = 0;
//
//    setRobotConfiguration(configuration);
//    mj_step1(_model, _data);
//
//    int numberOfCollisions = getRobotNumberOfCollisions();
//
//    mj_resetData(_model, _data);
//    if (_resetLevel > 0) {
//        mjData* targetData = _mujocoStates.at(_resetLevel - 1);
//        mj_copyData(_data, _model, targetData);
//    }
//
//    //Appears not to need this
//    //mj_forward(m, d);
//
//    if (numberOfCollisions > 0) {
//        collision = true;
//    }
//
//    return collision;
//}

int MujocoController::getRobotNumberOfCollisions(mjData *d) {

    int numContacts = d->ncon;
    int numCollisions = 0;
//    for (int i = 0; i < numContacts; i++) {
//        auto contact = d->contact[i];
//
//        // Get the ids of the two bodies in contacts
//        int bodyInContact1 = _model->body_rootid[_model->geom_bodyid[contact.geom1]];
//        int bodyInContact2 = _model->body_rootid[_model->geom_bodyid[contact.geom2]];
//
//        // only consider it a collision if robot - robot
//        // or robot - table
//
//        bool contact1Robot = false;
//        bool contact1Table = false;
//        bool contact2Robot = false;
//        bool contact2Table = false;
//        for (int j = 0; j < 11; j++) {
//            if (bodyInContact1 == robotBodyID[j]) {
//                contact1Robot = true;
//            }
//
//            if (bodyInContact2 == robotBodyID[j]) {
//                contact2Robot = true;
//            }
//        }
//
//        if (contact1Robot) {
//            if (contact2Robot || contact2Table) {
//                numCollisions++;
//            }
//        }
//        else if(contact2Robot) {
//            if (contact1Robot || contact1Table) {
//                numCollisions++;
//            }
//        }
//    }

    return numContacts;
}

m_quat MujocoController::axis2Quat(m_point axis){
    m_quat quat;
    double x, y, z, w;

    m_point normVec;

    double norm = sqrt(pow(axis(0), 2) + pow(axis(1), 2) + pow(axis(2), 2));
    double angle = norm;
    if(norm > 0.0000001){
        normVec = axis / norm;
    }

    double s = sin(angle/2);

    x = normVec(0) * s;
    y = normVec(1) * s;
    z = normVec(2) * s;

    w = cos(angle/2);

    quat(1) = x;
    quat(2) = y;
    quat(3) = z;
    quat(0) = w;

    return quat;
}

m_point MujocoController::quat2Axis(m_quat quaternion){
    m_point axisAngles;
    double x, y, z, w;

    w = quaternion(0);
    x = quaternion(1);
    y = quaternion(2);
    z = quaternion(3);

    if(w > 1){
        w = 1;
    }

    // angles to rotate about
    double angle = 2 * acos(w);

    double s = sqrt(1-(w * w));
    if(s < 0.001){  // test to see if divide by zero???
        axisAngles(0) = x;
        axisAngles(1) = y;
        axisAngles(2) = z;
    }
    else{
        axisAngles(0) = x / s;
        axisAngles(1) = y / s;
        axisAngles(2) = z / s;
    }
    axisAngles *= angle;

    return axisAngles;
}

// Returns euler angles in ZYX convention (from online calculator) (yaw pitch roll)
m_point MujocoController::quat2Eul(m_quat quaternion){
    m_point eulAngles;

    double roll, pitch, yaw;
    double x, y, z, w;
    double t0, t1, t2, t3, t4;

    x = quaternion(0);
    y = quaternion(1);
    z = quaternion(2);
    w = quaternion(3);

    t0 = 2.0 * (w * x + y * z);
    t1 = 1.0 - 2.0 * (x * x + y * y);
    roll = atan2(t0, t1);

    t2 = 2.0 * (w * y - z * x);
    if(t2 > 1){
        t2 = 1.0;
    }
    if(t2 < -1){
        t2 = -1.0;
    }
    pitch = asin(t2);

    t3 = 2.0 * (w * z + x * y);
    t4 = 1 - 2 * (y * y + z * z);
    yaw = atan2(t3, t4);

    eulAngles(0) = roll;
    eulAngles(1) = pitch;
    eulAngles(2) = yaw;

    return eulAngles;
}

m_quat MujocoController::invQuat(m_quat quat){
    m_quat invQuat;

    invQuat(0) = quat(0);
    invQuat(1) = -quat(1);
    invQuat(2) = -quat(2);
    invQuat(3) = -quat(3);
    return invQuat;
}

m_quat MujocoController::multQuat(m_quat quat_l, m_quat quat_r){
    m_quat result;
    double wr, xr, yr, zr;  // left side quaternion
    double ws, xs, ys, zs;  // right side quaternion

    wr = quat_l(0);
    xr = quat_l(1);
    yr = quat_l(2);
    zr = quat_l(3);

    ws = quat_r(0);
    xs = quat_r(1);
    ys = quat_r(2);
    zs = quat_r(3);

    // new W
    result(0) = (wr * ws) - (xr * xs) - (yr * ys) - (zr * zs);

    // new X
    result(1) = (ws * xr) + (wr * xs) + (yr * zs) - (zr * ys);

    // new Y
    result(2) = (ws * yr) + (wr * ys) + (zr * xs) - (xr * zs);

    // new Z
    result(3) = (ws * zr) + (wr * zs) + (xr * ys) - (yr * xs);

    return result;
}

void MujocoController::set_qPosVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis, double val){
    const int jointIndex = m->body_jntadr[bodyId];
    const int dofIndex = m->jnt_dofadr[jointIndex];
    const int posIndex = m->jnt_qposadr[jointIndex];

    // free joint axis can be any number between 0 and 3 (x, y, z)
    if(freeJntAxis < 0 or freeJntAxis > 3){
        std::cout << "you have used set_qPosVal wrong!!!!!!!!!!! Freejntaxis was: " << freeJntAxis << endl;
    }

    if(!freeJoint){
        // automatically return 1 val
        d->qpos[posIndex] = val;

    }
    else{
        // have to add on freeJntAxis to get desired x y or z component of free joint
        d->qpos[posIndex + freeJntAxis] = val;
    }

}

void MujocoController::set_qVelVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis, double val){
    const int jointIndex = m->body_jntadr[bodyId];
    const int dofIndex = m->jnt_dofadr[jointIndex];

    // free joint axis can be any number between 0 and 5 (x, y, z, roll, pitch, yaw)
    if(freeJntAxis < 0 or freeJntAxis > 5){
        std::cout << "you have used set_qVelVal wrong!!!!!!!!!!! Freejntaxis was: " << freeJntAxis << endl;
    }

    if(!freeJoint){
        // automatically return 1 val
        d->qvel[dofIndex] = val;

    }
    else{
        // have to add on freeJntAxis to get desired x y or z component of free joint
        d->qvel[dofIndex + freeJntAxis] = val;
    }
}

double MujocoController::return_qPosVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis){
    double qPosVal;
    const int jointIndex = m->body_jntadr[bodyId];
    const int dofIndex = m->jnt_dofadr[jointIndex];
    const int posIndex = m->jnt_qposadr[jointIndex];

    // free joint axis can be any number between 0 and 3 (x, y, z)
    if(freeJntAxis < 0 or freeJntAxis > 3){
        std::cout << "you have used return_qPosVal wrong!!!!!!!!!!! Freejntaxis was: " << freeJntAxis << endl;
    }

    if(!freeJoint){
        // automatically return 1 val
        qPosVal = d->qpos[posIndex];

    }
    else{
        // have to add on freeJntAxis to get desired x y or z component of free joint
        qPosVal = d->qpos[posIndex + freeJntAxis];
    }

    return qPosVal;
}

double MujocoController::return_qVelVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis){
    double qVelVal;
    const int jointIndex = m->body_jntadr[bodyId];
    const int dofIndex = m->jnt_dofadr[jointIndex];

    // free joint axis can be any number between 0 and 5 (x, y, z, roll pitch yaw)
    if(freeJntAxis < 0 or freeJntAxis > 5){
        std::cout << "you have used return_qVelVal wrong!!!!!!!!!!! Freejntaxis was: " << freeJntAxis << endl;
    }

    if(!freeJoint){
        // automatically return 1 val
        qVelVal = d->qvel[dofIndex];

    }
    else{
        // have to add on freeJntAxis to get desired x y or z component of free joint
        qVelVal = d->qvel[dofIndex + freeJntAxis];
    }

    return qVelVal;
}

double MujocoController::return_qAccVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis){
    double qAccVal;
    const int jointIndex = m->body_jntadr[bodyId];
    const int dofIndex = m->jnt_dofadr[jointIndex];

    // free joint axis can be any number between 0 and 5 (x, y, z, roll pitch yaw)
    if(freeJntAxis < 0 or freeJntAxis > 5){
        std::cout << "you have used return_qAccVal wrong!!!!!!!!!!! Freejntaxis was: " << freeJntAxis << endl;
    }

    if(!freeJoint){
        // automatically return 1 val
        qAccVal = d->qacc[dofIndex];

    }
    else{
        // have to add on freeJntAxis to get desired x y or z component of free joint
        qAccVal = d->qacc[dofIndex + freeJntAxis];
    }

    return qAccVal;
}

m_point MujocoController::returnBodyPoint(mjModel *m, mjData *d, int bodyId){
    m_point bodyPos;

    for(int i = 0; i < 3; i++){
        bodyPos(i) = d->xpos[(bodyId * 3) + i];
    }

    return bodyPos;
}

m_pose MujocoController::returnBodyPose(mjModel *m, mjData *d, int bodyId){

    m_pose bodyPose;
    m_quat bodyQuat = returnBodyQuat(m, d, bodyId);
    m_point bodyPoint = returnBodyPoint(m, d, bodyId);
    m_point bodyAxisAngles;
    m_point eulAngles;
    //cout << "body quat is: " << bodyQuat << endl;

    eulAngles = quat2Eul(bodyQuat);
    //cout << "body eul is: " << eulAngles << endl;
    bodyAxisAngles = quat2Axis(bodyQuat);
    //cout << "bodyAxisAngles: " << bodyAxisAngles << endl;

    for(int i = 0; i < 3; i++){
        bodyPose(i) = bodyPoint(i);
        bodyPose(i+3) = bodyAxisAngles(i);
    }

    return bodyPose;
}


void MujocoController::setBodyPose(mjModel *m, mjData *d, int bodyId, const Ref<const m_pose> pose){
    const int jointIndex = m->body_jntadr[bodyId];
    const int dofIndex = m->jnt_dofadr[jointIndex];
    const int posIndex = m->jnt_qposadr[jointIndex];

    for(int i = 0; i < 3; i++){
        d->qpos[posIndex + i] = pose(i);
    }
}

m_quat MujocoController::returnBodyQuat(mjModel *m, mjData *d, int bodyId){
    m_quat bodyQuat;

    // TODO SEE  IF FORWARD IS NEEDED HERE OR IF CAN BE FIXED ELSEWHERE IN DATA COPIES
    mj_forward(m, d);
    int jointIndex = m->body_jntadr[bodyId];
    int qposIndex = m->jnt_qposadr[jointIndex];

    for(int i = 0; i < 4; i++){
        bodyQuat(i) = d->xquat[(bodyId * 4) + i];
    }

    return bodyQuat;
}

void MujocoController::setBodyQuat(mjModel *m, mjData *d, int bodyId, m_quat bodyQuat){
    int jointIndex = m->body_jntadr[bodyId];
    int qposIndex = m->jnt_qposadr[jointIndex];

    for(int i = 0; i < 4; i++){
        d->qpos[qposIndex + 3 + i] = bodyQuat(i);
    }
}

m_point MujocoController::returnSitePoint(mjModel *m, mjData *d, int bodyId){
    m_point bodyPos;

    for(int i = 0; i < 3; i++){
        bodyPos(i) = d->site_xpos[(bodyId * 3) + i];
    }

    return bodyPos;
}

void MujocoController::saveSimulationState(){
    mjData *newData = mj_makeData(_model);
    mj_copyData(newData, _model, _data);
    _mujocoStates.push_back(newData);

}

void MujocoController::setResetLevel(int resetLevel){
    _resetLevel = resetLevel;
}

void MujocoController::resetSimulation(){
    mj_resetData(_model, _data);

    if (_resetLevel > 0) {
        mjData *targetData = _mujocoStates.at(_resetLevel);
        mj_copyData(_data, _model, targetData);
    }

    mj_forward(_model, _data);
}

void MujocoController::loadSimulationState(int stateIndex){

    if(0){
        mj_copyData(_data, _model, _mujocoStates[stateIndex]);
    }
    else{
        _data->time = _mujocoStates[stateIndex]->time;
        mju_copy(_data->qpos, _mujocoStates[stateIndex]->qpos, _model->nq);
        mju_copy(_data->qvel, _mujocoStates[stateIndex]->qvel, _model->nv);
        mju_copy(_data->act,  _mujocoStates[stateIndex]->act,  _model->na);

        // copy mocap body pose and userdata
        mju_copy(_data->mocap_pos,  _mujocoStates[stateIndex]->mocap_pos,  3*_model->nmocap);
        mju_copy(_data->mocap_quat, _mujocoStates[stateIndex]->mocap_quat, 4*_model->nmocap);
        mju_copy(_data->userdata, _mujocoStates[stateIndex]->userdata, _model->nuserdata);

        // copy warm-start acceleration
        mju_copy(_data->qacc_warmstart, _mujocoStates[stateIndex]->qacc_warmstart, _model->nv);
    }
}

Eigen::MatrixXd MujocoController::calculateJacobian(mjModel *m, mjData *d, int bodyId){
    Eigen::MatrixXd kinematicJacobian(6, 7);

    //mjtNum* J_COMi_temp = mj_stackAlloc(_data, 3*_model->nv);
    Matrix<double, Dynamic, Dynamic, RowMajor> J_p(3, m->nv);
    Matrix<double, Dynamic, Dynamic, RowMajor> J_r(3, m->nv);

    mj_jacBody(m, d, J_p.data(), J_r.data(), bodyId);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            kinematicJacobian(i, j) = J_p(i, j);
            //cout << kinematicJacobian(i, j) << endl;
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            kinematicJacobian(i + 3, j) = J_p(i, j);
        }
    }

    return kinematicJacobian;
}

int MujocoController::returnModelID(const std::string& input){
    return(mj_name2id(model, mjOBJ_BODY, input.c_str()));
}

void cpMjData(const mjModel* m, mjData* d_dest, const mjData* d_src){
    d_dest->time = d_src->time;
    mju_copy(d_dest->qpos, d_src->qpos, m->nq);
    mju_copy(d_dest->qvel, d_src->qvel, m->nv);
    mju_copy(d_dest->qacc, d_src->qacc, m->nv);
    mju_copy(d_dest->qacc_warmstart, d_src->qacc_warmstart, m->nv);
    mju_copy(d_dest->qfrc_applied, d_src->qfrc_applied, m->nv);
    mju_copy(d_dest->xfrc_applied, d_src->xfrc_applied, 6*m->nbody);
    mju_copy(d_dest->ctrl, d_src->ctrl, m->nu);
}

float randFloat(float a, float b) {
    if(globalMujocoController->firstRand){
        srand(time(0));
        globalMujocoController->firstRand = false;
    }
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}