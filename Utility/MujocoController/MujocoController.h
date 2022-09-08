//
// Created by davem on 20/01/2022.
//
#ifndef CLIONS_PROJECTS_MUJOCOCONTROLLER_H
#define CLIONS_PROJECTS_MUJOCOCONTROLLER_H

#include "mujoco.h"
#include "glfw3.h"
#include "../stdInclude/stdInclude.h"


using namespace std;

//#define NUM_JOINTS  7

typedef struct {

    /* Controller gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant */
    float tau;

    /* Output limits */
    float limMin;
    float limMax;

    /* Integrator limits */
    float limMinInt;
    float limMaxInt;

    /* Sample time (in seconds) */
    float T;

    /* Controller "memory" */
    float integrator;
    float prevError;			/* Required for integrator */
    float differentiator;
    float prevMeasurement;		/* Required for differentiator */

    /* Controller output */
    float out;

} PIDController;

enum controlStates{
    simulating,
    ilqrSim,
    staticPos,
    linearInterpolation,
    staticCalc
};

class MujocoController {
public:
    MujocoController();

    mjModel* _model;
    mjData* _data;
    int robotBodyID[11];
    int _resetLevel = 0;
    std::vector<mjData*> _mujocoStates;
    bool firstRand = true;

    bool resetSimFlag = false;

    MatrixXd J_COMi;

    mjModel* returnModel();
    mjData* returnCurrentModelData();

    // Save current mujoco state and add it to a array of mujoco states
    void saveMujocoState();

    // delete last entry in array of mujoco states
    void deleteLastMujocoState();


    void setBodyState(int bodyId, const Ref<const m_pose> pose, const Ref<const m_pose> velocities);

    void setBodyVel(int  bodyId, const Ref<const m_pose> vel);
    void setBodyAccelerations(int bodyId, const Ref<const m_pose> acc);
    void setBodyForces(int bodyId, const Ref<const m_pose> frc);
    m_pose returnBodyState(int bodyId);
    m_pose returnBodyVelocities(int bodyId);
    m_pose returnBodyAcceleration(int bodyId);
    m_pose returnBodyForces(int bodyId);

//    void setRobotConfiguration(const Ref<const m_ctrl> configuration);
//    m_ctrl returnRobotConfiguration();
//    void setRobotVelocities(const Ref<const m_ctrl> jointVelocities);
//    m_ctrl returnRobotVelocities();
//    void setRobotAccelerations(const Ref<const m_ctrl> jointAccelerations);
//    m_ctrl returnRobotAccelerations();


    m_quat axis2Quat(m_point axis);
    m_point quat2Axis(m_quat quaternion);
    m_point quat2Eul(m_quat quaternion);
    m_quat invQuat(m_quat quat);
    m_quat multQuat(m_quat quat_l, m_quat quat_r);

    void set_qPosVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis, double val);
    void set_qVelVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis, double val);
    double return_qPosVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis);
    double return_qVelVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis);
    double return_qAccVal(mjModel *m, mjData *d, int bodyId, bool freeJoint, int freeJntAxis);

    void setBodyPose(mjModel *m, mjData *d, int bodyId, const Ref<const m_pose> pose);
    m_point returnBodyPoint(mjModel *m, mjData *d, int bodyId);
    m_pose returnBodyPose(mjModel *m, mjData *d, int bodyId);
    m_quat returnBodyQuat(mjModel *m, mjData *d, int bodyId);
    void setBodyQuat(mjModel *m, mjData *d, int bodyId, m_quat bodyQuat);

    m_point returnSitePoint(mjModel *m, mjData *d, int bodyId);

    //bool isConfigInCollision();
    int getRobotNumberOfCollisions(mjData *d);

    void saveSimulationState();
    void setResetLevel(int resetLevel);
    void resetSimulation();
    void loadSimulationState(int stateIndex);
    void resetSimulationToStart();

    void step();

    Eigen::MatrixXd calculateJacobian(mjModel *m, mjData *d, int bodyId);
    int returnModelID(const std::string& input);
};

void cpMjData(const mjModel* m, mjData* d_dest, const mjData* d_src);
float randFloat(float a, float b);



#endif //CLIONS_PROJECTS_MUJOCOCONTROLLER_H