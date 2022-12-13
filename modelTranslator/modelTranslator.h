//
// Created by david on 03/05/22.
//

#ifndef MUJOCO_ACROBOT_CONTROL_FRANKAARMANDBOX_H
#define MUJOCO_ACROBOT_CONTROL_FRANKAARMANDBOX_H

#include "mujoco.h"
#include "../Utility/stdInclude/stdInclude.h"
#include "../Utility/MujocoController/MujocoController.h"

//#define DOUBLE_PENDULUM 1
//#define REACHING 1
#define OBJECT_PUSHING 1
//#define PUSHING_CLUTTER

#ifdef DOUBLE_PENDULUM
#define DOF 2
#define NUM_CTRL 2
#define PENDULUM_TASK 1
#endif

#ifdef REACHING
#define DOF 7
#define NUM_CTRL 7
#define REACHING_TASK 1
#endif

#ifdef OBJECT_PUSHING
#define DOF 9
#define NUM_CTRL 7
#define OBJECT_PUSHING_TASK 1
#endif

#ifdef PUSHING_CLUTTER
#define DOF 13
#define NUM_CTRL 7
#define PUSHING_CLUTTER_TASK 1
#endif

#ifdef REACHING_CLUTTER
#define DOF 17
#define NUM_CTRL 7
#define REACHING_CLUTTER_TASK 1
#endif

#define GRIPPERS_OPEN   0.04
#define GRIPPERS_CLOSED 0

typedef Matrix<double, NUM_CTRL, 1> m_ctrl;
typedef Matrix<double, DOF, 1> m_dof;

typedef Matrix<double, (2*DOF), 1> m_state;
typedef Matrix<double, (2*DOF), (2*DOF)> m_state_state;
typedef Matrix<double, (2*DOF), NUM_CTRL> m_state_ctrl;
typedef Matrix<double, NUM_CTRL, (2*DOF)> m_ctrl_state;

typedef Matrix<double, NUM_CTRL, NUM_CTRL> m_ctrl_ctrl;
typedef Matrix<double, DOF, DOF> m_dof_dof;
typedef Matrix<double, DOF, NUM_CTRL> m_dof_ctrl;

class frankaModel{
public:
    frankaModel();

    double armControlCosts;
    double armStateCosts;
    double armVelCosts;
    int taskNumber;
    int stateIndexToStateName[DOF];
    int stateIndexToFreeJntIndex[DOF];

#ifdef OBJECT_PUSHING_TASK
    double cubeXPosCost = 1;
    double cubeYPosCost = 1;
    double cubeVelCosts = 0.5;
#endif

    // State vector is: 7 joint angles, two cube pos (X and Y), cube rot, 7 joint velocities, two cube velocities (X and Y)

    double terminalConstant = 10;

    double A = 0.1;
    double sigma = 0.03;
    double alphax = 0.7;
    double alphay = 0.02;

    std::vector<std::string> stateNames;

    float torqueLims[7] = {87, 87, 87, 87, 12, 12, 12};
    float jointLimsMax[7] = {2.97, 1.83, 2.97, 0, 2.97, 2.18, 2.97};
    float jointLimsMin[7] = {-2.97, -1.83, -2.97, -3.14, -2.97, -1.66, -2.97};

    mjModel* model;
    m_state X_desired;
    DiagonalMatrix<double, NUM_CTRL> R;

#ifdef GARBAGE
    DiagonalMatrix<double, (2 * DOF) + 3> Q;
#else
    DiagonalMatrix<double, 2 * DOF> Q;
#endif

    DiagonalMatrix<double, NUM_CTRL> J;

    void init(mjModel *m);
    void setDesiredState(m_state _desiredState);

    //double getCost(mjData *d, m_ctrl lastControl, int controlNum, int totalControls, bool firstControl);
    // Given a set of mujoco data, what is the cost of its state and controls
    double costFunction(mjData *d, int controlNum, int totalControls, m_ctrl lastControl, bool firstControl);

    // Given a set of mujoco data, what are its cost derivates with respect to state and control
    void costDerivatives(mjData *d, Ref<m_state> l_x, Ref<m_state_state> l_xx, Ref<m_ctrl> l_u, Ref<m_ctrl_ctrl> l_uu, int controlNum, int totalControls, m_ctrl lastU);
    void costDerivatives_fd(mjData *d, Ref<m_state> l_x, Ref<m_state_state> l_xx, Ref<m_ctrl> l_u, Ref<m_ctrl_ctrl> l_uu, int controlNum, int totalControls, m_ctrl U_last,  bool firstControl);
    m_ctrl costDerivatives_fd_1stOrder(m_state X, m_ctrl U, m_ctrl U_last, int controlNum, int totalControls, bool firstControl);
    void costDerivsControlsAnalytical(mjData *d, Ref<m_ctrl> l_u, Ref<m_ctrl_ctrl> l_uu, m_ctrl lastControl);

    // set the state of a mujoco data object as per this model
    void setState(mjData *d, m_state X);

    // Return the state of a mujoco data model
    m_state returnState(mjData *d);

    // Set the controls of a mujoco data object
    void setControls(mjData *d, m_ctrl U, bool grippersOpen);

    // Return the controls of a mujoco data object
    m_ctrl returnControls(mjData *d);

    m_dof returnPositions(mjData *d);
    m_dof returnVelocities(mjData *d);
    m_dof returnAccelerations(mjData *d);

    void perturbVelocity(mjData *perturbedData, mjData *origData, int stateIndex, double eps);

    void perturbPosition(mjData *perturbedData, mjData *origData, int stateIndex, double eps);

    void stepModel(mjData *d, int numSteps);

    bool isStateInCollision(mjData *d, m_state state);

    m_state generateRandomStartState(mjData *d);
    m_state generateRandomGoalState(m_state startState, mjData *d);

    double EEDistToGoal(mjData *d, int goalObjectId);
    m_point returnEE_point(mjData *d);

};

#endif //MUJOCO_ACROBOT_CONTROL_FRANKAARMANDBOX_H
