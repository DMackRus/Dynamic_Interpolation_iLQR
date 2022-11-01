//
// Created by david on 13/04/2022.
//

#ifndef MUJOCO_ACROBOT_CONTROL_ILQR_DATACENTRIC_H
#define MUJOCO_ACROBOT_CONTROL_ILQR_DATACENTRIC_H

#include "mujoco.h"
#include "glfw3.h"
#include "../Utility/stdInclude/stdInclude.h"
#include "../modelTranslator/modelTranslator.h"
#include "../Utility/MujocoController/MujocoUI.h"

#define MIN_STEPS_PER_CONTROL 1
#define NUM_SCALING_LEVELS  1

// ------ These three methods of interpolation are implemented, ONLY activate 1 at a time, otherwise probably will crash ---------
#define COPYING_DERIVS          0
#define LINEAR_INTERP_DERIVS    0
#define DYNAMIC_LINEAR_DERIVS   1


// ------- Unimplemented methods of inteprolating derivatives ------
#define QUADRATIC_INTERP_DERIVS 0
#define NN_INTERP_DERIVS        0

// -------- Parameters for the dynamic linear inteprolation methods ----------
#define VEL_GRAD_SENSITIVITY_CUBE       0.0001
#define VEL_GRAD_SENSITIVITY_JOINTS     0.0005
#define MIN_N                           5
#define MAX_N                           50

// previous parameters for dynamic linear inteprolation - 5, 50, 0.0001, 0.0005 (minN, maxN, cube sens, joint sens)

#define DQACCDQ_MAX                     250

class iLQR
{
    public:

    // constructor - mujoco model, data, initial controls and initial state
    iLQR(mjModel* m, mjData* d, frankaModel* _modelTranslator, MujocoController* _mujocoController);

    /*      Data     */
    // MuJoCo model and data
    mjModel* model;
    mjData* mdata = NULL;
    frankaModel *modelTranslator;
    MujocoController *mujocoController;

    // Array of mujoco data structure along the trajectory
    mjData* dArray[MUJ_STEPS_HORIZON_LENGTH + 1];
    // Mujoco data for the initial state of the system
    mjData* d_init;
    m_state X0;

    /**************************************************************************
     *
     *  iLQR Parameters
     *
     *
     */
    float maxLamda = 10000;             // Maximum lambda before canceliing optimisation
    float minLamda = 0.00001;           // Minimum lamda
    float lamdaFactor = 10;             // Lamda multiplicative factor
    float epsConverge = 0.02;          // Satisfactory convergence of cost function
    int maxIterations = 15;

    int scalingLevelCount = 0;
    int scalingLevel[NUM_SCALING_LEVELS] = {1};
    int num_mj_steps_per_control;
    float ilqr_dt;
    int ilqr_horizon_length;

    float avgLinTime;
    float avgNumEvals;
    double avgVariance;
    float finalCost;
    Matrix<double, 2, 1> cubeTermPos;


    float initCost;
    std::vector<m_ctrl> initControls;
    std::vector<m_ctrl> finalControls;
    std::vector<bool> grippersOpen_iLQR;

    // Initialise partial differentiation matrices for all timesteps T
    // for linearised dynamics
    std::vector<m_state_state> f_x;
    std::vector<m_state_ctrl> f_u;

    std::vector<m_state_state> A;
    std::vector<m_state_ctrl> B;

    // Quadratic cost partial derivatives
    std::vector<m_state> l_x;
    std::vector<m_state_state> l_xx;
    std::vector<m_ctrl> l_u;
    std::vector<m_ctrl_ctrl> l_uu;

    // Initialise state feedback gain matrices
    std::vector<m_ctrl> k;
    std::vector<m_ctrl_state> K;

    // Initialise new controls and states storage for evaluation
    std::vector<m_ctrl> U_new;
    std::vector<m_ctrl> U_old;
    std::vector<m_state> X_final;

    std::vector<float> linTimes;
    std::vector<int> numEvals;
    std::vector<double> evalsVariance;
    std::vector<m_state> X_old;

    std::vector<int> evaluationWaypoints;

    float lamda = 0.1;
    int numIterations = 0;
    bool trajecCollisionFree = true;

    void optimise();
    float rollOutTrajectory();

    void generateEvaluationWaypoints();
    bool reEvaluationNeeded(m_dof currentVelGrad, m_dof lastVelGrad);
    void getDerivativesDynamically();
    void getDerivativesStatically();
    void smoothAMatrices();
    void copyDerivatives();
    void linearInterpolateDerivs();
    void quadraticInterpolateDerivs();
    void NNInterpolateDerivs();
    void dynamicLinInterpolateDerivs();

    bool backwardsPass_Quu_reg();
    bool backwardsPass_Vxx_reg();
    bool isMatrixPD(Ref<MatrixXd> matrix);

    float forwardsPass(float oldCost);

    bool checkForConvergence(float newCost, float oldCost);

    bool updateScaling();
    void updateDataStructures();

    void lineariseDynamics(Ref<MatrixXd> _A, Ref<MatrixXd> _B, mjData *linearisedData);

    m_ctrl returnDesiredControl(int controlIndex, bool finalControl);
    void setInitControls(std::vector<m_ctrl> _initControls, std::vector<bool> _gripperOpen);
    void makeDataForOptimisation();
    void deleteMujocoData();
    void updateNumStepsPerDeriv(int stepPerDeriv);
    void resetInitialStates(mjData *_d_init, m_state _X0);


    double calcVariance(std::vector<int> data);

};

#endif //MUJOCO_ACROBOT_CONTROL_ILQR_DATACENTRIC_H
