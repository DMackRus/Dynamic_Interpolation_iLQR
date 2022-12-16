//
// Created by davem on 14/12/2022.
//
#include "modelTranslator.h"

extern mjvCamera cam;                   // abstract camera
extern mjvScene scn;                    // abstract scene
extern mjvOption opt;			        // visualization options
extern mjrContext con;				    // custom GPU context
extern GLFWwindow *window;

#ifdef DOUBLE_PENDULUM
// Given a set of mujoco data, what is the cost of its state and controls
double taskTranslator::costFunction(mjData *d, int controlNum, int totalControls, m_ctrl lastControl, bool firstControl){
    double stateCost = 0;
    m_state X = returnState(d);
    m_ctrl U = returnControls(d);
    m_state X_diff;

    VectorXd temp(1);

    // actual - desired
    X_diff = X - X_desired;

    temp = ((X_diff.transpose() * Q * X_diff)) + (U.transpose() * R * U);

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

    l_x = 2 *  Q * X_diff;
    l_xx = 2 * Q;

    l_u = (2 * R * U) + (2 * J * U_diff);
    l_uu = (2 * R) + (2 * J);
}

// set the state of a mujoco data object as per this model
void taskTranslator::setState(mjData *d, m_state X){
    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str() );
        globalMujocoController->set_qPosVal(model, d, bodyId, false, 0, X(i));
        globalMujocoController->set_qVelVal(model, d, bodyId, false, 0, X(i + DOF));
    }
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

    return state;
}

m_dof taskTranslator::returnVelocities(mjData *d){
    m_dof velocities;

    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        velocities(i) = globalMujocoController->return_qVelVal(model, d, bodyId, false, 0);
    }

    return velocities;
}

m_dof taskTranslator::returnAccelerations(mjData *d){
    m_dof accelerations;

    for(int i = 0; i < NUM_CTRL; i++){
        int bodyId = mj_name2id(model, mjOBJ_BODY, stateNames[i].c_str());
        accelerations(i) = globalMujocoController->return_qAccVal(model, d, bodyId, false, 0);
    }

    return accelerations;
}

m_state taskTranslator::generateRandomStartState(mjData *d){
    m_state randState;

    float arm1Pos = randFloat(-2, 2);
    float arm2Pos = randFloat(-2, 2);

    randState << arm1Pos, arm2Pos, 0, 0;


    return randState;
}

m_state taskTranslator::generateRandomGoalState( m_state startState, mjData *d){
    m_state randGoalState;

    float randNum = randFloat(0, 1);
    if(randNum > 0.5){
        // in stable down position
        randGoalState << 3.14, 0, 0, 0;
    }
    else{
        // unstable up position
        randGoalState << 0, 0, 0, 0;
    }


    return randGoalState;
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

    }
    else{
        // Generate start and desired state randomly
        X0 = generateRandomStartState(d);
        //cout << "-------------- random init state ----------------" << endl << X0 << endl;
        X_desired = generateRandomGoalState(X0, d);
        //cout << "-------------- random desired state ----------------" << endl << X_desired << endl;
    }

    setState(d, X0);

    // smooth any random pertubations for starting data
    stepModel(d, 2);

    return X0;
}

std::vector<m_ctrl> taskTranslator::initControls(mjData *d, mjData *d_init, m_state X0){
    std::vector<m_ctrl> initControls;
    cpMjData(model, d, d_init);

    for(int i = 0; i <= MUJ_STEPS_HORIZON_LENGTH; i++){

        initControls.push_back(m_ctrl());

        for(int k = 0; k < NUM_CTRL; k++){

            initControls[i](k) = 0;
            d->ctrl[k] = initControls[i](k);
        }

        stepModel(d, 1);

    }
    return initControls;
}

#endif