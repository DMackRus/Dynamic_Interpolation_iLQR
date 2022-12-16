//
// Created by davem on 14/12/2022.
//

#include "modelTranslator.h"

#ifdef REACHING
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

void taskTranslator::costDerivsControlsAnalytical(mjData *d, Ref<m_ctrl> l_u, Ref<m_ctrl_ctrl> l_uu, m_ctrl lastControl){
    m_ctrl U;
    m_ctrl U_diff;

    U = returnControls(d);
    U_diff = U - lastControl;

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
    bool collisionFreeState = false;
    while(!collisionFreeState){
        for(int i = 0; i < 7; i++){
            float randNum = randFloat(jointLimsMin[i]/2, jointLimsMax[i]/2);
            randState(i) = randNum;
            randState(i + NUM_CTRL) = 0.0f;
        }
        if(!isStateInCollision(d, randState)){
            collisionFreeState = true;
        }
    }

//    randState << 1.19, 0.458, 0.653, -1.82, 0, 0, 0,
//                0, 0, 0, 0, 0, 0, 0;

    return randState;
}

m_state taskTranslator::generateRandomGoalState(m_state startState, mjData *d){
    m_state randState;
    bool collisionFreeState = false;
    float midPoints[NUM_CTRL] = {0, 0, 0, -1.57, 0, 0.26, 0};
    while(!collisionFreeState){
        for(int i = 0; i < 7; i++){
            float randNum = randFloat(0.5, 1.5);

            // add on random stochastic variable
            if(startState(i) < midPoints[i]){
                randState(i) = startState(i) + randNum;
            }
                // subtract
            else{
                randState(i) = startState(i) - randNum;
            }

            //randState(i) = randNum;
            randState(i + NUM_CTRL) = 0.0f;
        }

        if(!isStateInCollision(d, randState)){
            collisionFreeState = true;
        }
    }

//    randState << -1.48, -0.513, -1.19, -1.49, 0, 0.684, 0,
//            0, 0, 0, 0, 0, 0, 0;
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

    // Use a PID Controller
    if(TORQUE_CONTROL){
        m_ctrl lastControl;
        lastControl.setZero();
        int jerkLimit = 1;
        int K[7] = {200, 200, 200, 200, 50, 50, 50};

        for(int i = 0; i <= MUJ_STEPS_HORIZON_LENGTH; i++){

            initControls.push_back(m_ctrl());
            m_state X_diff;
            m_state X = returnState(d);
            m_ctrl nextControl;
            X_diff = X_desired - X;

            for(int k = 0; k < NUM_CTRL; k++){
                nextControl(k) = X_diff(k) * K[k];

                if(nextControl(k) - lastControl(k) > jerkLimit){
                    nextControl(k) = lastControl(k) + jerkLimit;
                }

                if(nextControl(k) - lastControl(k) < -jerkLimit){
                    nextControl(k) = lastControl(k) - jerkLimit;
                }

                if(nextControl(k) > torqueLims[k]) nextControl(k) = torqueLims[k];
                if(nextControl(k) < -torqueLims[k]) nextControl(k) = -torqueLims[k];

                initControls[i](k) = nextControl(k);
                d->ctrl[k] = initControls[i](k);

                lastControl = nextControl.replicate(1,1);

            }

            stepModel(d, 1);

        }
    }
    else{
        m_dof desiredState;
        m_state stateDiff = X_desired - X0;
        m_state currControlState = X0.replicate(1, 1);
        m_ctrl nextControl;


        for(int i = 0; i <= MUJ_STEPS_HORIZON_LENGTH; i++){

            initControls.push_back(m_ctrl());
            currControlState += (stateDiff/MUJ_STEPS_HORIZON_LENGTH);

            for(int k = 0; k < NUM_CTRL; k++){
                nextControl(k) = currControlState(k);

                initControls[i](k) = nextControl(k);

            }

            setControls(d, initControls[i], false);

            stepModel(d, 1);
        }
    }

    return initControls;
}

#endif
