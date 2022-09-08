//
// Created by david on 13/04/2022.
//

#include "iLQR_dataCentric.h"

iLQR* optimiser;
extern mjvCamera cam;                   // abstract camera
extern mjvScene scn;                    // abstract scene
extern mjvOption opt;			        // visualization options
extern mjrContext con;				    // custom GPU context
extern GLFWwindow *window;

iLQR::iLQR(mjModel* m, mjData* d, frankaModel* _modelTranslator, MujocoController* _mujocoController){
    numIterations = 0;
    lamda = 0.00001;

    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
        A.push_back(m_state_state());
        B.push_back(m_state_ctrl());

        A[i].block(0, 0, DOF, DOF).setIdentity();
        A[i].block(0, DOF, DOF, DOF).setIdentity();
        A[i].block(0, DOF, DOF, DOF) *= MUJOCO_DT;
        B[i].setZero();

    }

    for(int j = 0; j < MUJ_STEPS_HORIZON_LENGTH; j++) {
        f_x.push_back(m_state_state());
        f_u.push_back(m_state_ctrl());

        l_x.push_back(m_state());
        l_xx.push_back(m_state_state());
        l_u.push_back(m_ctrl());
        l_uu.push_back(m_ctrl_ctrl());

        k.push_back(m_ctrl());
        K.push_back(m_ctrl_state());

        U_new.push_back(m_ctrl());
        U_old.push_back(m_ctrl());

        initControls.push_back(m_ctrl());
        finalControls.push_back(m_ctrl());
        grippersOpen_iLQR.push_back(bool());

        X_final.push_back(m_state());
        X_old.push_back(m_state());
    }

    // Extra as one more state than controls
    l_x.push_back(m_state());
    l_xx.push_back(m_state_state());


    // Initialise internal iLQR model and data
    model = m;
    mujocoController = _mujocoController;
    modelTranslator = _modelTranslator;
    mdata = mj_makeData(model);
    cpMjData(model, mdata, d);

    d_init = mj_makeData(model);

}

void iLQR::optimise(){
    bool optimisationFinished = false;
    float newCost = 0;
    float oldCost = 1000;
    auto optstart = high_resolution_clock::now();

    oldCost = rollOutTrajectory();
    cout << "initial Trajectory cost: " << oldCost << endl;
    cout << "---------------------------------------------------- " << endl;

    // iterate until optimisation finished, convergence or if lamda > maxLamda
    for(int i = 0; i < maxIterations; i++){

        auto start = high_resolution_clock::now();

        // Linearise the dynamics and save cost values at each state
        // STEP 1 - Linearise dynamics and calculate cost quadratics at every time step

        if(!DYNAMIC_LINEAR_DERIVS){
            getDerivativesStatically();
        }
        else{
            generateEvaluationWaypoints();
            getDerivativesDynamically();
            dynamicLinInterpolateDerivs();
            evaluationWaypoints.clear();
        }

        if(COPYING_DERIVS){
            copyDerivatives();
        }
        else if(LINEAR_INTERP_DERIVS){
            linearInterpolateDerivs();
        }
        else if(QUADRATIC_INTERP_DERIVS){
            quadraticInterpolateDerivs();
        }
        else if(NN_INTERP_DERIVS){
            NNInterpolateDerivs();
        }
        else if(DYNAMIC_LINEAR_DERIVS){

        }
        else{
            // Error ocurred, no valid method for caluclaitng intermediate derivatives
            std::cout << "error, no valid method for calculating intermediate derivatives" << std::endl;
        }

        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        cout << "Linearising model: " << duration.count()/1000 << " milliseconds" << endl;
        linTimes.push_back(duration.count()/1000);

        bool validBackPass = false;
        bool lamdaExit = false;

        // Until a valid backwards pass was calculated with no PD Q_uu_reg matrices
        while(!validBackPass) {

            // STEP 2 - Backwards pass to compute optimal linear and feedback gain matrices k and K
            auto bpStart = high_resolution_clock::now();
            validBackPass = backwardsPass_Quu_reg();
            auto bpstop = high_resolution_clock::now();
            auto bpduration = duration_cast<microseconds>(bpstop - bpStart);
            cout << "backwards pass: " << bpduration.count()/1000 << " milliseconds" << endl;


            if (!validBackPass) {
                if (lamda < maxLamda) {
                    lamda *= lamdaFactor;
                } else {
                    lamdaExit = true;
                    optimisationFinished = true;
                    break;
                }
            } else {
                if (lamda > minLamda) {
                    lamda /= lamdaFactor;
                }
            }
        }


        if(!lamdaExit){
            // STEP 3 - Forwards pass to calculate new optimal controls - with optional alpha backtracking line search
            auto fdStart = high_resolution_clock::now();
            newCost = forwardsPass(oldCost);
            auto fdstop = high_resolution_clock::now();
            auto fdduration = duration_cast<microseconds>(fdstop - fdStart);
            cout << "forward pass: " << fdduration.count()/1000 << " milliseconds" << endl;
            // STEP 4 - Check for convergence
            bool currentStepsConverged = checkForConvergence(newCost, oldCost);
            if(currentStepsConverged){
                //optimisationFinished = updateScaling();
                break;
            }
//            if(optimisationFinished){
//                break;
//            }

            oldCost = newCost;
            finalCost = newCost;
        }
        else{
            cout << "optimisation exited after lamda exceed lamda max, iteration: " << i << endl;
            break;
        }
    }

    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
        finalControls[i] = U_new[i].replicate(1, 1);
    }

    auto optstop = high_resolution_clock::now();
    auto optduration = duration_cast<microseconds>(optstop - optstart);
    cout << "Optimisation took: " << optduration.count()/1000 << " milliseconds" << endl;
}

float iLQR::rollOutTrajectory(){
    float cost = 0;

    cpMjData(model, mdata, d_init);
    X_old[0] = modelTranslator->returnState(mdata);
    cpMjData(model, dArray[0], mdata);
    int stepsCounter = num_mj_steps_per_control;
    int dArrayCounter = 1;

    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
        modelTranslator->setControls(mdata, U_old[i], grippersOpen_iLQR[i]);
        float stateCost;
        if(i == 0){
            stateCost = modelTranslator->costFunction(mdata, i, MUJ_STEPS_HORIZON_LENGTH, U_old[0], true);
        }
        else{
            stateCost = modelTranslator->costFunction(mdata, i, MUJ_STEPS_HORIZON_LENGTH, U_old[i-1], false);
        }

        cost += (stateCost * MUJOCO_DT);
        modelTranslator->stepModel(mdata, 1);
        X_old[i + 1] = modelTranslator->returnState(mdata);

        cpMjData(model, dArray[dArrayCounter], mdata);
        dArrayCounter++;

//        stepsCounter--;
//        if(stepsCounter == 0){
//            cpMjData(model, dArray[dArrayCounter], mdata);
//            //mj_forward(model, dArray[dArrayCounter]);
//            dArrayCounter++;
//            stepsCounter = num_mj_steps_per_control;
//        }

//        if( i % 20 == 0){
//            mjrRect viewport = { 0, 0, 0, 0 };
//            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
//
//            // update scene and render
//            mjv_updateScene(model, mdata, &opt, NULL, &cam, mjCAT_ALL, &scn);
//            mjr_render(viewport, &scn, &con);
//
//            // swap OpenGL buffers (blocking call due to v-sync)
//            glfwSwapBuffers(window);
//
//            // process pending GUI events, call GLFW callbacks
//            glfwPollEvents();
//        }

    }

    m_state termState = modelTranslator->returnState(dArray[ilqr_horizon_length]);
    if(modelTranslator->taskNumber == 2){
        double cubeXDiff = termState(7) - modelTranslator->X_desired(7);
        double cubeYDiff = termState(8) - modelTranslator->X_desired(8);
        cout << "final cube pos, x diff: " << cubeXDiff << ", y: " << cubeYDiff << endl;
    }
    cout << "--------------------------------------------------" << endl;
    cpMjData(model, mdata, d_init);

    initCost = cost;
    return cost;
}

bool iLQR::reEvaluationNeeded(m_dof currentVelGrad, m_dof lastVelGrad){

    for(int i = 0; i < DOF; i++){
        double velGradDiff = currentVelGrad(i) - lastVelGrad(i);
        double requiredSensitivity;
        if(i >= DOF - 2){
            requiredSensitivity = VEL_GRAD_SENSITIVITY_CUBE;
        }
        else{
            requiredSensitivity = VEL_GRAD_SENSITIVITY_JOINTS;
        }

        if(velGradDiff > requiredSensitivity){
            return true;
        }
        if(velGradDiff < -requiredSensitivity){
            return true;
        }
    }

    return false;
}

void iLQR::generateEvaluationWaypoints(){
    int counterSinceLastEval = 0;
    bool first = true;

    m_dof currentVelGrads;
    m_dof lastVelGrads;
    evaluationWaypoints.push_back(0);

    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){

        m_state currState = X_old[i].replicate(1,1);
        m_state lastState = X_old[i-1].replicate(1,1);

        for(int j = 0; j < DOF; j++){
            currentVelGrads(j) = currState(j + DOF) - lastState(j + DOF);
        }
        //cout << currentVelGrads(8) << endl;

        if(counterSinceLastEval >= MIN_N){
            if(first){
                first = false;
                counterSinceLastEval = 0;
                evaluationWaypoints.push_back(i);

            }
            else{
                if(reEvaluationNeeded(currentVelGrads, lastVelGrads)){
                    evaluationWaypoints.push_back(i);
                    counterSinceLastEval = 0;
                }
            }

            lastVelGrads = currentVelGrads.replicate(1, 1);
        }

        if(counterSinceLastEval >= MAX_N){
            evaluationWaypoints.push_back(i);
            counterSinceLastEval = 0;
        }

        counterSinceLastEval++;
    }

    evaluationWaypoints.push_back(MUJ_STEPS_HORIZON_LENGTH - 1);


    cout << "num evaluation points: " << evaluationWaypoints.size() << endl;

    std::vector<int> distBetweenIndices;

    for(int i = 0; i < evaluationWaypoints.size() - 1; i++){

        distBetweenIndices.push_back(evaluationWaypoints[i+1] - evaluationWaypoints[i]);

    }
    double variance;

    variance = calcVariance(distBetweenIndices);

    evalsVariance.push_back(variance);


}

double iLQR::calcVariance(std::vector<int> data){

    int sumMean = 0;
    double mean = 0.0f;

    for(int i = 0; i < data.size(); i++){



        sumMean += data[i];

    }

    mean = sumMean / data.size();

    double sumVariance = 0.0f;
    double variance = 0.0f;

    for(int i = 0; i < data.size(); i++){
        sumVariance += pow((data[i] -  mean), 2);
    }

    variance = sumVariance / data.size();

    return variance;

}

void iLQR::getDerivativesDynamically(){
    cout << "num waypoints is: " << evaluationWaypoints.size() << endl;
    numEvals.push_back(evaluationWaypoints.size());
//    for(int i = 0; i < evaluationWaypoints.size(); i++){
//        cout << evaluationWaypoints[i] << " ";
//    }
//    cout << endl;

    int save_iterations = model->opt.iterations;
    mjtNum save_tolerance = model->opt.tolerance;
    model->opt.iterations = 30;
    model->opt.tolerance = 0;

    #pragma omp parallel for default(none)
    for(int t = 0; t < evaluationWaypoints.size(); t++){

        int index = evaluationWaypoints[t];

        lineariseDynamics(A[t], B[t], dArray[index]);

    }

    model->opt.iterations = save_iterations;
    model->opt.tolerance = save_tolerance;

    #pragma omp parallel for default(none)
    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
        m_ctrl lastControl;
        if(i == 0){
            lastControl.setZero();
        }
        else{
            lastControl = modelTranslator->returnControls(dArray[i - 1]);
        }

        modelTranslator->costDerivatives(dArray[i], l_x[i], l_xx[i], l_u[i], l_uu[i], i, MUJ_STEPS_HORIZON_LENGTH, lastControl);
    }

    l_xx[MUJ_STEPS_HORIZON_LENGTH] = l_xx[MUJ_STEPS_HORIZON_LENGTH - 1].replicate(1,1);
    l_x[MUJ_STEPS_HORIZON_LENGTH] = l_x[MUJ_STEPS_HORIZON_LENGTH - 1].replicate(1,1);
}

void iLQR::getDerivativesStatically(){

    int save_iterations = model->opt.iterations;
    mjtNum save_tolerance = model->opt.tolerance;

    model->opt.iterations = 30;
    model->opt.tolerance = 0;

    auto iLQRStart = high_resolution_clock::now();

    // Linearise the dynamics along the trajectory
    #pragma omp parallel for default(none)
    for(int t = 0; t < ilqr_horizon_length; t++){

        lineariseDynamics(A[t], B[t], dArray[t * num_mj_steps_per_control]);

    }

    #pragma omp parallel for default(none)
    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
        m_ctrl lastControl;
        if(i == 0){
            lastControl.setZero();
        }
        else{
            lastControl = modelTranslator->returnControls(dArray[i - 1]);
        }

        modelTranslator->costDerivatives(dArray[i], l_x[i], l_xx[i], l_u[i], l_uu[i], i, MUJ_STEPS_HORIZON_LENGTH, lastControl);
    }

    l_x[MUJ_STEPS_HORIZON_LENGTH] = l_x[MUJ_STEPS_HORIZON_LENGTH - 1].replicate(1,1);
    l_xx[MUJ_STEPS_HORIZON_LENGTH] = l_xx[MUJ_STEPS_HORIZON_LENGTH - 1].replicate(1,1);

    model->opt.iterations = save_iterations;
    model->opt.tolerance = save_tolerance;

    // Pushing task
    if(modelTranslator->taskNumber == 2){
        smoothAMatrices();
    }


//    for(int i = 0; i < 3; i++){
//        cout << "A " << A[i] << endl;
//        m_state test = modelTranslator->returnState(dArray[i]);

//    }

    auto iLQRStop = high_resolution_clock::now();
    auto iLQRDur = duration_cast<microseconds>(iLQRStop - iLQRStart);
    cout << "time taken to get derivatives " << iLQRDur.count() / 1000 << endl;

}

void iLQR::smoothAMatrices(){

    //cout << "begin smoothing matrices" << endl;
    // loop through bottom left part of matrix and calculate mean + standard deviation, then remove vals appropriately
    for(int i = 0; i < DOF; i++){
        for(int j = 0; j < DOF; j++){
            double sum = 0;

            // calculate the mean
            for(int k = 0; k < ilqr_horizon_length; k++){
                sum += A[k](i + DOF, j);
            }

            double mean = sum / ilqr_horizon_length;
            double sumStdDev;

            // calcuilate the standard deviation
            for(int k = 0; k < ilqr_horizon_length; k++){
                sumStdDev += pow(A[k](i + DOF, j) - mean, 2);
            }

            double stdDev = sqrt(sumStdDev / ilqr_horizon_length);

            //cout << "mean is: " << mean << endl;
            //cout << "std dev is: " << stdDev << endl;

            // any values outside 2 standard deviations, set them to the mean
            for(int k = 0; k < ilqr_horizon_length; k++){
                if(A[k](i + DOF, j) - mean > (0.5 * stdDev)){
                    //cout << "val outside 2 standard deviations: " << A[k](i + DOF, j) << endl;
                    A[k](i + DOF, j) = mean;

                }

                if(A[k](i + DOF, j) - mean < -(0.5 * stdDev)){
                    //cout << "val outside 2 standard deviations: " << A[k](i + DOF, j) << endl;
                    A[k](i + DOF, j) = mean;
                }
            }
        }
    }

    for(int i = 0; i < ilqr_horizon_length; i++){


    }
}


void iLQR::copyDerivatives(){

    for(int t = 0; t < ilqr_horizon_length; t++){
        for(int i = 0; i < num_mj_steps_per_control; i++){
            f_x[(t * num_mj_steps_per_control) + i] = A[t].replicate(1,1);
            f_u[(t * num_mj_steps_per_control) + i] = B[t].replicate(1,1);
        }
    }

//    for(int t = 0; t < ilqr_horizon_length; t++){
//        for(int i = 0; i < num_mj_steps_per_control; i++){
//            l_x.at((t * num_mj_steps_per_control) + i)  = l_x_o[t].replicate(1,1) * MUJOCO_DT;
//            l_xx.at((t * num_mj_steps_per_control) + i) = l_xx_o[t].replicate(1,1) * MUJOCO_DT;
//            l_u.at((t * num_mj_steps_per_control) + i)  = l_u_o[t].replicate(1,1) * MUJOCO_DT;
//            l_uu.at((t * num_mj_steps_per_control) + i) = l_uu_o[t].replicate(1,1) * MUJOCO_DT;
//        }
//    }
//
//    for(int i = 0; i < num_mj_steps_per_control; i++){
//        l_x[(ilqr_horizon_length * num_mj_steps_per_control) + i]  = l_x_o[ilqr_horizon_length] * MUJOCO_DT;
//        l_xx[(ilqr_horizon_length * num_mj_steps_per_control)  + i] = l_xx_o[ilqr_horizon_length] * MUJOCO_DT;
//    }

//        cout << "l_x end: " << l_x[MUJ_STEPS_HORIZON_LENGTH] << endl;
//        cout << "l_xx end: " << l_xx[MUJ_STEPS_HORIZON_LENGTH] << endl;
//        cout << "l_u end: " << l_u[MUJ_STEPS_HORIZON_LENGTH - 1] << endl;
//        cout << "l_uu end: " << l_uu[MUJ_STEPS_HORIZON_LENGTH - 1] << endl;
//
//        cout << "f_u end: " << f_u[MUJ_STEPS_HORIZON_LENGTH - 1] << endl;
//        cout << "f_x end: " << f_x[MUJ_STEPS_HORIZON_LENGTH - 1] << endl;
}

void iLQR::linearInterpolateDerivs(){
    for(int t = 0; t < ilqr_horizon_length; t++){

        m_state_state addA;
        m_state_ctrl addB;

        if(t != ilqr_horizon_length - 1){
            m_state_state startA = A[t].replicate(1, 1);
            m_state_state endA = A[t + 1].replicate(1, 1);
            m_state_state diffA = endA - startA;
            addA = diffA / num_mj_steps_per_control;

            m_state_ctrl startB = B[t].replicate(1, 1);
            m_state_ctrl endB = B[t + 1].replicate(1, 1);
            m_state_ctrl diffB = endB - startB;
            addB = diffB / num_mj_steps_per_control;
        }
        else{
            addA.setZero();
            addB.setZero();
        }

//            cout << "start A " << endl << startA << endl;
//            cout << "endA A " << endl << endA << endl;
//            cout << "diff A " << endl << diff << endl;
//            cout << "add A " << endl << add << endl;

        for(int i = 0; i < num_mj_steps_per_control; i++){
            f_x[(t * num_mj_steps_per_control) + i] = A[t].replicate(1,1) + (addA * i);
            f_u[(t * num_mj_steps_per_control) + i] = B[t].replicate(1,1) + (addB * i);
            //cout << "f_x " << endl << f_x[(t * num_mj_steps_per_control) + i] << endl;
        }
    }

}

void iLQR::quadraticInterpolateDerivs(){

}

void iLQR::NNInterpolateDerivs(){

}

void iLQR::dynamicLinInterpolateDerivs() {

    for(int t = 0; t < evaluationWaypoints.size()-1; t++){

        m_state_state addA;
        m_state_ctrl addB;
        m_state add_l_x;
        m_state_state add_l_xx;
        int nextInterpolationSize = evaluationWaypoints[t+1] - evaluationWaypoints[t];
        int startIndex = evaluationWaypoints[t];

        m_state_state startA = A[t].replicate(1, 1);
        m_state_state endA = A[t + 1].replicate(1, 1);
        m_state_state diffA = endA - startA;
        addA = diffA / nextInterpolationSize;

        m_state_ctrl startB = B[t].replicate(1, 1);
        m_state_ctrl endB = B[t + 1].replicate(1, 1);
        m_state_ctrl diffB = endB - startB;
        addB = diffB / nextInterpolationSize;


        // Interpolate A and B matrices
        for(int i = 0; i < nextInterpolationSize; i++){
            f_x[startIndex + i] = A[t].replicate(1,1) + (addA * i);
            f_u[startIndex + i] = B[t].replicate(1,1) + (addB * i);


            //cout << "index: " << startIndex + i << endl;
            //cout << f_x[startIndex + i] << endl;

        }
//            cout << "start A " << endl << startA << endl;
//            cout << "endA A " << endl << endA << endl;
//            cout << "diff A " << endl << diff << endl;
//            cout << "add A " << endl << add << endl;
    }

    f_x[MUJ_STEPS_HORIZON_LENGTH - 1] = f_x[MUJ_STEPS_HORIZON_LENGTH - 2].replicate(1,1);
    f_u[MUJ_STEPS_HORIZON_LENGTH - 1] = f_u[MUJ_STEPS_HORIZON_LENGTH - 2].replicate(1,1);

//    l_x[MUJ_STEPS_HORIZON_LENGTH - 1] = l_x[MUJ_STEPS_HORIZON_LENGTH - 2].replicate(1,1);
//    l_xx[MUJ_STEPS_HORIZON_LENGTH - 1] = l_xx[MUJ_STEPS_HORIZON_LENGTH - 2].replicate(1,1);
//
//    l_u[MUJ_STEPS_HORIZON_LENGTH - 1] = l_u[MUJ_STEPS_HORIZON_LENGTH - 2].replicate(1,1);
//    l_uu[MUJ_STEPS_HORIZON_LENGTH - 1] = l_uu[MUJ_STEPS_HORIZON_LENGTH - 2].replicate(1,1);
//
//    l_x[MUJ_STEPS_HORIZON_LENGTH]  = l_x[MUJ_STEPS_HORIZON_LENGTH - 1];
//    l_xx[MUJ_STEPS_HORIZON_LENGTH]  = l_xx[MUJ_STEPS_HORIZON_LENGTH - 1];

//    cout << "f_x end " << f_x[MUJ_STEPS_HORIZON_LENGTH - 2] << endl;
//    cout << "f_u end " << f_u[MUJ_STEPS_HORIZON_LENGTH - 2] << endl;
//    cout << "l_x end " << l_x[MUJ_STEPS_HORIZON_LENGTH - 2] << endl;
//    cout << "l_u end " << l_u[MUJ_STEPS_HORIZON_LENGTH - 2] << endl;
//
//    cout << "l_xx end " << l_xx[MUJ_STEPS_HORIZON_LENGTH - 2] << endl;
//    cout << "l_uu end " << l_uu[MUJ_STEPS_HORIZON_LENGTH - 2] << endl;
//
//    cout << "-------------------------------------------------" << endl;
//
//    cout << "f_x end " << f_x[MUJ_STEPS_HORIZON_LENGTH - 1] << endl;
//    cout << "f_u end " << f_u[MUJ_STEPS_HORIZON_LENGTH - 1] << endl;
//    cout << "l_x end " << l_x[MUJ_STEPS_HORIZON_LENGTH - 1] << endl;
//    cout << "l_u end " << l_u[MUJ_STEPS_HORIZON_LENGTH - 1] << endl;
//
//    cout << "l_xx end " << l_xx[MUJ_STEPS_HORIZON_LENGTH - 1] << endl;
//    cout << "l_uu end " << l_uu[MUJ_STEPS_HORIZON_LENGTH - 1] << endl;
}

bool iLQR::backwardsPass_Quu_reg(){
    m_state V_x;
    V_x = l_x[MUJ_STEPS_HORIZON_LENGTH];
    m_state_state V_xx;
    V_xx = l_xx[MUJ_STEPS_HORIZON_LENGTH];
    int Quu_pd_check_counter = 0;
    int number_steps_between_pd_checks = 100;
    float time = 0.0f;

    for(int t = MUJ_STEPS_HORIZON_LENGTH - 1; t > -1; t--){
        m_state Q_x;
        m_ctrl Q_u;
        m_state_state Q_xx;
        m_ctrl_ctrl Q_uu;
        m_ctrl_state Q_ux;

        Quu_pd_check_counter++;



        Q_u = l_u[t] + (f_u[t].transpose() * V_x);

        Q_x = l_x[t] + (f_x[t].transpose() * V_x);

        Q_ux = (f_u[t].transpose() * (V_xx * f_x[t]));

        Q_uu = l_uu[t] + (f_u[t].transpose() * (V_xx * f_u[t]));

        Q_xx = l_xx[t] + (f_x[t].transpose() * (V_xx * f_x[t]));



        m_ctrl_ctrl Q_uu_reg = Q_uu.replicate(1, 1);

        for(int i = 0; i < NUM_CTRL; i++){
            Q_uu_reg(i, i) += lamda;
        }

        if(Quu_pd_check_counter >= number_steps_between_pd_checks){
            if(!isMatrixPD(Q_uu_reg)){
                cout << "iteration " << t << endl;
                cout << "f_x[t - 3] " << f_x[t - 3] << endl;
                cout << "f_x[t - 2] " << f_x[t - 2] << endl;
                cout << "f_x[t - 1] " << f_x[t - 1] << endl;
                cout << "f_x[t] " << f_x[t] << endl;
                cout << "Q_uu_reg " << Q_uu_reg << endl;
                return false;
            }
            Quu_pd_check_counter = 0;
        }

        auto temp = (Q_uu_reg).ldlt();
        m_ctrl_ctrl I;
        I.setIdentity();
        m_ctrl_ctrl Q_uu_inv = temp.solve(I);

        k[t] = -Q_uu_inv * Q_u;
        K[t] = -Q_uu_inv * Q_ux;

        V_x = Q_x + (K[t].transpose() * (Q_uu * k[t])) + (K[t].transpose() * Q_u) + (Q_ux.transpose() * k[t]);
        V_xx = Q_xx + (K[t].transpose() * (Q_uu * K[t])) + (K[t].transpose() * Q_ux) + (Q_ux.transpose() * K[t]);

        V_xx = (V_xx + V_xx.transpose()) / 2;

//        cout << "l_x " << l_x[t] << endl;
//        cout << "l_xx " << l_xx[t] << endl;
//        cout << "Q_ux " << Q_ux << endl;
//        cout << "f_u[t] " << f_u[t] << endl;
//        cout << "Q_uu " << Q_uu << endl;
//        cout << "Q_uu_inv " << Q_uu_inv << endl;
//        cout << "V_xx " << V_xx << endl;
//        cout << "V_x " << V_x << endl;
//        cout << "K[t] " << K[t] << endl;
    }

    return true;
}

bool iLQR::backwardsPass_Vxx_reg(){
    m_state V_x;
    V_x = l_x[MUJ_STEPS_HORIZON_LENGTH];
    m_state_state V_xx;
    V_xx = l_xx[MUJ_STEPS_HORIZON_LENGTH];

    for(int t = MUJ_STEPS_HORIZON_LENGTH - 1; t > -1; t--){
        m_state Q_x;
        m_ctrl Q_u;
        m_state_state Q_xx;
        m_ctrl_ctrl Q_uu;
        m_ctrl_state Q_ux;
        m_state_state V_xx_reg;

        V_xx_reg = V_xx.replicate(1, 1);
        for(int i = 0; i < (2 * DOF); i++){
            V_xx_reg(i, i) += lamda;
        }

        Q_x = l_x[t] + (f_x[t].transpose() * V_x);

        Q_u = l_u[t] + (f_u[t].transpose() * V_x);

        Q_xx = l_xx[t] + (f_x[t].transpose() * (V_xx * f_x[t]));

        Q_uu = l_uu[t] + (f_u[t].transpose() * (V_xx * f_u[t]));

        Q_ux = (f_u[t].transpose() * (V_xx * f_x[t]));

        m_ctrl_ctrl Q_uu_reg;
        m_ctrl_state Q_ux_reg;

        Q_uu_reg = l_uu[t] + (f_u[t].transpose() * (V_xx_reg * f_u[t]));

        Q_ux_reg = (f_u[t].transpose() * (V_xx_reg * f_x[t]));

        if(!isMatrixPD(Q_uu_reg)){
            cout << "iteration " << t << endl;
            cout << "f_x[t] " << f_x[t] << endl;
            cout << "Q_uu_reg " << Q_uu_reg << endl;
            return false;
        }

        auto temp = (Q_uu_reg).ldlt();
        m_ctrl_ctrl I;
        I.setIdentity();
        m_ctrl_ctrl Q_uu_inv = temp.solve(I);

        k[t] = -Q_uu_inv * Q_u;
        K[t] = -Q_uu_inv * Q_ux_reg;

        V_x = Q_x + (K[t].transpose() * (Q_uu * k[t])) + (K[t].transpose() * Q_u) + (Q_ux.transpose() * k[t]);

        V_xx = Q_xx + (K[t].transpose() * (Q_uu * K[t])) + (K[t].transpose() * Q_ux) + (Q_ux.transpose() * K[t]);

        V_xx = (V_xx + V_xx.transpose()) / 2;

    }

    return true;
}

bool iLQR::isMatrixPD(Ref<MatrixXd> matrix){
    bool matrixPD = true;
    //TODO implement cholesky decomp for PD check and maybe use result for inverse Q_uu

    Eigen::LLT<Eigen::MatrixXd> lltOfA(matrix); // compute the Cholesky decomposition of the matrix
    if(lltOfA.info() == Eigen::NumericalIssue)
    {
        matrixPD = false;
    }

    return matrixPD;
}

float iLQR::forwardsPass(float oldCost){
    float alpha = 1.0;
    float newCost = 0.0;
    bool costReduction = false;
    int alphaCount = 0;

    while(!costReduction){
        cpMjData(model, mdata, d_init);
        newCost = 0;
        m_state stateFeedback;
        m_state _X;
        m_state X_new;
        m_ctrl _U;

        for(int t = 0; t < ilqr_horizon_length; t++){
            // Step 1 - get old state and old control that were linearised around

            if(COPYING_DERIVS){
                _X = modelTranslator->returnState(dArray[t]);
                _U = modelTranslator->returnControls(dArray[t]);
            }

            for(int i = 0; i < num_mj_steps_per_control; i++){

                X_new = modelTranslator->returnState(mdata);

                if(!COPYING_DERIVS){
                    _X = X_old[(t * num_mj_steps_per_control) + i].replicate(1,1);
                    _U = U_old[(t * num_mj_steps_per_control) + i].replicate(1,1);
                }

                // Calculate difference from new state to old state
                stateFeedback = X_new - _X;

                m_ctrl feedBackGain = K[(t * num_mj_steps_per_control) + i] * stateFeedback;

                if(alphaCount == 9){
                    // guarantee if alpha = 0, we get old trajectory
                    U_new[(t * num_mj_steps_per_control) + i] = U_old[(t * num_mj_steps_per_control) + i];
                }
                else{

                    U_new[(t * num_mj_steps_per_control) + i] = _U + (alpha * k[(t * num_mj_steps_per_control) + i]) + feedBackGain;
                }

                // Clamp torques within torque limits
                for(int k = 0; k < NUM_CTRL; k++){
                    if(U_new[(t * num_mj_steps_per_control) + i](k) > modelTranslator->torqueLims[k]) U_new[(t * num_mj_steps_per_control) + i](k) = modelTranslator->torqueLims[k];
                    if(U_new[(t * num_mj_steps_per_control) + i](k) < -modelTranslator->torqueLims[k]) U_new[(t * num_mj_steps_per_control) + i](k) = -modelTranslator->torqueLims[k];
                }

//                cout << "old control: " << endl << U_old[(t * num_mj_steps_per_control) + i] << endl;
//                cout << "state feedback" << endl << stateFeedback << endl;
//                cout << "new control: " << endl << U_new[(t * num_mj_steps_per_control) + i] << endl;

                modelTranslator->setControls(mdata, U_new[(t * num_mj_steps_per_control) + i], grippersOpen_iLQR[(t * num_mj_steps_per_control) + i]);

                float currentCost;
                if(t == 0){
                    currentCost = modelTranslator->costFunction(mdata, t, ilqr_horizon_length, U_new[0], true);
                }
                else{
                    currentCost = modelTranslator->costFunction(mdata, t, ilqr_horizon_length, U_new[(t * num_mj_steps_per_control) + i - 1], false);
                }

                newCost += (currentCost * MUJOCO_DT);

                modelTranslator->stepModel(mdata, 1);
            }
        }

        if(newCost < oldCost){
            costReduction = true;
        }
        else{
            alpha = alpha - 0.1;
            alphaCount++;
            if(alpha <= 0){
                break;
            }
        }
    }

    if(newCost < oldCost){

        cpMjData(model, mdata, d_init);

        for(int k = 0; k < NUM_CTRL; k++){
            mdata->ctrl[k] = U_new[0](k);
        }

        cpMjData(model, dArray[0], mdata);

        for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){

            X_final[i] = modelTranslator->returnState(mdata);
            modelTranslator->setControls(mdata, U_new.at(i), grippersOpen_iLQR[i]);
            X_old.at(i) = modelTranslator->returnState(mdata);

            modelTranslator->stepModel(mdata, 1);
            cpMjData(model, dArray[i + 1], mdata);
        }

        cpMjData(model, dArray[ilqr_horizon_length], mdata);
    }

    //m_state termStateBest = modelTranslator->returnState(mdata);
    //cout << "terminal state best: " << endl << termStateBest << endl;
    cout << "best alpha was " << alpha << endl;
    //cout << "best final control: " << modelTranslator->returnControls(dArray[ilqr_horizon_length - 1]) << endl;
//    cout << "best cost was " << newCost << endl;
//    cout << "-------------------- END FORWARDS PASS ------------------------" << endl;

    return newCost;
}

bool iLQR::checkForConvergence(float newCost, float oldCost){
    bool convergence = false;
    m_state terminalState = modelTranslator->returnState(mdata);

    if(modelTranslator->taskNumber == 2){
        double cubeXDiff = terminalState(7) - modelTranslator->X_desired(7);
        double cubeYDiff = terminalState(8) - modelTranslator->X_desired(8);
        cout << "final cube pos, x: " << terminalState(7) << ", y: " << terminalState(8) << endl;
        cout << "final cube pos, x desired: " << modelTranslator->X_desired(7) << ", y: " << modelTranslator->X_desired(8) << endl;
        cout << "final cube pos, x diff: " << cubeXDiff << ", y: " << cubeYDiff << endl;
    }

    std::cout << "New cost: " << newCost <<  std::endl;
    std::cout << "--------------------------------------------------" <<  std::endl;


    numIterations++;
    float costGrad = (oldCost - newCost)/newCost;

    if(numIterations > 2 and (costGrad < epsConverge)) {
        convergence = true;
        cout << "ilQR converged, num Iterations: " << numIterations << " final cost: " << newCost << endl;
        float linTimeSum = 0.0f;
        int evalsSum = 0;
        double varianceSum = 0.0f;
        for(int i = 0; i < numIterations; i++){
            linTimeSum += linTimes[i];
            if(DYNAMIC_LINEAR_DERIVS){
                evalsSum += numEvals[i];
                varianceSum += evalsVariance[i];
            }
        }
        avgLinTime = linTimeSum / numIterations;
        avgNumEvals = evalsSum / numIterations;
        avgVariance = varianceSum / numIterations;
        finalCost = newCost;
        //cubeTermPos(0) = cubeX;
        //cubeTermPos(1) = cubeY;
        cout << "average time linearising " << linTimeSum / numIterations << endl;
    }

    // store new controls
    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
        U_old[i] = U_new[i].replicate(1, 1);
    }

    return convergence;
}

//bool iLQR::updateScaling(){
//    bool algorithmFinished = false;
//    scalingLevelCount++;
//
//    if(scalingLevelCount < NUM_SCALING_LEVELS){
//        num_mj_steps_per_control = scalingLevel[scalingLevelCount];
//        ilqr_dt = MUJOCO_DT * num_mj_steps_per_control;
//        ilqr_horizon_length = MUJ_STEPS_HORIZON_LENGTH / num_mj_steps_per_control;
//        updateDataStructures();
//        cout << "Scaling updated: num steps per linearisation: " << num_mj_steps_per_control << "horizon: " << ilqr_horizon_length <<  endl;
//    }
//    else{
//        algorithmFinished = true;
//    }
//
//    return algorithmFinished;
//}

//void iLQR::updateDataStructures(){
//    cpMjData(model, mdata, d_init);
//    for(int i = 0; i < ilqr_horizon_length; i++){
////        cout << "index: " << (i * num_mj_steps_per_control) << endl;
////        cout << "control: " << U_old[(i * num_mj_steps_per_control)] << endl;
//        modelTranslator->setControls(mdata, U_old[(i * num_mj_steps_per_control)], grippersOpen_iLQR[i]);
//
//        cpMjData(model, dArray[i], mdata);
//
//        for(int i = 0; i < num_mj_steps_per_control; i++){
//            modelTranslator->stepModel(mdata, 1);
//        }
//    }
//
//    cpMjData(model, mdata, d_init);
//}

void iLQR::lineariseDynamics(Ref<MatrixXd> _A, Ref<MatrixXd> _B, mjData *linearisedData){
    // Initialise variables
    static int nwarmup = 3;

    float epsControls = 1e-6;
    float epsVelocities = 1e-6;
    float epsPos = 1e-6;

    // Initialise matrices for forwards dynamics
    m_dof_dof dqveldq;
    m_dof_dof dqveldqvel;
    m_dof_dof dqposdqvel;
    m_dof_ctrl dqveldctrl;
    m_dof_dof dqaccdqvel;
    m_dof_dof dqaccdq;
    m_dof_ctrl dqaccdctrl;

    m_dof velDec;
    m_dof velInc;
    m_dof acellInc, acellDec;

    // Create a copy of the current data that we want to differentiate around
    mjData *saveData;
    saveData = mj_makeData(model);
    cpMjData(model, saveData, linearisedData);

    // Allocate memory for variables
    mjtNum* warmstart = mj_stackAlloc(saveData, DOF);

//    cout << "accel before: " << saveData->qacc[0] << endl;
//    // Compute mj_forward once with no skips
    mj_forward(model, saveData);
//    cout << "accel before: " << saveData->qacc[0] << endl;

    // Compute mj_forward a few times to allow optimiser to get a more accurate value for qacc
    // skips position and velocity stages (TODO LOOK INTO IF THIS IS NEEDED FOR MY METHOD)
    for( int rep=1; rep<nwarmup; rep++ )
        mj_forwardSkip(model, saveData, mjSTAGE_VEL, 1);

    // save output for center point and warmstart (needed in forward only)
    mju_copy(warmstart, saveData->qacc_warmstart, DOF);

    // CALCULATE dqveldctrl

    for(int i = 0; i < NUM_CTRL; i++){
        saveData->ctrl[i] = linearisedData->ctrl[i] + epsControls;

        // evaluate dynamics, with center warmstart
        mju_copy(saveData->qacc_warmstart, warmstart, model->nv);
        //mj_forwardSkip(model, saveData, mjSTAGE_VEL, 1);
        modelTranslator->stepModel(saveData, 1);

        // copy and store +perturbation
        velInc = modelTranslator->returnVelocities(saveData);

        // perturb selected target -
        cpMjData(model, saveData, linearisedData);
        saveData->ctrl[i] = linearisedData->ctrl[i] - epsControls;

        // evaluate dynamics, with center warmstart
        mju_copy(saveData->qacc_warmstart, warmstart, model->nv);
        modelTranslator->stepModel(saveData, 1);

        velDec = modelTranslator->returnVelocities(saveData);

        for(int j = 0; j < DOF; j++){
            dqveldctrl(j, i) = (velInc(j) - velDec(j))/(2*epsControls);
        }

        // undo pertubation
        cpMjData(model, saveData, linearisedData);

    }


//    //calculate dqacc/dctrl
//    for(int i = 0; i < NUM_CTRL; i++){
//        saveData->ctrl[i] = linearisedData->ctrl[i] + epsControls;
//
//        // evaluate dynamics, with center warmstart
//        mju_copy(saveData->qacc_warmstart, warmstart, model->nv);
//        mj_forwardSkip(model, saveData, mjSTAGE_VEL, 1);
//
//        // copy and store +perturbation
//        acellInc = modelTranslator->returnAccelerations(saveData);
//
//        // perturb selected target -
//        cpMjData(model, saveData, linearisedData);
//        saveData->ctrl[i] = linearisedData->ctrl[i] - epsControls;
//
//        // evaluate dynamics, with center warmstart
//        mju_copy(saveData->qacc_warmstart, warmstart, model->nv);
//        mj_forwardSkip(model, saveData, mjSTAGE_VEL, 1);
//
//        acellDec = modelTranslator->returnAccelerations(saveData);
//
//        for(int j = 0; j < DOF; j++){
//            dqaccdctrl(j, i) = (acellInc(j) - acellDec(j))/(2*epsControls);
//        }
//
//        // undo pertubation
//        cpMjData(model, saveData, linearisedData);
//
//    }

    // CALCULATE dqaccdvel
//    for(int i = 0; i < DOF; i++){
//        // perturb velocity +
//
//        modelTranslator->perturbVelocity(saveData, linearisedData, i, epsVelocities);
//
//        // evaluate dynamics, with center warmstart
//        mju_copy(saveData->qacc_warmstart, warmstart, model->nv);
//        mj_forwardSkip(model, saveData, mjSTAGE_POS, 1);
//
//        // copy and store +perturbation
//        acellInc = modelTranslator->returnAccelerations(saveData);
//        //cout << "acellInc " << endl << acellInc << endl;
//
//        // perturb velocity -
//        modelTranslator->perturbVelocity(saveData, linearisedData, i, -epsVelocities);
//
//        // evaluate dynamics, with center warmstart
//        mju_copy(saveData->qacc_warmstart, warmstart, model->nv);
//        mj_forwardSkip(model, saveData, mjSTAGE_POS, 1);
//
//        acellDec = modelTranslator->returnAccelerations(saveData);
//        //cout << "acellDec " << endl << acellDec << endl;
//
//        // compute column i of derivative 1
//        for(int j = 0; j < DOF; j++){
//            dqaccdqvel(j, i) = (acellInc(j) - acellDec(j))/(2*epsVelocities);
//        }
//        //cout << "dq/dvel " << endl << dqaccdqvel << endl;
//
//        // undo perturbation
//        cpMjData(model, saveData, linearisedData);
//    }

    // CALCULATE dqveldvel
    for(int i = 0; i < DOF; i++){
        // perturb velocity +
        modelTranslator->perturbVelocity(saveData, linearisedData, i, epsVelocities);

        // evaluate dynamics, with center warmstart
        mju_copy(saveData->qacc_warmstart, warmstart, model->nv);
        modelTranslator->stepModel(saveData, 1);

        // copy and store +perturbation
        velInc = modelTranslator->returnVelocities(saveData);

        // undo perturbation
        cpMjData(model, saveData, linearisedData);

        modelTranslator->perturbVelocity(saveData, linearisedData, i, epsVelocities);

        // evaluate dynamics, with center warmstart
        mju_copy(saveData->qacc_warmstart, warmstart, model->nv);
        modelTranslator->perturbVelocity(saveData, linearisedData, i, -epsVelocities);

        // evaluate dynamics, with center warmstart
        mju_copy(saveData->qacc_warmstart, warmstart, model->nv);
        modelTranslator->stepModel(saveData, 1);

        velDec = modelTranslator->returnVelocities(saveData);

        // compute column i of derivative 1
        for(int j = 0; j < DOF; j++){
            double diffScaled = (velInc(j) - velDec(j));
            dqveldqvel(j, i) = diffScaled/(2*epsVelocities);
        }

        // undo perturbation
        cpMjData(model, saveData, linearisedData);
    }

//    for(int i = 0; i < DOF; i++){
//        // perturb position +
//        modelTranslator->perturbPosition(saveData, linearisedData, i, epsPos);
//
//        // evaluate dynamics, with center warmstart
//        mju_copy(saveData->qacc_warmstart, warmstart, model->nv);
//        modelTranslator->stepModel(saveData, 1);
//        velInc = modelTranslator->returnVelocities(saveData);
//
//        // perturb position -
//        modelTranslator->perturbPosition(saveData, linearisedData, i, -epsPos);
//        modelTranslator->stepModel(saveData, 1);
//        velDec = modelTranslator->returnVelocities(saveData);
//
//        // evaluate dynamics, with center warmstart
////        mju_copy(saveData->qacc_warmstart, warmstart, model->nv);
////        mj_forwardSkip(model, saveData, mjSTAGE_NONE, 1);
////        if(computeCost){
////            costDec = modelTranslator->costFunction(saveData, controlNum, totalControls, dummyControl, true);
////            cout << "cost dec was: " << costDec << endl;
////        }
////
////        acellDec = modelTranslator->returnAccelerations(saveData);
//        // compute column i of derivative 1
//        for(int j = 0; j < DOF; j++){
//            dqveldq(j, i) = (velInc(j) - velDec(j))/(2*epsPos);
//        }
//
//        //cout << "dqaccdq " << endl << dqaccdq << endl;
//
//        // undo perturbation
//        cpMjData(model, saveData, linearisedData);
//    }

    // CALUCLATE dqaccdqpos
    for(int i = 0; i < DOF; i++){
        // perturb position +
        modelTranslator->perturbPosition(saveData, linearisedData, i, epsPos);

        // evaluate dynamics, with center warmstart
        mju_copy(saveData->qacc_warmstart, warmstart, model->nv);
        mj_forwardSkip(model, saveData, mjSTAGE_NONE, 1);

        acellInc = modelTranslator->returnAccelerations(saveData);

        // perturb position -
        modelTranslator->perturbPosition(saveData, linearisedData, i, -epsPos);

        // evaluate dynamics, with center warmstart
        mju_copy(saveData->qacc_warmstart, warmstart, model->nv);
        mj_forwardSkip(model, saveData, mjSTAGE_NONE, 1);


        acellDec = modelTranslator->returnAccelerations(saveData);
        // compute column i of derivative 1
        for(int j = 0; j < DOF; j++){
            dqaccdq(j, i) = (acellInc(j) - acellDec(j))/(2*epsPos);
        }



        //cout << "dqaccdq " << endl << dqaccdq << endl;

        // undo perturbation
        cpMjData(model, saveData, linearisedData);
    }


    mj_deleteData(saveData);

    for(int i = 0; i < DOF; i++){
        for(int j = 0; j < DOF; j++){
            if(dqaccdq(i, j) > DQACCDQ_MAX){
                dqaccdq(i, j) = DQACCDQ_MAX;
            }
            if(dqaccdq(i, j)< -DQACCDQ_MAX){
                dqaccdq(i, j) = -DQACCDQ_MAX;
            }
        }
    }


    _A.block(DOF, 0, DOF, DOF) = (dqaccdq * MUJOCO_DT);
    //_A.block(DOF, 0, DOF, DOF) = dqveldq;
    //_A.block(DOF, DOF, DOF, DOF).setIdentity();
    _A.block(DOF, DOF, DOF, DOF) = dqveldqvel;
    _B.block(DOF, 0, DOF, NUM_CTRL) = dqveldctrl;
    //_B.block(DOF, 0, DOF, NUM_CTRL) = (dqaccdctrl * MUJOCO_DT);

    //cout << "A matrix is: " << _A << endl;
//    cout << " B Mtrix is: " << _B << endl;

}

m_ctrl iLQR::returnDesiredControl(int controlIndex, bool finalControl){
    if(finalControl){
        return finalControls[controlIndex];
    }
    else{
        return initControls[controlIndex];
    }
}

void iLQR::setInitControls(std::vector<m_ctrl> _initControls, std::vector<bool> _grippersOpen){

    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
        initControls[i] = _initControls[i].replicate(1,1);
        U_old[i] = _initControls[i].replicate(1,1);
        U_new[i] = _initControls[i].replicate(1,1);

        grippersOpen_iLQR[i] = _grippersOpen[i];
    }
}

void iLQR::makeDataForOptimisation(){

    for(int i = 0; i <= MUJ_STEPS_HORIZON_LENGTH; i++){
        // populate dArray with mujoco data objects from start of trajec until end
        dArray[i] = mj_makeData(model);

    }
    dArray[MUJ_STEPS_HORIZON_LENGTH] = mj_makeData(model);
    cpMjData(model, dArray[MUJ_STEPS_HORIZON_LENGTH], mdata);

    // reset mdata back to initial state
    cpMjData(model, mdata, d_init);
}

void iLQR::deleteMujocoData(){
    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
        mj_deleteData(dArray[i]);
    }
}

void iLQR::updateNumStepsPerDeriv(int stepPerDeriv){
    num_mj_steps_per_control = stepPerDeriv;
    ilqr_dt = MUJOCO_DT * num_mj_steps_per_control;
    ilqr_horizon_length = MUJ_STEPS_HORIZON_LENGTH / num_mj_steps_per_control;
    numIterations = 0;
    linTimes.clear();
    numEvals.clear();
    evalsVariance.clear();

}

void iLQR::resetInitialStates(mjData *_d_init, m_state _X0){
    cpMjData(model, d_init, _d_init);
    X0 = _X0.replicate(1,1);
    trajecCollisionFree = true;
    lamda = 0.1;
    numIterations = 0;

}

//void iLQR::scaleLinearisation(Ref<m_state_state> A_scaled, Ref<m_state_ctrl> B_scaled, Ref<m_state_state> A, Ref<m_state_ctrl> B, int num_steps_per_dt){
//
//    // TODO look into ways of speeding up matrix to the power of calculation
//    A_scaled = A.replicate(1, 1);
//    B_scaled = B.replicate(1, 1);
//    m_state_ctrl currentBTerm;
//
//    for(int i = 0; i < num_steps_per_dt - 1; i++){
//        A_scaled *= A;
//    }
//
//    currentBTerm = B.replicate(1, 1);
//    for(int i = 0; i < num_steps_per_dt - 1; i++){
//        currentBTerm = A * currentBTerm;
//        B_scaled += currentBTerm;
//    }
//}