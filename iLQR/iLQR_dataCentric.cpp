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

iLQR::iLQR(mjModel* m, mjData* d, taskTranslator* _modelTranslator){
    numIterations = 0;
    lamda = 0.1;

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
    modelTranslator = _modelTranslator;
    mdata = mj_makeData(model);
    cpMjData(model, mdata, d);

    d_init = mj_makeData(model);

    makeDataForOptimisation();

}

std::vector<m_ctrl> iLQR::optimise(mjData *_d_init, std::vector<m_ctrl> initControls, int maxIterations, int horizonLength, int stepsPerDeriv){
    // Setup Initial variables
    bool optimisationFinished = false;
    float newCost = 0;
    float oldCost;
    lamda = 0.1;
    ilqr_horizon_length = horizonLength;
    updateNumStepsPerDeriv(stepsPerDeriv);
    // time how long it took to compute optimal controls
    auto optstart = high_resolution_clock::now();
    cpMjData(model, d_init, _d_init);
    numIterations = 0;
    std::vector<bool> tempGripperControls;
    setInitControls(initControls, tempGripperControls);

    auto start = high_resolution_clock::now();
    auto stop = high_resolution_clock::now();
    auto linDuration = duration_cast<microseconds>(stop - start);
    auto bpDuration = duration_cast<microseconds>(stop - start);

    oldCost = rollOutTrajectory();
    cout << "////////////////////////////////////////////////////////////////////////////////////" << endl;
    cout << "initial Trajectory cost: " << oldCost << endl;
    cout << "---------------------------------------------------- " << endl;

    // iterate until optimisation finished, convergence or if lamda > maxLamda
    for(int i = 0; i < maxIterations; i++){

        start = high_resolution_clock::now();

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

        stop = high_resolution_clock::now();
        linDuration = duration_cast<microseconds>(stop - start);
        linTimes.push_back(linDuration.count()/1000);

        bool validBackPass = false;
        bool lamdaExit = false;

        // Until a valid backwards pass was calculated with no PD Q_uu_reg matrices
        while(!validBackPass) {

            // STEP 2 - Backwards pass to compute optimal linear and feedback gain matrices k and K
            start = high_resolution_clock::now();
            validBackPass = backwardsPass_Quu_reg();
            //validBackPass = backwardsPass_Vxx_reg();
            stop = high_resolution_clock::now();
            bpDuration = duration_cast<microseconds>(stop - start);

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
            start = high_resolution_clock::now();
            bool costReduced = false;
            if(!DYNAMIC_LINEAR_DERIVS){
                newCost = forwardsPassStatic(oldCost, costReduced);
            }
            else{
                newCost = forwardsPassDynamic(oldCost, costReduced);
            }

            stop = high_resolution_clock::now();
            auto fPDuration = duration_cast<microseconds>(stop - start);
            cout << "Lin: " << linDuration.count()/1000 << " ms," << " BP: " << bpDuration.count()/1000 << " ms," << " FP: " << fPDuration.count()/1000 << " ms" << endl;
            // STEP 4 - Check for convergence
            bool currentStepsConverged = checkForConvergence(newCost, oldCost, costReduced);
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

    cout << "-----------------------------------------------------------------------------------" << endl;
    m_state terminalState = modelTranslator->returnState(dArray[ilqr_horizon_length]);
    if(modelTranslator->taskNumber == 2){
        double cubeXDiff = terminalState(7) - modelTranslator->X_desired(7);
        double cubeYDiff = terminalState(8) - modelTranslator->X_desired(8);
        cout << "final cube pos, x: " << terminalState(7) << ", y: " << terminalState(8) << endl;
        cout << "final cube pos, x desired: " << modelTranslator->X_desired(7) << ", y: " << modelTranslator->X_desired(8) << endl;
        cout << "final cube pos, x diff: " << cubeXDiff << ", y: " << cubeYDiff << endl;
    }


    auto optstop = high_resolution_clock::now();
    auto optduration = duration_cast<microseconds>(optstop - optstart);
    cout << "Optimisation took: " << optduration.count()/1000 << " milliseconds" << endl;
    cout << "////////////////////////////////////////////////////////////////////////////////////" << endl;

    std::vector<m_ctrl> returnControls;
    for(int i = 0; i < ilqr_horizon_length; i++){
        returnControls.push_back(U_old[i]);
    }
    return returnControls;
}

float iLQR::rollOutTrajectory(){
    float cost = 0;

    cpMjData(model, mdata, d_init);
    X_old[0] = modelTranslator->returnState(mdata);
    cpMjData(model, dArray[0], mdata);
    int stepsCounter = num_mj_steps_per_control;

    for(int i = 0; i < ilqr_horizon_length; i++){
        modelTranslator->setControls(mdata, U_old[i], grippersOpen_iLQR[i]);
        float stateCost;
        if(i == 0){
            stateCost = modelTranslator->costFunction(mdata, i, ilqr_horizon_length, d_init);
        }
        else{
            stateCost = modelTranslator->costFunction(mdata, i, ilqr_horizon_length, dArray[i - 1]);
        }

        cost += (stateCost * MUJOCO_DT);
        modelTranslator->stepModel(mdata, 1);
        X_old[i + 1] = modelTranslator->returnState(mdata);

        cpMjData(model, dArray[i+1], mdata);

        if (VISUALISE_ROLLOUTS) {
            if (i % 40 == 0) {
                mjrRect viewport = {0, 0, 0, 0};
                glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

                // update scene and render
                mjv_updateScene(model, mdata, &opt, NULL, &cam, mjCAT_ALL, &scn);
                mjr_render(viewport, &scn, &con);

                // swap OpenGL buffers (blocking call due to v-sync)
                glfwSwapBuffers(window);

                // process pending GUI events, call GLFW callbacks
                glfwPollEvents();
            }
        }

    }

    m_state termState = modelTranslator->returnState(mdata);
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

    for(int i = 0; i < ilqr_horizon_length; i++){

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

    evaluationWaypoints.push_back(ilqr_horizon_length - 1);


    cout << "num evaluation points: " << evaluationWaypoints.size() << endl;

    std::vector<int> distBetweenIndices;

    for(int i = 0; i < evaluationWaypoints.size() - 1; i++){

        distBetweenIndices.push_back(evaluationWaypoints[i+1] - evaluationWaypoints[i]);

    }
//    double variance;
//
//    variance = calcVariance(distBetweenIndices);
//
//    evalsVariance.push_back(variance);


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
    for(int i = 0; i < ilqr_horizon_length; i++){
        m_ctrl lastControl;
        if(i == 0){
//            lastControl.setZero();
            modelTranslator->costDerivatives(dArray[i], l_x[i], l_xx[i], l_u[i], l_uu[i], i, ilqr_horizon_length, dArray[0]);
        }
        else{
//            lastControl = modelTranslator->returnControls(dArray[i - 1]);
            modelTranslator->costDerivatives(dArray[i], l_x[i], l_xx[i], l_u[i], l_uu[i], i, ilqr_horizon_length, dArray[i-1]);
        }

    }

    modelTranslator->costDerivatives(dArray[ilqr_horizon_length], l_x[ilqr_horizon_length], l_xx[ilqr_horizon_length], l_u[ilqr_horizon_length - 1], l_uu[ilqr_horizon_length - 1], ilqr_horizon_length - 1, ilqr_horizon_length, dArray[ilqr_horizon_length-1]);


}

void iLQR::getDerivativesStatically(){

    int save_iterations = model->opt.iterations;
    mjtNum save_tolerance = model->opt.tolerance;

    model->opt.iterations = 30;
    model->opt.tolerance = 0;

    // Linearise the dynamics along the trajectory
    #pragma omp parallel for default(none)
    for(int t = 0; t < numCalcedDerivs; t++){

        lineariseDynamics(A[t], B[t], dArray[t * num_mj_steps_per_control]);
    }

    #pragma omp parallel for default(none)
    for(int i = 0; i < ilqr_horizon_length; i++){
        if(i == 0){
            modelTranslator->costDerivatives(dArray[i], l_x[i], l_xx[i], l_u[i], l_uu[i], i, ilqr_horizon_length, dArray[0]);
        }
        else{
            modelTranslator->costDerivatives(dArray[i], l_x[i], l_xx[i], l_u[i], l_uu[i], i, ilqr_horizon_length, dArray[i - 1]);
        }
    }

    modelTranslator->costDerivatives(dArray[ilqr_horizon_length], l_x[ilqr_horizon_length], l_xx[ilqr_horizon_length], l_u[ilqr_horizon_length - 1], l_uu[ilqr_horizon_length - 1], ilqr_horizon_length - 1, ilqr_horizon_length, dArray[ilqr_horizon_length - 1]);

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

}

void iLQR::smoothAMatrices(){

    //cout << "begin smoothing matrices" << endl;
    // loop through bottom left part of matrix and calculate mean + standard deviation, then remove vals appropriately
    for(int i = 0; i < DOF; i++){
        for(int j = 0; j < DOF; j++){
            double sum = 0;

            // calculate the mean
            for(int k = 0; k < numCalcedDerivs; k++){
                sum += A[k](i + DOF, j);
            }

            double mean = sum / numCalcedDerivs;
            double sumStdDev;

            // calcuilate the standard deviation
            for(int k = 0; k < numCalcedDerivs; k++){
                sumStdDev += pow(A[k](i + DOF, j) - mean, 2);
            }

            double stdDev = sqrt(sumStdDev / numCalcedDerivs);

            //cout << "mean is: " << mean << endl;
            //cout << "std dev is: " << stdDev << endl;

            // any values outside 2 standard deviations, set them to the mean
            for(int k = 0; k < numCalcedDerivs; k++){
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

}


void iLQR::copyDerivatives(){

    for(int t = 0; t < numCalcedDerivs; t++){
        for(int i = 0; i < num_mj_steps_per_control; i++){
            f_x[(t * num_mj_steps_per_control) + i] = A[t].replicate(1,1);
            f_u[(t * num_mj_steps_per_control) + i] = B[t].replicate(1,1);
        }
    }
}

void iLQR::linearInterpolateDerivs(){
    for(int t = 0; t < numCalcedDerivs; t++){

        m_state_state addA;
        m_state_ctrl addB;

        if(t != numCalcedDerivs - 1){
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

    f_x[ilqr_horizon_length - 1] = f_x[ilqr_horizon_length - 2].replicate(1,1);
    f_u[ilqr_horizon_length - 1] = f_u[ilqr_horizon_length - 2].replicate(1,1);

    f_x[ilqr_horizon_length] = f_x[ilqr_horizon_length - 1].replicate(1,1);
    f_u[ilqr_horizon_length] = f_u[ilqr_horizon_length - 1].replicate(1,1);
}

bool iLQR::backwardsPass_Quu_reg(){
    m_state V_x;
    V_x = l_x[ilqr_horizon_length];
    m_state_state V_xx;
    V_xx = l_xx[ilqr_horizon_length];
    int Quu_pd_check_counter = 0;
    int number_steps_between_pd_checks = 100;
    float time = 0.0f;

    for(int t = ilqr_horizon_length - 1; t > -1; t--){
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

//        cout << "------------------ iteration " << t << " ------------------" << endl;
//        cout << "l_x " << l_x[t] << endl;
//        cout << "l_xx " << l_xx[t] << endl;
//        cout << "l_u " << l_u[t] << endl;
//        cout << "l_uu " << l_uu[t] << endl;
//        cout << "Q_ux " << Q_ux << endl;
//        cout << "f_u[t] " << f_u[t] << endl;
//        cout << "Q_uu " << Q_uu << endl;
//        cout << "Q_uu_inv " << Q_uu_inv << endl;
//        cout << "Q_x " << Q_x << endl;
//        cout << "Q_xx " << Q_xx << endl;
//        cout << "V_xx " << V_xx << endl;
//        cout << "V_x " << V_x << endl;
//        cout << "K[t] " << K[t] << endl;
    }

    return true;
}

bool iLQR::backwardsPass_Vxx_reg(){
    m_state V_x;
    V_x = l_x[ilqr_horizon_length];
    m_state_state V_xx;
    V_xx = l_xx[ilqr_horizon_length];

    for(int t = ilqr_horizon_length - 1; t > -1; t--){
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

float iLQR::forwardsPassDynamic(float oldCost, bool &costReduced){
    float alpha = 1.0;
    float newCost = 0.0;
    bool costReduction = false;
    int alphaCount = 0;
    float alphaReduc = 0.1;
    int alphaMax = (1 / alphaReduc) - 1;
    mjData *d_old;
    d_old = mj_makeData(model);

    while(!costReduction){
        cpMjData(model, mdata, d_init);
        cpMjData(model, d_old, mdata);
        newCost = 0;
        m_state stateFeedback;
        m_state _X;
        m_state X_new;
        m_ctrl _U;

        for(int t = 0; t < ilqr_horizon_length; t++) {
            // Step 1 - get old state and old control that were linearised around
            _X = modelTranslator->returnState(dArray[t]);
            _U = modelTranslator->returnControls(dArray[t]);

            X_new = modelTranslator->returnState(mdata);

            // Calculate difference from new state to old state
            stateFeedback = X_new - _X;

            m_ctrl feedBackGain = K[t] * stateFeedback;

            // Calculate new optimal controls
            U_new[t] = _U + (alpha * k[t]) + feedBackGain;

            // Clamp torques within torque limits
            if (TORQUE_CONTROL) {
                for (int k = 0; k < NUM_CTRL; k++) {
                    if (U_new[t](k) > modelTranslator->torqueLims[k]) U_new[t](k) = modelTranslator->torqueLims[k];
                    if (U_new[t](k) < -modelTranslator->torqueLims[k]) U_new[t](k) = -modelTranslator->torqueLims[k];
                }
            }


//            cout << "old control: " << endl << U_old[t] << endl;
////            cout << "state feedback" << endl << stateFeedback << endl;
//            cout << "new control: " << endl << U_new[t] << endl;

            modelTranslator->setControls(mdata, U_new[t], grippersOpen_iLQR[t]);

            float currentCost;
            currentCost = modelTranslator->costFunction(mdata, t, ilqr_horizon_length, d_old);

            newCost += (currentCost * MUJOCO_DT);

            if (VISUALISE_ROLLOUTS) {
                if (t % 40 == 0) {

//                    cout << "old control: " << endl << _U << endl;
//                    cout << "feedBackGain" << endl << feedBackGain << endl;
//                    cout << "new control: " << endl << U_new[t] << endl;

                    mjrRect viewport = {0, 0, 0, 0};
                    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

                    // update scene and render
                    mjv_updateScene(model, mdata, &opt, NULL, &cam, mjCAT_ALL, &scn);
                    mjr_render(viewport, &scn, &con);

                    // swap OpenGL buffers (blocking call due to v-sync)
                    glfwSwapBuffers(window);

                    // process pending GUI events, call GLFW callbacks
                    glfwPollEvents();
                }
            }
            cpMjData(model, d_old, mdata);
            modelTranslator->stepModel(mdata, 1);
        }

//        cout << "cost from alpha: " << alphaCount << ": " << newCost << endl;

        if(newCost < oldCost){
            costReduction = true;
            costReduced = true;
        }
        else{
            alpha = alpha - 0.1;
            alphaCount++;
            if(alphaCount >= alphaMax){
                break;
            }
        }
    }

    mj_deleteData(d_old);

    // If the cost was reduced
    if(newCost < oldCost){

        // Copy initial data to main data
        cpMjData(model, mdata, d_init);

//        for(int k = 0; k < NUM_CTRL; k++){
//            mdata->ctrl[k] = U_new[0](k);
//        }
        modelTranslator->setControls(mdata, U_new.at(0), grippersOpen_iLQR[0]);

        cpMjData(model, dArray[0], mdata);

        for(int i = 0; i < ilqr_horizon_length; i++){

            X_final[i] = modelTranslator->returnState(mdata);
            modelTranslator->setControls(mdata, U_new.at(i), grippersOpen_iLQR[i]);
            X_old.at(i) = modelTranslator->returnState(mdata);

            modelTranslator->stepModel(mdata, 1);
            cpMjData(model, dArray[i + 1], mdata);
        }

        cout << "best alpha was " << alpha << endl;
        cout << "cost improved in F.P - new cost: " << newCost << endl;
        m_state termStateBest = modelTranslator->returnState(dArray[ilqr_horizon_length]);
        if(modelTranslator->taskNumber == 2){
            double cubeXDiff = termStateBest(7) - modelTranslator->X_desired(7);
            double cubeYDiff = termStateBest(8) - modelTranslator->X_desired(8);
            cout << "final cube pos, x: " << termStateBest(7) << ", y: " << termStateBest(8) << endl;
            cout << "final cube pos, x desired: " << modelTranslator->X_desired(7) << ", y: " << modelTranslator->X_desired(8) << endl;
            cout << "final cube pos, x diff: " << cubeXDiff << ", y: " << cubeYDiff << endl;
        }
        return newCost;
    }

    return oldCost;
}

//float iLQR::forwardsPassDynamic(float oldCost, bool &costReduced){
//    float alpha = 1.0;
//    bool costReduction = false;
//    mjData *d_old;
//    d_old = mj_makeData(model);
//
//    double newCost[5] = {0.0f};
//    float alphas[5] = {0.2, 0.4, 0.6, 0.8, 1.0};
//    std::vector<std::vector<m_ctrl>> U_new_alpha;
//
//
//    #pragma omp parallel for default(none)
//    for(int i = 0; i < 5; i++){
//        cpMjData(model, d_alpha[i], d_init);
//        cpMjData(model, d_alpha_last[i], mdata);
//        m_state stateFeedback;
//        m_state _X;
//        m_state X_new;
//        m_ctrl _U;
//
//        for(int t = 0; t < ilqr_horizon_length; t++) {
//            // Step 1 - get old state and old control that were linearised around
//            _X = modelTranslator->returnState(dArray[t]);
//            _U = modelTranslator->returnControls(dArray[t]);
//
//            X_new = modelTranslator->returnState(d_alpha[i]);
//
//            // Calculate difference from new state to old state
//            stateFeedback = X_new - _X;
//
//            m_ctrl feedBackGain = K[t] * stateFeedback;
//
//            // Calculate new optimal controls
//            U_new[t] = _U + (alphas[i] * k[t]) + feedBackGain;
//
//            // Clamp torques within torque limits
//            if (TORQUE_CONTROL) {
//                for (int k = 0; k < NUM_CTRL; k++) {
//                    if (U_new[t](k) > modelTranslator->torqueLims[k]) U_new[t](k) = modelTranslator->torqueLims[k];
//                    if (U_new[t](k) < -modelTranslator->torqueLims[k]) U_new[t](k) = -modelTranslator->torqueLims[k];
//                }
//            }
//
//            modelTranslator->setControls(mdata, U_new[t], grippersOpen_iLQR[t]);
//
//            float currentCost;
//            currentCost = modelTranslator->costFunction(mdata, t, ilqr_horizon_length, d_alpha_last[i]);
//
//            newCost[i] += (currentCost * MUJOCO_DT);
//
//            cpMjData(model, d_alpha_last[i], d_alpha[i]);
//            modelTranslator->stepModel(d_alpha[i], 1);
//
//        }
//    }
//
//    for(int i = 0; i < NUM_ALPHA; i++){
//
//    }
//
//    // If the cost was reduced
//    if(newCost < oldCost){
//
//        // Copy initial data to main data
//        cpMjData(model, mdata, d_init);
//
////        for(int k = 0; k < NUM_CTRL; k++){
////            mdata->ctrl[k] = U_new[0](k);
////        }
//        modelTranslator->setControls(mdata, U_new.at(0), grippersOpen_iLQR[0]);
//
//        cpMjData(model, dArray[0], mdata);
//
//        for(int i = 0; i < ilqr_horizon_length; i++){
//
//            X_final[i] = modelTranslator->returnState(mdata);
//            modelTranslator->setControls(mdata, U_new.at(i), grippersOpen_iLQR[i]);
//            X_old.at(i) = modelTranslator->returnState(mdata);
//
//            modelTranslator->stepModel(mdata, 1);
//            cpMjData(model, dArray[i + 1], mdata);
//        }
//
//        cout << "best alpha was " << alpha << endl;
//        cout << "cost improved in F.P - new cost: " << newCost << endl;
//        m_state termStateBest = modelTranslator->returnState(dArray[ilqr_horizon_length]);
//        if(modelTranslator->taskNumber == 2){
//            double cubeXDiff = termStateBest(7) - modelTranslator->X_desired(7);
//            double cubeYDiff = termStateBest(8) - modelTranslator->X_desired(8);
//            cout << "final cube pos, x: " << termStateBest(7) << ", y: " << termStateBest(8) << endl;
//            cout << "final cube pos, x desired: " << modelTranslator->X_desired(7) << ", y: " << modelTranslator->X_desired(8) << endl;
//            cout << "final cube pos, x diff: " << cubeXDiff << ", y: " << cubeYDiff << endl;
//        }
//        return newCost;
//    }
//
//    return oldCost;
//}

float iLQR::forwardsPassStatic(float oldCost, bool &costReduced){
    float alpha = 1.0;
    float newCost = 0.0;
    bool costReduction = false;
    int alphaCount = 0;
    float alphaReduc = 0.1;
    int alphaMax = (1 / alphaReduc) - 1;
    mjData *d_old;
    d_old = mj_makeData(model);

    while(!costReduction){
        cpMjData(model, mdata, d_init);
        newCost = 0;
        m_state stateFeedback;
        m_state _X;
        m_state X_new;
        m_ctrl _U;

        for(int t = 0; t < numCalcedDerivs; t++){
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
                if(TORQUE_CONTROL){
                    for(int k = 0; k < NUM_CTRL; k++){
                        if(U_new[(t * num_mj_steps_per_control) + i](k) > modelTranslator->torqueLims[k]) U_new[(t * num_mj_steps_per_control) + i](k) = modelTranslator->torqueLims[k];
                        if(U_new[(t * num_mj_steps_per_control) + i](k) < -modelTranslator->torqueLims[k]) U_new[(t * num_mj_steps_per_control) + i](k) = -modelTranslator->torqueLims[k];
                    }
                }


//                cout << "old control: " << endl << U_old[(t * num_mj_steps_per_control) + i] << endl;
//                cout << "state feedback" << endl << stateFeedback << endl;
//                cout << "new control: " << endl << U_new[(t * num_mj_steps_per_control) + i] << endl;

                if(((t * num_mj_steps_per_control) + i) < 5){
                    cout << "new U " << (t * num_mj_steps_per_control) + i << ": " << U_new[(t * num_mj_steps_per_control) + i] << endl;
                }

                modelTranslator->setControls(mdata, U_new[(t * num_mj_steps_per_control) + i], grippersOpen_iLQR[(t * num_mj_steps_per_control) + i]);

                float currentCost;
                if(t == 0){
                    currentCost = modelTranslator->costFunction(mdata, (t * num_mj_steps_per_control) + i, ilqr_horizon_length, d_old);
                }
                else{
                    currentCost = modelTranslator->costFunction(mdata, (t * num_mj_steps_per_control) + i, ilqr_horizon_length, d_old);
                }

                newCost += (currentCost * MUJOCO_DT);

                if(VISUALISE_ROLLOUTS){
                    if(t % 10 == 0){
                        mjrRect viewport = { 0, 0, 0, 0 };
                        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

                        // update scene and render
                        mjv_updateScene(model, mdata, &opt, NULL, &cam, mjCAT_ALL, &scn);
                        mjr_render(viewport, &scn, &con);

                        // swap OpenGL buffers (blocking call due to v-sync)
                        glfwSwapBuffers(window);

                        // process pending GUI events, call GLFW callbacks
                        glfwPollEvents();
                    }
                }
                cpMjData(model, d_old, mdata);
                modelTranslator->stepModel(mdata, 1);
            }
        }

//        cout << "cost from alpha: " << alphaCount << ": " << newCost << endl;

        if(newCost < oldCost){
            costReduction = true;
            costReduced = true;
        }
        else{
            alpha = alpha - 0.1;
            alphaCount++;
            if(alphaCount >= alphaMax){
                break;
            }
        }
    }

    mj_deleteData(d_old);

    if(newCost < oldCost){

        cpMjData(model, mdata, d_init);

//        for(int k = 0; k < NUM_CTRL; k++){
//            mdata->ctrl[k] = U_new[0](k);
//        }
        modelTranslator->setControls(mdata, U_new.at(0), grippersOpen_iLQR[0]);

        cpMjData(model, dArray[0], mdata);

        for(int i = 0; i < ilqr_horizon_length; i++){

            X_final[i] = modelTranslator->returnState(mdata);
            modelTranslator->setControls(mdata, U_new.at(i), grippersOpen_iLQR[i]);
            X_old.at(i) = modelTranslator->returnState(mdata);

            modelTranslator->stepModel(mdata, 1);
            cpMjData(model, dArray[i + 1], mdata);
        }

        cpMjData(model, dArray[ilqr_horizon_length], mdata);

        m_state terminalState = modelTranslator->returnState(mdata);
        if(modelTranslator->taskNumber == 2){
            double cubeXDiff = terminalState(7) - modelTranslator->X_desired(7);
            double cubeYDiff = terminalState(8) - modelTranslator->X_desired(8);
            cout << "final cube pos, x: " << terminalState(7) << ", y: " << terminalState(8) << endl;
            cout << "final cube pos, x desired: " << modelTranslator->X_desired(7) << ", y: " << modelTranslator->X_desired(8) << endl;
            cout << "final cube pos, x diff: " << cubeXDiff << ", y: " << cubeYDiff << endl;
        }

//        cout << "best alpha was " << alpha << endl;
//        cout << "cost improved in F.P - new cost: " << newCost << endl;
        return newCost;
    }

    m_state terminalState = modelTranslator->returnState(mdata);
    if(modelTranslator->taskNumber == 2){
        double cubeXDiff = terminalState(7) - modelTranslator->X_desired(7);
        double cubeYDiff = terminalState(8) - modelTranslator->X_desired(8);
        cout << "final cube pos, x: " << terminalState(7) << ", y: " << terminalState(8) << endl;
        cout << "final cube pos, x desired: " << modelTranslator->X_desired(7) << ", y: " << modelTranslator->X_desired(8) << endl;
        cout << "final cube pos, x diff: " << cubeXDiff << ", y: " << cubeYDiff << endl;
    }

    return oldCost;
}

bool iLQR::checkForConvergence(float newCost, float oldCost, bool costReduced){
    bool convergence = false;
    m_state terminalState = modelTranslator->returnState(mdata);

//    if(modelTranslator->taskNumber == 2){
//        double cubeXDiff = terminalState(7) - modelTranslator->X_desired(7);
//        double cubeYDiff = terminalState(8) - modelTranslator->X_desired(8);
//        cout << "final cube pos, x: " << terminalState(7) << ", y: " << terminalState(8) << endl;
//        cout << "final cube pos, x desired: " << modelTranslator->X_desired(7) << ", y: " << modelTranslator->X_desired(8) << endl;
//        cout << "final cube pos, x diff: " << cubeXDiff << ", y: " << cubeYDiff << endl;
//    }

    std::cout << "New cost: " << newCost <<  std::endl;
    std::cout << "--------------------------------------------------" <<  std::endl;

    numIterations++;
    float costGrad = (oldCost - newCost)/newCost;

    if(numIterations > 2 and (costGrad < epsConverge)) {
        convergence = true;
        cout << "ilQR converged, num Iterations: " << numIterations << " final cost: " << newCost << endl;
        float linTimeSum = 0.0f;
        for(int i = 0; i < numIterations; i++){
            linTimeSum += linTimes[i];
        }
        avgLinTime = linTimeSum / numIterations;
        finalCost = newCost;
        //cubeTermPos(0) = cubeX;
        //cubeTermPos(1) = cubeY;
        cout << "average time linearising " << linTimeSum / numIterations << endl;
    }

    // store new controls
    if(costReduced){
        for(int i = 0; i < ilqr_horizon_length; i++){
            U_old[i] = U_new[i].replicate(1, 1);
        }
    }
//    else{
//        lamda *= lamdaFactor;
//    }

    return convergence;
}

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

void iLQR::setInitControls(std::vector<m_ctrl> _initControls, std::vector<bool> _grippersOpen){

    for(int i = 0; i < ilqr_horizon_length; i++){
        initControls[i] = _initControls[i].replicate(1,1);
        U_old[i] = _initControls[i].replicate(1,1);
        U_new[i] = _initControls[i].replicate(1,1);

//        grippersOpen_iLQR[i] = _grippersOpen[i];
        grippersOpen_iLQR[i] = false;
    }
}

void iLQR::makeDataForOptimisation(){

    for(int i = 0; i <= MUJ_STEPS_HORIZON_LENGTH; i++){
        // populate dArray with mujoco data objects from start of trajec until end
        dArray[i] = mj_makeData(model);
    }

    dArray[MUJ_STEPS_HORIZON_LENGTH] = mj_makeData(model);
    cpMjData(model, dArray[MUJ_STEPS_HORIZON_LENGTH], mdata);

//    for(int i = 0; i < NUM_ALPHA; i++){
//        d_alpha[i] = mj_makeData(model);
//        d_alpha_last[i] = mj_makeData(model);
//    }

}

void iLQR::deleteMujocoData(){
    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
        mj_deleteData(dArray[i]);
    }
}

void iLQR::updateNumStepsPerDeriv(int stepPerDeriv){
    num_mj_steps_per_control = stepPerDeriv;
    numCalcedDerivs = ilqr_horizon_length / num_mj_steps_per_control;
    numIterations = 0;
    linTimes.clear();
    numEvals.clear();

}