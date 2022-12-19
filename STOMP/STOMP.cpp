//
// Created by dave on 19/12/22.
//

#include "STOMP.h"

STOMP* optimiser_stomp;
extern mjvCamera cam;                   // abstract camera
extern mjvScene scn;                    // abstract scene
extern mjvOption opt;			        // visualization options
extern mjrContext con;				    // custom GPU context
extern GLFWwindow *window;

STOMP::STOMP(mjModel* m, mjData* d, taskTranslator* _modelTranslator){
    model = m;
    modelTranslator = _modelTranslator;
    mdata = mj_makeData(model);
    cpMjData(model, mdata, d);

    d_init = mj_makeData(model);

    for(int j = 0; j < MUJ_STEPS_HORIZON_LENGTH; j++) {
        U_best.push_back(m_ctrl());
    }

    for(int i = 0; i < ROLLOUTS_PER_ITERATION; i++){
        d_Array[i] = mj_makeData(model);
    }
}

std::vector<m_ctrl> STOMP::optimise(std::vector<m_ctrl> U_init){
    double bestCostSoFar = rolloutTrajectory(mdata, U_init);
    cout << "cost of initial trajectory: " << bestCostSoFar << endl;

    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
        U_best[i] = U_init[i].replicate(1, 1);
    }

    for(int i = 0; i < maxIterations; i++){
        bool betterTrajecFound;
        double newBestCost;

        U_best = generateAndEvaluateNoisyTrajectories(U_best, bestCostSoFar, betterTrajecFound, newBestCost);

        cout << "------------------------------------------------ " << endl;
        cout << "iteration " << numIterations << endl;
        if(newBestCost < bestCostSoFar){
            cout << "new best cost: " << newBestCost << endl;
            if(checkForConvergence(bestCostSoFar, newBestCost)){
                break;
            }
            else{
                bestCostSoFar = newBestCost;
            }
        }
        else{
            // do nothing
        }

        numIterations++;
    }

    cout << "finished stomp optimisation" << endl;
    return U_best;

}

std::vector<m_ctrl> STOMP::generateAndEvaluateNoisyTrajectories(std::vector<m_ctrl> U_best, double oldBestCost, bool &betterTrajecFound, double &newBestCost){
    double newTrajecsCosts[ROLLOUTS_PER_ITERATION];
    std::vector<m_ctrl> noisyTrajecs[ROLLOUTS_PER_ITERATION];
    betterTrajecFound = false;

    cpMjData(model, mdata, d_init);
    for(int i = 0; i < ROLLOUTS_PER_ITERATION; i++){
        cpMjData(model, d_Array[i], d_init);
    }

    //#pragma omp parallel for default(none)
    for(int j = 0; j < ROLLOUTS_PER_ITERATION; j++){
        noisyTrajecs[j] = generateNoisyTrajectory(U_best);
        newTrajecsCosts[j] = rolloutTrajectory(d_Array[j], noisyTrajecs[j]);
    }

//    cout << "new trajecs evaluated" << endl;
//    cout << "------------ cost of new trajecs ----------------" << endl;

    newBestCost = oldBestCost;
    int bestTrajecIndex = 0;
    for(int j = 0; j < ROLLOUTS_PER_ITERATION; j++){
        //cout << "cost: " << newTrajecsCosts[j] << endl;
        if(newTrajecsCosts[j] < newBestCost){
            betterTrajecFound = true;
            newBestCost = newTrajecsCosts[j];
            bestTrajecIndex = j;
        }
    }

    if(betterTrajecFound){
        return noisyTrajecs[bestTrajecIndex];
    }
    else{
        return U_best;
    }
}

double STOMP::rolloutTrajectory(mjData *d, std::vector<m_ctrl> U){
    double cost = 0.0f;

    cpMjData(model, mdata, d_init);
    mjData *d_old;
    d_old = mj_makeData(model);

    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++) {
        modelTranslator->setControls(mdata, U[i], false);
        float stateCost;
        if (i == 0) {
            stateCost = modelTranslator->costFunction(mdata, i, MUJ_STEPS_HORIZON_LENGTH, d_init);
        } else {
            stateCost = modelTranslator->costFunction(mdata, i, MUJ_STEPS_HORIZON_LENGTH, d_old);
        }

        cost += (stateCost * MUJOCO_DT);
        cpMjData(model, d_old, mdata);
        modelTranslator->stepModel(mdata, 1);

        if(VISUALISE_ROLLOUTS_STOMP){
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

    mj_deleteData(d_old);

//    m_state termState = modelTranslator->returnState(mdata);
//    if(modelTranslator->taskNumber == 2){
//        double cubeXDiff = termState(7) - modelTranslator->X_desired(7);
//        double cubeYDiff = termState(8) - modelTranslator->X_desired(8);
//        cout << "final cube pos, x diff: " << cubeXDiff << ", y: " << cubeYDiff << endl;
//    }
    //cout << "--------------------------------------------------" << endl;
    cpMjData(model, mdata, d_init);

    return cost;
}

std::vector<m_ctrl> STOMP::generateNoisyTrajectory(std::vector<m_ctrl> U_best){
    std::vector<m_ctrl> U_noisy;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);

    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){

        m_ctrl noisyControl;
        for(int j = 0; j < NUM_CTRL; j++){
            std::normal_distribution<double> dist(0.0, noiseProfile[j]);
            double noiseVal = dist(generator);
            noisyControl(j) = U_best[i](j) + noiseVal;
        }
        U_noisy.push_back(noisyControl);
    }

    return U_noisy;
}

bool STOMP::checkForConvergence(double oldCost, double newCost){
    bool converged = false;

    float costGrad = (oldCost - newCost)/newCost;

    if(numIterations > 2 and (costGrad < epsConverge)) {
        converged = true;
    }

    return converged;
}

void STOMP::initialise(mjData *_d_init){
    cpMjData(model, d_init, _d_init);
}
