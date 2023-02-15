//
// Created by dave on 27/01/23.
//

#include "saveData.h"

saveData::saveData(){

}

void saveData::saveCylinderDiffToCSV(std::vector<double> cubeDiffs, std::string fileName){
    ofstream outputCylinderDiff;
    outputCylinderDiff.open(fileName);
    for(int i = 0; i < cubeDiffs.size(); i++){
        outputCylinderDiff << cubeDiffs[i] << endl;
    }
    outputCylinderDiff.close();
}

void saveData::saveTrajecToCSV(iLQR* optimiser){

    for(int i = 0; i < MUJ_STEPS_HORIZON_LENGTH; i++){
        outputFile << i << ",";

        for(int j = 0; j < NUM_CTRL; j++){
            outputFile << optimiser->finalControls[i](j) << ",";
        }

        for(int j = 0; j < NUM_CTRL; j++){
            outputFile << optimiser->initControls[i](j) << ",";
        }

        for(int j = 0; j < DOF; j++){
            outputFile << optimiser->X_final[i](j) << ",";
        }

        for(int j = 0; j < DOF; j++){
            outputFile << optimiser->X_final[i](j+DOF) << ",";
        }
        outputFile << endl;
    }

    outputFile.close();
}

void saveData::saveMatricesToCSV(){
    int size = A_matrices.size();
    cout << "size is" << size << endl;
    outputMatrices.open(matricesFileName);
    for(int i = 0; i < size; i++){
        for(int j = 0; j < (DOF); j++){
            for(int k = 0; k < (2 * DOF); k++){
                outputMatrices << A_matrices[i](j + DOF, k);
                if(k == (2 * DOF) - 1){
                    outputMatrices << endl;
                }
                else{
                    outputMatrices << ",";
                }
            }

        }
    }
    outputMatrices.close();

    outputBMatrices.open("matrices_B.csv");

    for(int i = 0; i < size; i++){
        for(int j = 0; j < (DOF); j++){
            for(int k = 0; k < NUM_CTRL; k++){
                outputBMatrices << B_matrices[i](j + DOF, k);
                if(k == NUM_CTRL - 1){
                    outputBMatrices << endl;
                }
                else{
                    outputBMatrices << ",";
                }
            }
        }
    }

    outputBMatrices.close();

    outputMatrices.open("savedStates.csv");
    for(int i = 0; i < size; i++){
        for(int j = 0; j < (DOF * 2); j++)
        {
            outputMatrices << savedStates[i](j);
            if(j == (2 * DOF) - 1){
                outputMatrices << endl;
            }
            else{
                outputMatrices << ",";
            }
        }
    }

    outputBMatrices.close();

    outputBMatrices.open("savedControls.csv");

    for(int i = 0; i < size; i++){
        for(int j = 0; j < NUM_CTRL; j++) {
            outputBMatrices << savedControls[i](j);

            if(j == NUM_CTRL - 1){
                outputBMatrices << endl;
            }
            else{
                outputBMatrices << ",";
            }
        }
    }

    outputBMatrices.close();
}

void saveData::saveDataCollectionToCSV(int taskNumber){

    std::string root;
    std::string filePath;

    if(COPYING_DERIVS){
        root = "Testing_Data/Copying/";
    }
    else if(LINEAR_INTERP_DERIVS){
        root = "Testing_Data/Linear_Interpolation/";
    }
    else if(DYNAMIC_LINEAR_DERIVS){
        root = "Testing_Data/Dynamic_Linear_Interpolation/";
    }

    filePath = root + DataCollecNames[taskNumber];
    cout << "filepath: " << filePath << endl;

    outputDataCollection.open(filePath);

    if(!DYNAMIC_LINEAR_DERIVS){
        for(int i = 0; i < NUM_TESTS_EXTRAPOLATION; i++){
            outputDataCollection << "num Iterations" << ",";
        }
        for(int i = 0; i < NUM_TESTS_EXTRAPOLATION; i++){
            outputDataCollection << "optimisation time" << ",";
        }
        for(int i = 0; i < NUM_TESTS_EXTRAPOLATION; i++){
            outputDataCollection << "cost reduction" << ",";
        }
        for(int i = 0; i < NUM_TESTS_EXTRAPOLATION; i++){
            outputDataCollection << "Mean Time Linearising" << ",";
        }

        outputDataCollection << endl;

        for(int i = 0; i < 4; i++){
            for(int j = 0; j < NUM_TESTS_EXTRAPOLATION; j++){
                outputDataCollection << scalingLevels[j] << ",";
            }
        }
        outputDataCollection << endl;

        for(int i = 0; i < NUM_TRAJECTORIES_DATA_COLLECTION; i++){
            for(int j = 0; j < NUM_TESTS_EXTRAPOLATION; j++){
                outputDataCollection << numIterations[i][j] << ",";
            }

            for(int j = 0; j < NUM_TESTS_EXTRAPOLATION; j++){
                outputDataCollection << iLQRTime[i][j] << ",";
            }

            for(int j = 0; j < NUM_TESTS_EXTRAPOLATION; j++){
                float costReduction = 0.0f;
                costReduction = 1 - (finalsCosts[i][j]/initCosts[i][j]);
                outputDataCollection << costReduction << ",";
            }

            for(int j = 0; j < NUM_TESTS_EXTRAPOLATION; j++){
                outputDataCollection << avgLinTime[i][j] << ",";
            }

            outputDataCollection << endl;
        }
    }
    else{

        outputDataCollection << "num Iterations" << ",";
        outputDataCollection << "optimisation time" << ",";
        outputDataCollection << "cost reduction" << ",";
        outputDataCollection << "Mean Linearising Time" << ",";
        outputDataCollection << "Mean Number of Evals" << ",";
        outputDataCollection << "Mean Variance of EVals" << ",";
        outputDataCollection << endl;

        for(int i = 0; i < NUM_TRAJECTORIES_DATA_COLLECTION; i++){
            outputDataCollection << numIterations[i][0] << ",";

            outputDataCollection << iLQRTime[i][0] << ",";

            float costReduction = 0.0f;
            costReduction = 1 - (finalsCosts[i][0]/initCosts[i][0]);
            outputDataCollection << costReduction << ",";


            outputDataCollection << avgLinTime[i][0] << ",";
            outputDataCollection << avgNumEvaluations[i][0] << ",";

            outputDataCollection << avgEvalsVariance[i][0] << ",";


            outputDataCollection << endl;
        }
    }



    outputDataCollection.close();
}

void saveData::saveTestingData(int numTestingData, const std::string& name){
    outputDataCollection.open(name);

    for(int i = 0; i < numTestingData; i++){
        for(int j = 0; j < (2 * DOF); j++){
            outputDataCollection << testing_X0[i](j) << ",";
        }
        for(int j = 0; j < (2 * DOF); j++){
            outputDataCollection << testing_X_Desired[i](j) << ",";
        }
        outputDataCollection << endl;
    }

    outputDataCollection.close();
}
