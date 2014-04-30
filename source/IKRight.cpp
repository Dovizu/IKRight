//
//  IKRight.cpp
//  IKRight
//  Copyright (c) 2014 Leo C & Donny R. All rights reserved.


#include "IKRight.h"
/*
 +––––––––––––––––––––––––––––––––––––––+
 |Main Functions                        |
 +––––––––––––––––––––––––––––––––––––––+
 */
/**
 *  IKRight Main Function
 *
 *  @param argc default argc
 *  @param argv default argv
 *
 *  @return status code
 */
int main(int argc, char *argv[]) {
    vector<CmdLineOptResult> *results;
    string basePath;
    string options = "--testArm(0)"; //test Arm functionality

    getCmdLineOptions(argc, argv, options, &results);

    string directoryName;
    string fileName;
    
    for (auto& result : *results) {
        if (result.optName.compare("--testArm")==0) {
            testArm();
        }
    }

    return 0;
}

