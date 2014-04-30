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
    string options = "--testOpenGL(0)"; //test OpenGL, GLEW, and GLFW
    options.append("--testBEZParser(0)"); //test BEZParser for files at "bezFiles/"

    getCmdLineOptions(argc, argv, options, &results);

    string directoryName;
    string fileName;

    return 0;
}

