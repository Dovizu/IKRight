//
//  UnitTest.cpp
//  IKRight
//
//  Created by Donny Reynolds on 4/4/14.
//  Copyright (c) 2014 Leo C & Donny R. All rights reserved.
//
#ifndef __IKRight__UnitTest__
#define __IKRight__UnitTest__

#include "utilities.h"
#include "Arm.h"

void testArm() {
    /*
    typedef struct {
        float length;
        float theta;
        float phi;
    } LinkInfo;
     */
    vector<LinkInfo> links;
    Vector3f root(0,0,0);
    LinkInfo link1 = {1, M_PI/4, M_PI/2};
    LinkInfo link2 = {1, -M_PI/4, -M_PI/2};
    LinkInfo link3 = {1, 3*M_PI/4, M_PI/2};
    links.push_back(link1);
    links.push_back(link2);
    links.push_back(link3);
    Arm arm(links, root);
    cout << "Should be: (0, 1.4, 1)" << endl;
    cout << arm.position() << endl;
}

#endif