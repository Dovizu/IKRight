//
//  Arm.h
//  IKRight
//
//  Copyright (c) 2014 Leo C & Donny R. All rights reserved.
//

#ifndef __IKRight__Arm__
#define __IKRight__Arm__

#include "utilities.h"

typedef struct {
    float length;
    float theta;
    float phi;
} LinkInfo;
typedef pair<float, float> AnglePair;
#define degrees(x) x*180/3.1415926

class Link;

class Arm {
    size_t numLinks = 0;
    Link* endLink;
public:
    Arm(vector<LinkInfo>& linkData, Vector3f& root);
    size_t size();
    Vector3f position();
    void moveby(vector<AnglePair>& angles);
    void graph();
};

#endif /* defined(__IKRight__Arm__) */
