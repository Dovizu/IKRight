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
    Vector3f axis;
    float angle;
} LinkInfo;

#define degrees(x) x*180/3.1415926

class Link;
class Arm {
    vector<Link*> links; //links from end effector to root in reverse order
    Vector3f rootPos = Vector3f(0,0,0);
    float tolerance = 1;
    float step = 0.5;
public:
    Arm(vector<LinkInfo>& linkData, Vector3f& root);
    //need to write destructor
    size_t size();
    Vector3f position();
    void moveby(VectorXf& deltas);
    void unmove(VectorXf& deltas);
    void graph();
    MatrixXf jacobian();
    MatrixXf pseudoInverse();
    bool update(Vector3f& g);
};

#endif /* defined(__IKRight__Arm__) */
