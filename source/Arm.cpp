//
//  Arm.cpp
//  IKRight
//
//  Copyright (c) 2014 Leo C & Donny R. All rights reserved.
//

#include "Arm.h"

#pragma mark - Link and Root Class

class Link {
    friend class Arm;
protected:
    float ltheta=0.0, lphi=0.0, length=0.0; //l stands for local
    Link* parent; //inbound link
public:
    /**
     *  @return angle relative to root
     */
    float theta()   {return ltheta+parent->theta();}
    float phi()     {return lphi+parent->phi();}
    
    /**
     *  @return the position relative to root
     *  @discussion
     *  x = r*cos(theta)*sin(phi)
     *  y = r*sin(theta)*sin(phi)
     *  z = r*cos(phi)
     */
    Vector3f position() {
        return Vector3f(length*cos(ltheta+parent->theta())*sin(lphi+parent->phi()), //x
                        length*sin(ltheta+parent->theta())*sin(lphi+parent->phi()), //y
                        length*cos(lphi+parent->phi()))                             //z
                        + parent->position();                                       //parent
    }
    
    /**
     *  Move by specified theta and phi
     *
     *  @param angles: a list of theta-phi pairs, the most back is the most outbound
     *  @warning: angles.size() == number of links from this to parents
     */
    void moveby(vector<AnglePair>& angles) {
        ltheta += angles.back().first;
        lphi += angles.back().second;
        angles.pop_back();
        parent->moveby(angles);
    }
};

/**
 *  Root functions as a sentinel in the linked structure
 */
class Root : public Link {
    Vector3f rootPos = Vector3f(0,0,0);
public:
    Root(Vector3f pos) {rootPos = pos;}
    float theta()   {return 0.0;} //base case
    float phi()     {return 0.0;} //base case
    Vector3f position() {return rootPos;}
    void moveby(vector<AnglePair>& angles) {
        ASSERT(angles.size()==0, "more angles to move than there are links");
    }
};

#pragma mark - Arm Class

size_t Arm::size() {return numLinks;}

/**
 *  Construct an Arm by iterating through LinkData structs
 */
Arm::Arm(vector<LinkInfo>& linkData, Vector3f& root) {
    Link *parent = new Root(root); ++numLinks;
    for (auto& lData : linkData) {
        Link* link = new Link();
        link->ltheta = lData.theta;
        link->lphi = lData.phi;
        link->length = lData.length;
        link->parent = parent;
        parent = link;
        ++numLinks;
    }
    endLink = parent;
}
