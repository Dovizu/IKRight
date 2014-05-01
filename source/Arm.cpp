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
    GLUquadricObj *quadric = gluNewQuadric();
public:
    /**
     *  @return angle relative to root
     */
    virtual float theta()   {return ltheta+parent->theta();}
    virtual float phi()     {return lphi+parent->phi();}
    
    /**
     *  @return the position relative to root
     *  @discussion
     *  x = r*cos(theta)*sin(phi)
     *  y = r*sin(theta)*sin(phi)
     *  z = r*cos(phi)
     */
    virtual Vector3f position() {
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
    virtual void moveby(vector<AnglePair>& angles) {
        ltheta += angles.back().first;
        lphi += angles.back().second;
        angles.pop_back();
        parent->moveby(angles);
    }
    
    /**
     *  Draws itself on screen
     */
    virtual void draw() {
        
        //this algorithm is super inefficient, need improvement
        parent->draw();
        glPushMatrix();
        
        Vector3f pos = parent->position();
        glTranslatef(pos(0), pos(1), pos(2));
        
        glRotatef(degrees(theta()), 0.0, 0.0, 1.0);
        glRotatef(degrees(phi()), 0.0, 1.0, 0.0);
        glColor3f(1.0f, 0.0f, 0.0f);
        gluCylinder(quadric, 1, 0, length, 20, 20);
 
        pos = position();
        glColor3f(0.0f, 1.0f, 0.0f);
        glutSolidSphere(1, 20, 20);
        glPopMatrix();
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
    void draw() {
        glColor3f(0.0f, 1.0f, 0.0f);
        glutSolidSphere(1, 20, 20);
    }
};

#pragma mark - Arm Class

/**
 *  Construct an Arm by iterating through LinkData structs
 */
Arm::Arm(vector<LinkInfo>& linkData, Vector3f& root) {
    Link *parent = new Root(root);
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

/**
 *  @return the size of links excluding root
 */
size_t Arm::size() {return numLinks;}

/**
 *  @return the end effector's position
 */
Vector3f Arm::position() {
    return endLink->position();
}
/**
 *  moves the arm by the specified list of angle deltas
 *  @param angles list of AnglePairs that describe the delta of each angle
 */
void Arm::moveby(vector<AnglePair>& angles) {
    endLink->moveby(angles);
}

/**
 *  Graph the entire arm using OpenGL
 */
void Arm::graph() {
    //need to Assert that OPENGL is initialized and working
    endLink->draw();
}