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
    float theta=0.0, phi=0.0, length=0.0; //l stands for local
    GLUquadricObj *quadric = gluNewQuadric();
};

#pragma mark - Arm Class

/**
 *  Construct an Arm by iterating through LinkData structs
 */
Arm::Arm(vector<LinkInfo>& linkData, Vector3f& root) {
    rootPos = root;
    for (int i=0; i<linkData.size(); ++i) {
        LinkInfo& lData = linkData[i];
        Link* link = new Link();
        link->theta = lData.theta;
        link->phi = lData.phi;
        link->length = lData.length;
        links.push_back(link);
    }
}

/**
 *  @return the size of links excluding root
 */
size_t Arm::size() {return links.size();}

/**
 *  @return the end effector's position
 */
Vector3f Arm::position() {
    Vector3f pos = rootPos;
    float theta=0;
    float phi=0;
    float l=0;
    for (int i=0; i<links.size(); ++i) {
        theta += links[i]->theta;
        phi += links[i]->phi;
        l = links[i]->length;
        pos(0) += l*cos(theta)*sin(phi);
        pos(1) += l*sin(theta)*sin(phi);
        pos(2) += l*cos(phi);
    }
    return pos;
}
/**
 *  moves the arm by the specified list of angle deltas
 *  @param angles list of AnglePairs that describe the delta of each angle
 */

void Arm::moveby(vector<AnglePair>& angles) {
    ASSERT(angles.size()==links.size(), "Num of angles doesn't match num of links");
    for (int i=0; i<angles.size(); ++i) {
        links[i]->theta += angles[i].first;
        links[i]->phi += angles[i].second;
    }
}

/**
 *  Graph the entire arm using OpenGL
 */
void Arm::graph() {
    //need to Assert that OPENGL is initialized and working
    Vector3f pos = rootPos;
    glColor3f(0.0f, 1.0f, 1.0f);
    glutSolidSphere(2, 20, 20);
    
    float theta=0, phi=0, l=0;
    for (int i=0; i<links.size(); ++i) {
        
        theta += links[i]->theta;
        phi += links[i]->phi;
        l = links[i]->length;
        
        glPushMatrix();
        glTranslatef(pos(0), pos(1), pos(2));
        glRotatef(degrees(theta), 0.0, 0.0, 1.0);
        glRotatef(degrees(phi), 0.0, 1.0, 0.0);
        glColor3f(1.0f, 0.0f, 0.0f);
        gluCylinder(links[i]->quadric, 1, 0, l, 20, 20);
        glColor3f(0.0f, 1.0f, 0.0f);
        glutSolidSphere(1, 20, 20);
        glPopMatrix();
        
        pos(0) += l*cos(theta)*sin(phi);
        pos(1) += l*sin(theta)*sin(phi);
        pos(2) += l*cos(phi);
    }
}

MatrixXf Arm::jacobian() {
    MatrixXf jac = MatrixXf::Zero(3, links.size()*2);
    float theta=0, phi=0;
    for (int li=0; li<links.size(); ++li) {
        //col0 = theta, col1 = phi
        Link& l = *links[li];
        theta += l.theta; phi += l.phi;
        //compute partial derivatives
        float pXpTheta = -l.length*sin(theta)*sin(phi);
        float pXpPhi = l.length*cos(theta)*cos(phi);
        float pYpTheta = l.length*cos(theta)*sin(phi);
        float pYpPhi = l.length*sin(theta)*cos(phi);
        float pZpTheta = 0.0f;
        float pZpPhi = -l.length*sin(phi);
        //compute the Jacobian
        for (int colIdx=0; colIdx<=li; ++colIdx) {
            jac(0, 2*colIdx) += pXpTheta;
            jac(1, 2*colIdx) += pYpTheta;
            jac(2, 2*colIdx) += pZpTheta;
            jac(0, 2*colIdx+1) += pXpPhi;
            jac(1, 2*colIdx+1) += pYpPhi;
            jac(2, 2*colIdx+1) += pZpPhi;
        }
    }
    return jac;
}
