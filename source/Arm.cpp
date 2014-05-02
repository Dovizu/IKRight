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
    float length, x, y, z, angle; //normalized x, y, z and angle
    GLUquadricObj *quadric = gluNewQuadric();
public:
    Link(float angle, Vector3f& axis, float length) {
        Vector3f nAxis = axis.normalized();
        x=nAxis(0); y=nAxis(1); z=nAxis(2);
        this->angle = angle;
        this->length = length;
    }
    
    Transform3f tf() {
        Transform3f t = Transform3f(Translation3f(0,0,length));
        t = Transform3f(AngleAxisf(angle, Vector3f(x,y,z)))*t;
        return t;
    }
};

#pragma mark - Arm Class

/**
 *  Construct an Arm by iterating through LinkData structs
 */
Arm::Arm(vector<LinkInfo>& linkData, Vector3f& root) {
    rootPos = root;
    for (int i=0; i<linkData.size(); ++i) {
        LinkInfo& lData = linkData[i];
        Link* link = new Link(lData.angle, lData.axis, lData.length);
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
    Vector3f pos = Vector3f::Zero();
    for (int li=0; li<links.size(); ++li) {
        pos = links[li]->tf()*pos;
    }
    pos = Translation3f(rootPos(0),rootPos(1),rootPos(2))*pos;
    return pos;
}
/**
 *  moves the arm by the specified list of angle deltas
 *  @param angles list of AnglePairs that describe the delta of each angle
 */


/*
void Arm::moveby(vector<AnglePair>& angles) {
    ASSERT(angles.size()==links.size(), "Num of angles doesn't match num of links");
    for (int i=0; i<angles.size(); ++i) {
        links[i]->theta += angles[i].first;
        links[i]->phi += angles[i].second;
    }
}
 */

/**
 *  Graph the entire arm using OpenGL
 */


void Arm::graph() {
    //need to Assert that OPENGL is initialized and working
    glPushMatrix();
    glColor3f(0.0f, 1.0f, 1.0f);
    glutSolidSphere(2, 20, 20);
    for (int li=links.size()-1; li>=0; --li) {
        glRotatef(degrees(links[li]->angle), links[li]->x,links[li]->y,links[li]->z);
        glColor3f(1.0f, 0.0f, 0.0f);
        gluCylinder(links[li]->quadric, 1, 0, links[li]->length, 20, 20);
        glTranslatef(0,0,links[li]->length);
        glColor3f(0.0f, 1.0f, 0.0f);
        glutSolidSphere(1, 20, 20);
    }
    glTranslatef(rootPos(0),rootPos(1),rootPos(2));
    glPopMatrix();
}

/*
MatrixXf Arm::jacobian() {
    MatrixXf jac = MatrixXf::Zero(3, links.size()*2);
    float theta=0, phi=0;
    for (int li=0; li<links.size(); ++li) {
        //col0 = theta, col1 = phi
        Link& l = links[li];
        theta += l.theta; phi += l.phi;
        //compute partial derivatives
        float pXpTheta = -l.length*sin(theta)*sin(phi);
        float pXpPhi = l.length*cos(theta)*cos(phi);
        float pYpTheta = l.length*cos(theta)*sin(phi);
        float pYpPhi = l.length*sin(theta)*cos(phi);
        float pZpTheta = 0.0f;
        float pZpPhi = -l.length*sin(phi);
        //compute the Jacobian
        for (int colIdx=0; colIdx<li; ++colIdx) {
            jac(0, 2*colIdx) += pXpTheta;
            jac(1, 2*colIdx) += pYpTheta;
            jac(2, 2*colIdx) += pZpTheta;
            jac(0, 2*colIdx+1) += pXpPhi;
            jac(1, 2*colIdx+1) += pYpPhi;
            jac(2, 2*colIdx+1) += pZpPhi;
        }
    }
}
 
 */
