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
    float length; //normalized x, y, z and angle
    Vector3f r;
    GLUquadricObj *quadric = gluNewQuadric();
public:
    Link(float angle, Vector3f& axis, float length) {
        Vector3f nAxis = axis.normalized();
        r = axis;
        r = r.normalized()*angle;
        this->length = length;
    }
    
    Transform3f tf() {
        Transform3f t = Transform3f(Translation3f(0,0,length));
        t = Transform3f(AngleAxisf(r.norm(), r.normalized()))*t;
        return t;
    }
    
    Matrix3f rotationMat() {
        Matrix3f rm;
        rm = AngleAxisf(r.norm(), r.normalized());
        return rm;
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
 *  moves the arm by the specified list of deltas of rx, ry and rz
 *  @param deltas: a list of deltas in order of rx, ry and rz, each tuple of three should correspond to a link
 *  @invarinace: size of deltas list should be three times links.size()
 */
void Arm::moveby(VectorXf& deltas) {
    ASSERT(deltas.size()/3==links.size(), "Num of angles doesn't match num of links");
    for (int i=0; i<links.size(); ++i) {
        links[i]->r(0) += deltas[i*3+0];
        links[i]->r(1) += deltas[i*3+1];
        links[i]->r(2) += deltas[i*3+2];
    }
}

void Arm::unmove(VectorXf& deltas) {
    ASSERT(deltas.size()/3==links.size(), "Num of angles doesn't match num of links");
    for (int i=0; i<links.size(); ++i) {
        links[i]->r(0) -= deltas[i*3+0];
        links[i]->r(1) -= deltas[i*3+1];
        links[i]->r(2) -= deltas[i*3+2];
    }
}

/**
 *  Graph the entire arm using OpenGL
 */


void Arm::graph() {
    //need to Assert that OPENGL is initialized and working
    glPushMatrix();
    glColor3f(0.0f, 1.0f, 1.0f);
    glutSolidSphere(2, 20, 20);
    for (int li=links.size()-1; li>=0; --li) {
        glRotatef(degrees(links[li]->r.norm()), links[li]->r(0),links[li]->r(1),links[li]->r(2));
        glColor3f(1.0f, 0.0f, 0.0f);
        gluCylinder(links[li]->quadric, 1, 0, links[li]->length, 20, 20);
        glTranslatef(0,0,links[li]->length);
        glColor3f(0.0f, 1.0f, 0.0f);
        glutSolidSphere(1, 20, 20);
    }
    glTranslatef(rootPos(0),rootPos(1),rootPos(2));
    glPopMatrix();
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
//        cout << "Local Jacobian of link " << li << " is this: " << endl;
//        cout << "ji is " << ji << endl;
//        cout << jac.block(0,ji,3,3) << endl;
        jac.block(0,ji,3,3) << t;
    }
    return jac;
}
