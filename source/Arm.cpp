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
    
    Matrix3f rotationMat() {
        Matrix3f rm;
        rm = AngleAxisf(angle, Vector3f(x,y,z));
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
void Arm::moveby(Vector3f& deltas) {
    ASSERT(deltas.size()/3==links.size(), "Num of angles doesn't match num of links");
    for (int i=0; i<links.size(); ++i) {
        links[i]->x+=deltas[i*3+0];
        links[i]->y+=deltas[i*3+1];
        links[i]->z+=deltas[i*3+2];
    }
}

void Arm::unmove(Vector3f& deltas) {
    ASSERT(deltas.size()/3==links.size(), "Num of angles doesn't match num of links");
    for (int i=0; i<links.size(); ++i) {
        links[i]->x-=deltas[i*3+0];
        links[i]->y-=deltas[i*3+1];
        links[i]->z-=deltas[i*3+2];
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

MatrixXf Arm::jacobian() {
    MatrixXf jac = MatrixXf::Zero(3, links.size()*3);
    //for each Jacobian block or each link
    
    for (int li=links.size()-1; li>=0; --li) {
        int ji = li*3;
        Vector3f localVec = links[li]->tf()*Vector3f::Zero();
        Matrix3f localJac;
        localJac <<     0, localVec(2), -localVec(1),
                        -localVec(2), 0, localVec(0),
                        localVec(1), -localVec(0), 0;
        Matrix3f t = Matrix3f::Identity();
        for (int i=links.size()-1; i>=0; --i) {
            if (i==li) {
                t = t*localJac;
            }else{
                t = t*links[i]->rotationMat();
            }
        }
//        cout << "Local Jacobian of link " << li << " is this: " << endl;
//        cout << "ji is " << ji << endl;
//        cout << jac.block(0,ji,3,3) << endl;
        jac.block(0,ji,3,3) << t;
    }
    return jac;
}

MatrixXf Arm::pseudoInverse() {
    MatrixXf j = jacobian();
    MatrixXf jjtInv = (j * j.transpose());
    jjtInv = jjtInv.inverse();
    
    return (j.transpose() * jjtInv);
}

bool Arm::update(Vector3f& g) {
    MatrixXf j_inv = pseudoInverse();
    Vector3f p = position();
    Vector3f deltas = Vector3f::Zero(links.size()*3, 1);
    //newton's method
    Vector3f deltaP = p - g;
    
    moveby(deltas);
    graph();
    return (position()-g).norm() < tolerance;
}
