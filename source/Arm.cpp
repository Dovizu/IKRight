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
        r = axis.normalized()*angle;
        this->length = length;
    }
    
    Transform3f R() {
        return Transform3f(AngleAxisf(r.norm(), r.normalized()));
    }
    
    Transform3f T() {
        return Transform3f(Translation3f(0,0,length));
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
        pos = links[li]->R()*links[li]->T()*pos;
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
    MatrixXf jac = MatrixXf::Zero(3, links.size()*3);
    //for each Jacobian block or each link
    
    /*
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
     */
    
    for (int li=0; li<links.size(); ++li) {
        int ji = li*3;
        Vector4f localVec = Vector4f::Zero(); //most outboard link
        localVec(3) = 1;
        
        int i;
        Matrix4f localJac;
        for (i=0; i<links.size(); ++i) {
            if (i==li) {
                localVec = links[i]->T().matrix()*localVec;
                localJac <<     0, localVec(2), -localVec(1), 0,
                                -localVec(2), 0, localVec(0), 0,
                                localVec(1), -localVec(0), 0, 0,
                                0,          0,      0,      1; //try last number = 1 if doesn't work
                localJac = links[i]->R()*localJac;
            }else if (i < li) {
                localVec = links[i]->R()*links[i]->T()*localVec;
//                localVec = links[i]->T()*localVec;
            }else if (i > li) {
                localJac = links[i]->R()*links[i]->T()*localJac;
//                localJac = links[i]->R()*localJac;
            }
        }
        localJac = Transform3f(Translation3f(rootPos(0),rootPos(1),rootPos(2)))*localJac;
        jac.block(0,ji,3,3) << localJac.block(0,0,3,3);
//        cout << "jacBlock: \n" << jac.block(0,ji,3,3) << endl;
//        cout << "Jacobian: \n" << localJac << endl;
    }
    //need to change the order of move-by
    cout << "Jacobian: \n" << jac << endl;
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
    VectorXf dR = VectorXf::Zero(links.size()*3, 1);

    //newton's method
    Vector3f dP = g-p;
//    Vector4f dP = Vector4f(deltaP(0),deltaP(1),deltaP(2),1);
    
//    Vector4f dP = Vector4f(deltaP, 1);
//    cout << "P is: " << endl << p << endl;
//    if (!(deltaP.norm()<step)) {
//        deltaP = step*deltaP.normalized();
//    }
//    cout << "deltaP is: " <<endl<<deltaP<<endl;
    bool decreased = false;

    dR = j_inv*dP;
    cout << "dR: \n" << dR << endl;
//    cout << "dR norm: " << dR.norm();
    dR.normalize();
//    cout << "normalized dR: \n" << dR << endl;
    dR = step*dR;
    cout << "step*dR: \n" << dR << endl;


    int MAX = 10;
    int count = 0;
    while (!decreased && count < MAX) {
        moveby(dR);
        Vector3f newP = position();
        cout << "New position: " << newP << endl;
        
        count+=1;
        if ((newP-g).norm() < (p-g).norm()) {
            decreased = true;
        }else{
            unmove(dR);
            dR = dR/2.0;
        }
    }




    if (count==MAX && !decreased) {
//        moveby(dR);
        return true;
//        moveby(dR);
    }


//    cout << "curr pos: "<<p(0)<<","<<p(1)<<","<<p(2)<<endl;
    

//    moveby(dR);
    cout << "Position: " << position() << endl;
    return (position()-g).norm() < tolerance ;
}
