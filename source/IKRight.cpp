//
//  IKRight.cpp
//  IKRight
//  Copyright (c) 2014 Leo C & Donny R. All rights reserved.


#include "IKRight.h"

// Initial size of graphics window.
const int WIDTH  = 800;
const int HEIGHT = 600;
// Current size of window.
int width  = WIDTH;
int height = HEIGHT;
// Mouse positions, normalized to [0,1].
double xMouse = 0.5;
double yMouse = 0.5;
double zMouse = 0.0;
// Bounds of viewing frustum.
float nearPlane =  1.0;
float farPlane  = 1000.0;
// Viewing angle.
double fovy = 40.0;
// Variables.
//double alpha = 0;                                  // Set by mouse X
//double beta = 0;                                   // Set by mouse Y

vector<LinkInfo> links;
Vector3f root(0,0,0);
LinkInfo link1 = {30, Vector3f(-1,1,0), M_PI/2};
LinkInfo link2 = {20, Vector3f(-1,0,0), M_PI/2};
//LinkInfo link3 = {20, 3*M_PI/4, M_PI/2};
Arm* arm;
Vector3f goal(-10, 10, 10);
bool resolved = false;
Vector3f targetPoint = Vector3f::Zero();

// This function is called to display the scene.
void display () {
    glEnable(GL_LIGHTING);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // Translate using Y mouse.
    glTranslatef(0, -10, -100);
    
    // Rotation from idle function.
//    glRotatef(90, 0, 0, 1);
    
    // Rotation using X mouse.
//    alpha = 180.0 * xMouse;
//    beta = 180.0 * yMouse;
//    glRotatef(alpha, 0, 1, 0);
//    glRotatef(beta, 1, 0, 0);
    
    if (!resolved) {
        arm->update(goal);
    }
    
    glPushMatrix();
    glColor3f(0.0f, 0.0f, 1.0f);
//    glTranslatef(0, -targetPoint(1), targetPoint(0));
//    glTranslatef(xMouse, yMouse, 0);
//    glutSolidSphere(3.0f, 8, 8); //selector sphere
//    glutSolidTeapot(8);
    glPopMatrix();
    
    glFlush();
    glutSwapBuffers();
}

// This function is called when there is nothing else to do.
void idle () {
    glutPostRedisplay();
}

// This function gets called every 10ms
void timer(int i) {
//    l.update();
//    if(l.isTargetResolved()) {targetPoint = l.getPointWithinRange(); l.moveToPoint(targetPoint); }
    glutTimerFunc(10, timer, i);
    glutPostRedisplay();
}

void mouseMovement (int mx, int my) {
    // Normalize mouse coordinates.
//    xMouse = double(mx) / double(width);
//    yMouse = 1 - double(my) / double(height);
    xMouse = mx - width/2;
    yMouse = my;
    // Redisplay image.
    glutPostRedisplay();
}

// Respond to window resizing, preserving proportions.
// Parameters give new window size in pixels.
void reshapeMainWindow (int newWidth, int newHeight) {
    width = newWidth;
    height = newHeight;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fovy, GLfloat(width) / GLfloat(height), nearPlane, farPlane);
}

// Respond to graphic character keys.
// Parameters give key code and mouse coordinates.
void graphicKeys (unsigned char key, int x, int y) {
    switch (key) {
        case 27:
            exit(0);
    }
}

/*
 +––––––––––––––––––––––––––––––––––––––+
 |Main Functions                        |
 +––––––––––––––––––––––––––––––––––––––+
 */
/**
 *  IKRight Main Function
 *
 *  @param argc default argc
 *  @param argv default argv
 *
 *  @return status code
 */
int main(int argc, char *argv[]) {
    vector<CmdLineOptResult> *results;
    string basePath;
    string options = "--testArm(0)"; //test Arm functionality

    getCmdLineOptions(argc, argv, options, &results);

    string directoryName;
    string fileName;
    
    for (auto& result : *results) {
        if (result.optName.compare("--testArm")==0) {
            testArm();
        }
    }
    
    links.push_back(link1);
    links.push_back(link2);
//    links.push_back(link3);
    arm = new Arm(links, root);
    
    // GLUT initialization.
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(width, height);
    glutCreateWindow("IKRight! - Leo Colobong & Donny Reynolds");
    
    // Register call backs.
    glutDisplayFunc(display);
    glutReshapeFunc(reshapeMainWindow);
    glutKeyboardFunc(graphicKeys);
    glutMotionFunc(mouseMovement);
    glutIdleFunc(idle);
    glutTimerFunc(60, timer, 0);
    
    // OpenGL initialization
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    
    GLfloat global_ambient[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glLightModelfv(GL_AMBIENT_AND_DIFFUSE, global_ambient);
    
    // Enter GLUT loop.
    glutMainLoop();
    
    return 0;
}


