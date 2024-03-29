#ifndef preHeader_h
#define preHeader_h

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <vector>
#include <map>
#include <algorithm>
#include "float.h"
#include <dirent.h>

#ifdef __APPLE__
//#include <OpenGL/gl.h> (Included by glew.h)
#include <GL/glew.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h> //(Outdated)
#include <GLFW/glfw3.h>
#include <thread>

#else

#include <GL/gl.h>
#include <GL/glu.h>
//#include <GL/glut.h>
#include <GL/glew.h>
#endif

#include <Eigen> //Eigen needs to go before CImg because X11 re-defines "Success" macro


using namespace std;
using namespace Eigen;

#ifdef DEBUG
#   define ASSERT(condition, message) \
do { \
if (! (condition)) { \
cerr << "Assertion `" #condition "` failed in " << __FILE__ \
<< " line " << __LINE__ << ": " << message << std::endl; \
exit(EXIT_FAILURE); \
} \
} while (false)
#else
#   define ASSERT(condition, message) do { } while (false)
#endif

#define nl << endl

#pragma mark - typedefs and enums

typedef Array3f Color;
typedef Array3f Point;
typedef Vector3f Vector;
typedef Array2f Sample;
typedef Translation<float, 3> Translation3f;
typedef UniformScaling<float> Scalingf;
typedef DiagonalMatrix<float, 3> Scaling3f;
#define Scaling3f(X, Y, Z) Scaling((float)X, (float)Y, (float)Z)
typedef Transform<float, 3, Affine> Transform3f;

typedef enum {
    LightSourceDirectional,
    LightSourcePoint
} LightSourceType;

struct CmdLineOptResult {
    string optName;
    int numOfArgs;
    vector<string> *args;
};
struct BoundingBox{
    Point min;
    Point max;
};

#pragma mark - Math

float sqr(float x);
Vector makeVec(Point start, Point end);
bool isInteger(const string & s);
float minf(float num1, float num2);
float maxf(float num1, float num2);

#pragma mark - Utilities

string fileNameWithoutExt(const string& fileName);
void println(string str);
bool isNewLine(const char* str);
string stripNewLine(string dirty);


/**
 *  Parses command line options specified in "options"
 *
 *  @param argc    argc from main
 *  @param argv    argc from main
 *  @param options a specially formatted string specifying what you want to parse
 *  @param results a pointer to a pointer to the vector of parsed results
 *  @discussion
 *      The format of options: "argX(n)argY(m)", where argX argY are the name of the options,
 *      n and m are the number of arguments for each option
 *      for example, if my program takes "./program -file one two --copy bunny -k rgb"
 *      then my options string would be "-file(2)--copy(1)-k(1)"
 *
 *      The results pointer will be set to a vector, containing CmdLineOptResult's, which is typedef'd above.
 *      To-Do: catch incorrectly formatted arguments and inconsistent number of arguments error instead of crashing
 *  @return none
 *  @example
 *      vector<CmdLineOptResults> *results;
 *      getCmdLineOptions(argc, argv, "-ka(2)-kz(1)", &results);
 *      for (auto & result : results) {
 *          cout << result.optName << "\n";
 *          cout << result.args->at(0) <<" "<< result.args-at(1);
 *      }
 */
void getCmdLineOptions(int argc, char *argv[], string options, vector<CmdLineOptResult> **results);

#pragma mark - Conversion
float floatFromString(string str);
int intFromString(string str);

#endif

