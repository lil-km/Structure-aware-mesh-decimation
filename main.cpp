#include <igl/opengl/glfw/Viewer.h>
#include <igl/hausdorff.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "objParser.h"
#include "structurePreserve.h"

using std::cout; using std::cerr;
using std::endl; using std::string;
using std::ifstream; using std::ostringstream;

vector<Vertex *> vertexList;
vector<face *> faceList;
map<pair<int, int>, h_edge *> edgeList;
map<Vertex *, int> indexMap;

//proxy info
map<int, glm::mat4x4 *> proxyEq;
map<int, int> proxyCount;

//normal maps
std::map<face*, GLVector*> faceNormalMap;
std::map<Vertex*, GLVector*> vertexNormalMap;

Eigen::MatrixXd VA, VB;
Eigen::MatrixXi FA, FB;

static void show_usage(std::string name)
{
    std::cerr << "Usage: " << name << " "
              << "Arguments:\n"
              << "\tInput file path\n"
              << "\tK \tSpecify Number of multiple choice\n"
              << "\tITER \tSpecify Number of iteration"
              << std::endl;
}

int main(int argc, char* argv[])
{
    if (argc < 3) {
        show_usage(argv[0]);
        return 1;
    }

    string filename(argv[1]);
    loadMesh(filename);

    // multipleChoiceDecimation(argv[2]), atoi(argv[3]));
    shapePreserveDecimation(atoi(argv[2]), atoi(argv[3]));

    saveMesh();


    // Load a mesh in OBJ format
    igl::readOBJ(argv[1], VA, FA);
    igl::readOBJ("output.obj", VB, FB);

    double d;
    igl::hausdorff(VA,FA,VB,FB,d);

    cout << "Hausdorff distance: " << d << endl;

    // Plot the mesh
    igl::opengl::glfw::Viewer viewerA;
    viewerA.data().set_mesh(VA, FA);
    viewerA.data().set_face_based(true);
    viewerA.launch();

    igl::opengl::glfw::Viewer viewerB;
    viewerB.data().set_mesh(VB, FB);
    viewerB.data().set_face_based(true);
    viewerB.launch();

    exit(EXIT_SUCCESS);
}
