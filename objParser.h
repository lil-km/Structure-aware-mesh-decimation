#ifndef OBJPARSER_H
#define OBJPARSER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <cstdio>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <math.h>
#include <cfloat>
#include "halfEdge.h"

using namespace std;

extern map<Vertex *, int> indexMap;
extern vector<Vertex *> vertexList;    
extern vector<face *> faceList;       
extern map<pair<int, int>, h_edge *> edgeList;
extern int meshImportFlag;
extern map<int, int> proxyCount;
extern std::map<face *, GLVector*> faceNormalMap;
extern std::map<Vertex *, GLVector*> vertexNormalMap;
extern void calculateVertexNormal();
extern void calculateFaceNormal();

GLVector* findVertexNormal(Vertex* );
/**
 *  for reading .obj file.
 *
 *  @param name of file
 *  @return void
 */
void loadMesh(string);

/**
 * For saving .obj file.
 *
 * @param filename
 * @return void
 */
void saveMesh();

#endif
