#pragma once

#include <vector>
#include <stdint.h>

#include "resource.h"

using namespace std;

#define RESIZED_WIDTH	600
#define RESIZED_HEIGHT	600
#define ROI				5
#define	PI				3.1415926535897932384626433832795

typedef struct _VECTOR3D_
{
	_VECTOR3D_() {
	
	}

	_VECTOR3D_(double x, double y, double z) {
		val[0] = x; val[1] = y; val[2] = z;
	}

	double getMagnitude() {
		return sqrt(val[0] * val[0] + val[1] * val[1] + val[2] * val[2]);
	}
	void doNormalize() {
		double m = this->getMagnitude();
		if (m == 0) {
			val[0] = 0;
			val[1] = 0;
			val[2] = 0;
		}
		else {
			val[0] /= m;
			val[1] /= m;
			val[2] /= m;
		}
	}
	double	val[3];
}VECTOR3D, *PVECTOR3D;

typedef struct _VERTEX_
{
	_VERTEX_() {
		val[0] = 0; val[1] = 0; val[2] = 0; val[3] = 1;
		ntriCnt = 0;
		optIndex = -1;
	}

	_VERTEX_(double x, double y, double z) {
		val[0] = x; val[1] = y; val[2] = z; val[3] = 1;
		ntriCnt = 0;
		optIndex = -1;
	}

	double getMagnitude() {
		return sqrt(val[0] * val[0] + val[1] * val[1] + val[2] * val[2]);
	}

	void doNormalize() {
		double m = this->getMagnitude();
		if (m == 0) {
			val[0] = 0;
			val[1] = 0;
			val[2] = 0;
		}
		else {
			val[0] /= m;
			val[1] /= m;
			val[2] /= m;
		}
	}
	double		val[4];
	VECTOR3D	nor;
	int			ntriCnt;
	int			optIndex;
}VERTEX, *PVERTEX;

typedef struct _TRIANGLE_
{
	_TRIANGLE_() {
		idx[0] = -1; idx[1] = -1; idx[2] = -1;
	}

	_TRIANGLE_(int idx1, int idx2, int idx3) {
		idx[0] = idx1; idx[1] = idx2; idx[2] = idx3;
	}
	int		idx[3];
	double	param[4];
	bool isValid = true;
}TRIANGLE, *PTRIANGLE;

typedef struct _SUBNODE_
{
	vector<VERTEX>		vertex;
	vector<TRIANGLE>	index;
}SUBNODE, *PSUBNODE;

typedef struct _MATRIX_ 
{
	double val[4][4];
}MATRIX, *PMATRIX;

enum FBXENUM
{
	SUCCESS_IMPORT,
	INVALID_FBX_FILE,
	UNSURPPORTED_FBX_VERSION,
	UNSURPPORTED_PROPERTY_TYPE,
	INVALID_VERTEX_PAIR,
	INVALID_POLYGON_PAIR,
	COMPRESSED_PROPERTY_FOUND
};


extern double	depth[RESIZED_WIDTH][RESIZED_HEIGHT];
extern int		indexMap1[RESIZED_WIDTH][RESIZED_HEIGHT];
extern int		indexMap2[RESIZED_WIDTH][RESIZED_HEIGHT];
extern VERTEX	vertMap[RESIZED_WIDTH][RESIZED_HEIGHT];
extern char		renderImg[RESIZED_HEIGHT][RESIZED_WIDTH * 4];
extern char		tmpImg[RESIZED_HEIGHT][RESIZED_WIDTH * 4];

extern double	roi_Depth;
extern int		gridInv;
extern double	maxDepth;
extern double	minDepth;
extern double	x_Rotate;
extern double	y_Rotate;
extern double	z_Rotate;
extern double	normalFactor;

extern vector<SUBNODE>	model;
extern vector<SUBNODE>	tNodes;
extern vector<SUBNODE>	eNodes;

extern VERTEX	minBound;
extern VERTEX	maxBound;

extern VECTOR3D tDirV;
extern MATRIX TMat;

bool isLittleEndian();
char readChar(FILE *pFile);
std::string readString(FILE *pFile, uint8_t length);
uint32_t readUint32(FILE *pFILE);
uint64_t readUint64(FILE *pFILE);
uint16_t readUint16(FILE *pFile);
float readFloat(FILE *pFile);
double readDouble(FILE *pFile);
bool FileExists(const char *filePathPtr);
std::string toString(float v);
void init3DOpt();

//3D Opt
double correctCenter(vector<SUBNODE> *pNode);
void doFindEuqation();
void doRender(double zoomfactor, bool transform);
void normalizeDepth();
void doSubodeTransform(SUBNODE *node, VECTOR3D tr, VECTOR3D rt, VECTOR3D sc);
void doProjection(VERTEX *a, double c);
void doProjection(VECTOR3D *a, double c);
void doFindRealPoint(VERTEX *a, double c);
VERTEX doCross(VERTEX a, VERTEX b);
VECTOR3D doCross(VECTOR3D a, VECTOR3D b);
VERTEX doMinus(VERTEX a, VERTEX b);
VECTOR3D doMinus(VECTOR3D a, VECTOR3D b);
VERTEX doAverage(VERTEX a, VERTEX b);

void init_Render();
MATRIX getUVEMatrix(VERTEX *axies, VERTEX eyePos);
void doTransform(vector<SUBNODE> *pNodes, MATRIX mat);
VERTEX doTransform(VERTEX v, MATRIX mat);
void doSetVertexNormal();
void makeTransformMatrix(double rx, double ry, double rz);