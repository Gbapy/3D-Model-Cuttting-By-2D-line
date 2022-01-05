#include "ImgCvt.h"

MATRIX s_Mat;
MATRIX t_Mat;
MATRIX rX_Mat;
MATRIX rY_Mat;
MATRIX rZ_Mat;
MATRIX TMat;

vector<SUBNODE>	model;
vector<SUBNODE>	tNodes;
vector<SUBNODE>	eNodes;

VERTEX	minBound;
VERTEX	maxBound;

double welding = 1e-1;
double maxDepth = 0;
double minDepth = 1e+10;
double x_Rotate = 0.52359877559829887307710723054658;
double y_Rotate = 0.52359877559829887307710723054658;
double z_Rotate = 0.52359877559829887307710723054658;
double normalFactor = 9.0f;

double		depth[RESIZED_WIDTH][RESIZED_HEIGHT];
int			indexMap1[RESIZED_WIDTH][RESIZED_HEIGHT];
int			indexMap2[RESIZED_WIDTH][RESIZED_HEIGHT];
VERTEX		vertMap[RESIZED_WIDTH][RESIZED_HEIGHT]; 
char		renderImg[RESIZED_HEIGHT][RESIZED_WIDTH * 4];
char		tmpImg[RESIZED_HEIGHT][RESIZED_WIDTH * 4];
double		roi_Depth = 1.0f;
int			gridInv = 0;

VECTOR3D dirV;
VECTOR3D tDirV;

void normalizeDepth() {
	maxDepth = -1;
	minDepth = 1e+10;
	for (int i = 0; i < RESIZED_HEIGHT; i++) {
		for (int j = 0; j < RESIZED_WIDTH; j++) {
			if (depth[i][j] > 0) {
				if (depth[i][j] > maxDepth) maxDepth = depth[i][j];
				if (depth[i][j] < minDepth) minDepth = depth[i][j];
			}
		}
	}

	for (int i = 0; i < RESIZED_HEIGHT; i++) {
		for (int j = 0; j < RESIZED_WIDTH; j++) {
			if (depth[i][j] > 0) {
				depth[i][j] = 255 - (depth[i][j] - minDepth) * 255.0f / (maxDepth - minDepth);
			}
		}
	}
}

void GetBound(VERTEX *mx, VERTEX *mn, vector<SUBNODE> *input) {
	for (uint32_t n = 0; n < input->size(); n++) {
		for (uint32_t i = 0; i < input->at(n).vertex.size(); i++) {
			VERTEX v = input->at(n).vertex[i];
			if (n == 0 && i == 0)
			{
				*mn = v;
				*mx = v;
			}
			else{
				if (mn->val[0] > v.val[0]) mn->val[0] = v.val[0];
				if (mn->val[1] > v.val[1]) mn->val[1] = v.val[1];
				if (mn->val[2] > v.val[2]) mn->val[2] = v.val[2];
				if (mx->val[0] < v.val[0]) mx->val[0] = v.val[0];
				if (mx->val[1] < v.val[1]) mx->val[1] = v.val[1];
				if (mx->val[2] < v.val[2]) mx->val[2] = v.val[2];
			}
		}
	}
}

void init3DOpt() {
	gridInv = 3;
	roi_Depth = 2.0f;
	x_Rotate = 0.0f;// 0.52359877559829887307710723054658;
	y_Rotate = 0.0f;// 0.52359877559829887307710723054658;
	z_Rotate = 0.0f;// 0.52359877559829887307710723054658;
	for (size_t i = 0; i < tNodes.size(); i++) {
		tNodes[i].index.clear();
		tNodes[i].vertex.clear();
	}
	tNodes.clear();
	for (size_t i = 0; i < eNodes.size(); i++) {
		eNodes[i].index.clear();
		eNodes[i].vertex.clear();
	}
	eNodes.clear();
	for (size_t i = 0; i < model.size(); i++) {
		model[i].index.clear();
		model[i].vertex.clear();
	}
	model.clear();
}

void init_Render() {
	minDepth = 1e+10;
	maxDepth = 0;

	for (size_t i = 0; i < tNodes.size(); i++) {
		tNodes[i].index.clear();
		tNodes[i].vertex.clear();
	}
	tNodes.clear();
	for (size_t n = 0; n < model.size(); n++) {
		SUBNODE sb;
		for (size_t m = 0; m < model[n].vertex.size(); m++) {
			VERTEX v = model[n].vertex[m];
			sb.vertex.push_back(v);
		}
		tNodes.push_back(sb);
	}

	for (int i = 0; i < RESIZED_HEIGHT; i++) {
		for (int j = 0; j < RESIZED_WIDTH; j++) {
			indexMap1[i][j] = -1;
			indexMap2[i][j] = -1;
		}
	}
}

VERTEX doAverage(VERTEX a, VERTEX b) {
	VERTEX r;

	r.val[0] = (a.val[0] + b.val[0]) * 0.5f;
	r.val[1] = (a.val[1] + b.val[1]) * 0.5f;
	r.val[2] = (a.val[2] + b.val[2]) * 0.5f;

	return r;
}

VERTEX doMinus(VERTEX a, VERTEX b) {
	VERTEX r = VERTEX(a.val[0] - b.val[0], a.val[1] - b.val[1], a.val[2] - b.val[2]);

	return r;
}

VECTOR3D doMinus(VECTOR3D a, VECTOR3D b) {
	VECTOR3D r = VECTOR3D(a.val[0] - b.val[0], a.val[1] - b.val[1], a.val[2] - b.val[2]);

	return r;
}

VERTEX doCross(VERTEX a, VERTEX b) {
	VERTEX r = VERTEX(a.val[1] * b.val[2] - a.val[2] * b.val[1], 
		a.val[2] * b.val[0] - a.val[0] * b.val[2],
		a.val[0] * b.val[1] - a.val[1] * b.val[0]);

	return r;
}

VECTOR3D doCross(VECTOR3D a, VECTOR3D b) {
	VECTOR3D r = VECTOR3D(a.val[1] * b.val[2] - a.val[2] * b.val[1],
		a.val[2] * b.val[0] - a.val[0] * b.val[2],
		a.val[0] * b.val[1] - a.val[1] * b.val[0]);

	return r;
}

VECTOR3D doCross(VECTOR3D *a, VERTEX *b) {
	VECTOR3D r = VECTOR3D(a->val[1] * b->val[2] - a->val[2] * b->val[1],
		a->val[2] * b->val[0] - a->val[0] * b->val[2],
		a->val[0] * b->val[1] - a->val[1] * b->val[0]);

	return r;
}

void xRotateMatrix(double alpha) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == j) rX_Mat.val[i][j] = 1; else rX_Mat.val[i][j] = 0;
		}
	}
	rX_Mat.val[1][1] = cos(alpha); rX_Mat.val[1][2] = sin(alpha);
	rX_Mat.val[2][1] = -sin(alpha); rX_Mat.val[2][2] = cos(alpha);
}

void yRotateMatrix(double alpha) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == j) rY_Mat.val[i][j] = 1; else rY_Mat.val[i][j] = 0;
		}
	}
	rY_Mat.val[0][0] = cos(alpha); rY_Mat.val[0][2] = sin(alpha);
	rY_Mat.val[2][0] = -sin(alpha); rY_Mat.val[2][2] = cos(alpha);
}

void zRotateMatrix(double alpha) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == j) rZ_Mat.val[i][j] = 1; else rZ_Mat.val[i][j] = 0;
		}
	}
	rZ_Mat.val[0][0] = cos(alpha); rZ_Mat.val[0][1] = sin(alpha);
	rZ_Mat.val[1][0] = -sin(alpha); rZ_Mat.val[1][1] = cos(alpha);
}

void tMatrix(VECTOR3D t) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == j) t_Mat.val[i][j] = 1; else t_Mat.val[i][j] = 0;
		}
	}
	t_Mat.val[3][0] = t.val[0]; t_Mat.val[3][1] = t.val[1]; t_Mat.val[3][2] = t.val[2];
}

void sMatrix(VECTOR3D s) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == j) s_Mat.val[i][j] = 1; else s_Mat.val[i][j] = 0;
		}
	}
	s_Mat.val[0][0] = s.val[0]; s_Mat.val[1][1] = s.val[1]; s_Mat.val[2][2] = s.val[2];
}

MATRIX makeConcatination(MATRIX *a, MATRIX *b) {
	MATRIX ret;

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			ret.val[i][j] = 0;
			for (int k = 0; k < 4; k++) {
				ret.val[i][j] += a->val[i][k] * b->val[k][j];
			}
		}
	}
	return ret;
}

void makeTransformMatrix(double rx, double ry, double rz) {
	xRotateMatrix(rx); yRotateMatrix(ry); zRotateMatrix(rz);
	TMat = rX_Mat;
	TMat = makeConcatination(&TMat, &rY_Mat);
	TMat = makeConcatination(&TMat, &rZ_Mat);
}

void doSetVertexNormal() {
	for (uint32_t n = 0; n < tNodes.size(); n++) {
		for (uint32_t m = 0; m < tNodes[n].vertex.size(); m++) {
			tNodes[n].vertex[m].nor = VECTOR3D(0, 0, 0);
			tNodes[n].vertex[m].ntriCnt = 0;
		}
	}
	for (uint32_t n = 0; n < model.size(); n++) {
		for (uint32_t m = 0; m < model[n].index.size(); m++) {
			VERTEX p1 = tNodes[n].vertex[model[n].index[m].idx[0]];
			VERTEX p2 = tNodes[n].vertex[model[n].index[m].idx[1]];
			VERTEX p3 = tNodes[n].vertex[model[n].index[m].idx[2]];
			VERTEX v1 = doMinus(p2, p1);
			VERTEX v2 = doMinus(p3, p1);
			VERTEX e = doCross(v1, v2);
			e.doNormalize();
			for (int p = 0; p < 3; p++) {
				tNodes[n].vertex[model[n].index[m].idx[p]].nor.val[0] += e.val[0];
				tNodes[n].vertex[model[n].index[m].idx[p]].nor.val[1] += e.val[1];
				tNodes[n].vertex[model[n].index[m].idx[p]].nor.val[2] += e.val[2];
				tNodes[n].vertex[model[n].index[m].idx[p]].ntriCnt++;
			}
		}
	}
	for (uint32_t n = 0; n < tNodes.size(); n++) {
		for (uint32_t m = 0; m < tNodes[n].vertex.size(); m++) {
			tNodes[n].vertex[m].nor.val[0] /= (float)(tNodes[n].vertex[m].ntriCnt);
			tNodes[n].vertex[m].nor.val[1] /= (float)(tNodes[n].vertex[m].ntriCnt);
			tNodes[n].vertex[m].nor.val[2] /= (float)(tNodes[n].vertex[m].ntriCnt);
			tNodes[n].vertex[m].nor.doNormalize();
		}
	}
}

VERTEX doTransform(VERTEX v, MATRIX mat) {
	VERTEX r;
	r.val[0] = 0; r.val[1] = 0; r.val[2] = 0; r.val[3] = 0;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			r.val[i] += v.val[j] * mat.val[j][i];
		}
	}
	return r;
}

void doSubnodeTransform(PSUBNODE node, MATRIX mat) {
	for (uint32_t m = 0; m < node->vertex.size(); m++) {
		VERTEX v = node->vertex[m];

		node->vertex[m] = doTransform(v, mat);
	}
}

void doTransform(vector<SUBNODE> *pNodes, MATRIX mat) {
	for (uint32_t n = 0; n < pNodes->size(); n++) {
		doSubnodeTransform(&(pNodes->at(n)), mat);
	}
}

void doWorldTransform() {

}

void doSubodeTransform(SUBNODE *node, VECTOR3D tr, VECTOR3D rt, VECTOR3D sc) {
	tMatrix(tr);
	sMatrix(sc);
	xRotateMatrix(rt.val[0]); yRotateMatrix(rt.val[1]); zRotateMatrix(rt.val[2]);
	TMat = rX_Mat;
	TMat = makeConcatination(&TMat, &rY_Mat);
	TMat = makeConcatination(&TMat, &rZ_Mat);
	TMat = makeConcatination(&TMat, &s_Mat);
	TMat = makeConcatination(&TMat, &t_Mat);
	doSubnodeTransform(node, TMat);
}

bool isValid(VERTEX *e, VERTEX *p) {
	double d = e->val[0] * p->val[0] + e->val[1] * p->val[1] + e->val[2] * p->val[2] + e->val[3];
	if (d == 0)
		return true;
	else
		return false;
}

void doNormalize(VERTEX *v) {
	double m = sqrt(v->val[0] * v->val[0] + v->val[1] * v->val[1] + v->val[2] * v->val[2]);
	v->val[0] /= m;
	v->val[1] /= m;
	v->val[2] /= m;
}

void doNormalize(VECTOR3D *v) {
	double m = sqrt(v->val[0] * v->val[0] + v->val[1] * v->val[1] + v->val[2] * v->val[2]);
	v->val[0] /= m;
	v->val[1] /= m;
	v->val[2] /= m;
}

void doFindEuqation() {
	for (int n = 0; n < model.size(); n++) {
		for (int m = 0; m < model[n].vertex.size(); m++) {
			tNodes[n].vertex[m].nor = VECTOR3D(0, 0, 0);
			tNodes[n].vertex[m].ntriCnt = 0;
		}
	}
	for (int i = 0; i < eNodes.size(); i++) {
		eNodes[i].index.clear();
		eNodes[i].vertex.clear();
	}
	eNodes.clear();
	for (int n = 0; n < model.size(); n++) {
		SUBNODE sb;
		for (int m = 0; m < model[n].index.size(); m++) {
			VERTEX p1 = tNodes[n].vertex[model[n].index[m].idx[0]];
			VERTEX p2 = tNodes[n].vertex[model[n].index[m].idx[1]];
			VERTEX p3 = tNodes[n].vertex[model[n].index[m].idx[2]];
			VERTEX v1 = doMinus(p2, p1);
			VERTEX v2 = doMinus(p3, p1);
			VERTEX e = doCross(v1, v2);
			doNormalize(&e);
			e.val[3] = -(e.val[0] * p1.val[0] + e.val[1] * p1.val[1] + e.val[2] * p1.val[2]);
			TRIANGLE tr = TRIANGLE(0, 0, 0);

			tr.param[0] = e.val[0]; tr.param[1] = e.val[1]; tr.param[2] = e.val[2]; tr.param[3] = e.val[3];
			for (int p = 0; p < 3; p++) {
				tNodes[n].vertex[model[n].index[m].idx[p]].nor.val[0] += e.val[0];
				tNodes[n].vertex[model[n].index[m].idx[p]].nor.val[1] += e.val[1];
				tNodes[n].vertex[model[n].index[m].idx[p]].nor.val[2] += e.val[2];
				tNodes[n].vertex[model[n].index[m].idx[p]].ntriCnt++;
			}
			sb.index.push_back(tr);
		}
		eNodes.push_back(sb);
	}
	for (int n = 0; n < model.size(); n++) {
		for (int m = 0; m < model[n].vertex.size(); m++) {
			tNodes[n].vertex[m].nor.val[0] /= (float)(tNodes[n].vertex[m].ntriCnt);
			tNodes[n].vertex[m].nor.val[1] /= (float)(tNodes[n].vertex[m].ntriCnt);
			tNodes[n].vertex[m].nor.val[2] /= (float)(tNodes[n].vertex[m].ntriCnt);
		}
	}
}

void doFindRealPoint(VERTEX *a, double c) {
	double wid = (double)RESIZED_WIDTH * 0.5f;
	double hig = (double)RESIZED_HEIGHT * 0.5f;

	a->val[0] = a->val[0] * a->val[2] / (c * wid);
	a->val[1] = a->val[1] * a->val[2] / (c * hig);
}

void doProjection(VERTEX *a, double c) {
	a->val[0] = (RESIZED_WIDTH / 2) + (a->val[0] * c / a->val[2]) * (RESIZED_WIDTH / 2);
	a->val[1] = (RESIZED_HEIGHT / 2) + (a->val[1] * c / a->val[2]) * (RESIZED_HEIGHT / 2);
	a->val[2] = 1;
}

void doProjection(VECTOR3D *a, double c) {
	a->val[0] = (RESIZED_WIDTH / 2) + (a->val[0] * c / a->val[2]) * (RESIZED_WIDTH / 2);
	a->val[1] = (RESIZED_HEIGHT / 2) + (a->val[1] * c / a->val[2]) * (RESIZED_HEIGHT / 2);
}

double correctCenter(vector<SUBNODE> *pNodes) {
	VERTEX mn, mx;
	VERTEX center;
	double c = sqrt(3);
	double ret = 0;

	center.val[0] = 0; center.val[1] = 0; center.val[2] = 0; center.val[3] = 1;
	GetBound(&mx, &mn, pNodes);
	

	center.val[0] = (mx.val[0] + mn.val[0]) / 2;
	center.val[1] = (mx.val[1] + mn.val[1]) / 2;
	center.val[2] = (mx.val[2] + mn.val[2]) / 2;

	double wx = mx.val[0] - mn.val[0];
	double wy = mx.val[1] - mn.val[1];
	double wz = mx.val[2] - mn.val[2];

	if (wx >= wy && wx >= wz) {
		dirV = VECTOR3D(1, 0, 0);
	}
	else if (wy >= wx && wy >= wz) {
		dirV = VECTOR3D(0, 1, 0);
	}
	else if (wz >= wx && wz >= wy) {
		dirV = VECTOR3D(0, 0, 1);
	}

	for (uint32_t n = 0; n < pNodes->size(); n++) {
		for (uint32_t m = 0; m < pNodes->at(n).vertex.size(); m++) {
			VERTEX v = pNodes->at(n).vertex[m];
			v.val[0] -= center.val[0]; v.val[1] -= center.val[1]; v.val[2] -= center.val[2];
			pNodes->at(n).vertex[m] = v;
		}
	}

	minBound = mn; maxBound = mx;

	minBound = doMinus(minBound, center);
	maxBound = doMinus(maxBound, center);

	double mxX = mx.val[0] - center.val[0];
	double mxY = mx.val[1] - center.val[1];
	double mxZ = mx.val[2] - center.val[2];

	ret = std::fmax(mxX, fmax(mxY, mxZ));
	return ret;
}

bool isInside(VERTEX *p, VERTEX a) {
	VERTEX pp[3];
	VERTEX v1 = doMinus(p[1], p[0]);
	VERTEX v2 = doMinus(a, p[0]);
	pp[0] = doCross(v1, v2);
	v1 = doMinus(p[2], p[1]);
	v2 = doMinus(a, p[1]);
	pp[1] = doCross(v1, v2);
	v1 = doMinus(p[0], p[2]);
	v2 = doMinus(a, p[2]);
	pp[2] = doCross(v1, v2);
	if ((pp[0].val[2] > 0 && pp[1].val[2] > 0 && pp[2].val[2] > 0) ||
		(pp[0].val[2] < 0 && pp[1].val[2] < 0 && pp[2].val[2] < 0)) 
		return true;
	return false; 
}

void PushVertex(int i, int j, int n, int m, double x, double y, double z) {
	depth[j][i] = z;
	indexMap1[j][i] = n;
	indexMap2[j][i] = m;
	vertMap[j][i] = VERTEX(x, y, z);
	VERTEX p1 = tNodes[n].vertex[model[n].index[m].idx[0]];
	VERTEX p2 = tNodes[n].vertex[model[n].index[m].idx[1]];
	VERTEX p3 = tNodes[n].vertex[model[n].index[m].idx[2]];
	VERTEX v1 = VERTEX(x - p1.val[0], y - p1.val[1], z - p1.val[2]);
	VERTEX v2 = VERTEX(x - p2.val[0], y - p2.val[1], z - p2.val[2]);
	VERTEX v3 = VERTEX(x - p3.val[0], y - p3.val[1], z - p3.val[2]);
	VERTEX l1 = VERTEX(p2.val[0] - p1.val[0], p2.val[1] - p1.val[1], p2.val[2] - p1.val[2]);
	VERTEX l2 = VERTEX(p3.val[0] - p2.val[0], p3.val[1] - p2.val[1], p3.val[2] - p2.val[2]);
	VERTEX l3 = VERTEX(p1.val[0] - p3.val[0], p1.val[1] - p3.val[1], p1.val[2] - p3.val[2]);
	
	double m1 = v1.getMagnitude();
	double m2 = v2.getMagnitude();
	double m3 = v3.getMagnitude();
	double ml1 = l1.getMagnitude();
	double ml2 = l2.getMagnitude();
	double ml3 = l3.getMagnitude();
	
	VERTEX c1 = doCross(v1, l1);
	VERTEX c2 = doCross(v2, l2);
	VERTEX c3 = doCross(v3, l3);

	double aa = c1.getMagnitude() / ml1;
	double bb = c2.getMagnitude() / ml2;
	double cc = c3.getMagnitude() / ml3;
	double s = doCross(l1, l2).getMagnitude();
	double a = s / ml1;
	double b = s / ml2;
	double c = s / ml3;

	aa /= a; bb /= b; cc /= c;
	/*
	aa *= (PI / 2.0f);
	bb *= (PI / 2.0f);
	bb *= (PI / 2.0f);
	aa = sin(aa); bb = sin(bb); cc = sin(cc);
	*/
	VECTOR3D nor1;
	VECTOR3D nor2;
	VECTOR3D nor3;

	nor1.val[0] = p1.nor.val[0] * bb; nor1.val[1] = p1.nor.val[1] * bb; nor1.val[2] = p1.nor.val[2] * bb;
	nor2.val[0] = p2.nor.val[0] * cc; nor2.val[1] = p2.nor.val[1] * cc; nor2.val[2] = p2.nor.val[2] * cc;
	nor3.val[0] = p3.nor.val[0] * aa; nor3.val[1] = p3.nor.val[1] * aa; nor3.val[2] = p3.nor.val[2] * aa;
	VECTOR3D nor;
	nor.val[0] = nor1.val[0] + nor2.val[0] + nor3.val[0];
	nor.val[1] = nor1.val[1] + nor2.val[1] + nor3.val[1];
	nor.val[2] = nor1.val[2] + nor2.val[2] + nor3.val[2];
	m1 = nor.getMagnitude();
	vertMap[j][i].nor = VECTOR3D(nor.val[0] / m1, nor.val[1] / m1, nor.val[2] / m1);
}

void doRender(double zoomfactor, bool transform) {
	double c1 = sqrt(3);
	double c = 1 / c1;
	memset(depth, 0, RESIZED_HEIGHT * RESIZED_WIDTH);
	memset(indexMap1, 0, sizeof(int) * RESIZED_HEIGHT * RESIZED_WIDTH);
	memset(indexMap2, 0, sizeof(int) * RESIZED_HEIGHT * RESIZED_WIDTH);
	memset(vertMap, 0, sizeof(VERTEX) * RESIZED_HEIGHT * RESIZED_WIDTH);
	memset(renderImg, 0, RESIZED_HEIGHT * RESIZED_WIDTH * 4);

	if (transform) {
		makeTransformMatrix(x_Rotate, y_Rotate, z_Rotate);
		doTransform(&tNodes, TMat);
	}

	for (uint32_t n = 0; n < tNodes.size(); n++) {
		for (uint32_t m = 0; m < tNodes[n].vertex.size(); m++) {
			tNodes[n].vertex[m].val[2] += zoomfactor;
		}
	}

	doFindEuqation();

	for (int i = 0; i < RESIZED_WIDTH; i++) {
		for (int j = 0; j < RESIZED_HEIGHT; j++) {
			depth[i][j] = 0;
		}
	}

	for (uint32_t n = 0; n < model.size(); n++) {
		for (uint32_t m = 0; m < model[n].index.size(); m++)
		{
			if (model[n].index[m].isValid == false) continue;
			VERTEX p[3];

			for (int i = 0; i < 3; i++) {
				p[i] = tNodes[n].vertex[model[n].index[m].idx[i]];
			}

			double mxX, mnX, mxY, mnY;
			for (int i = 0; i < 3; i++) {
				doProjection(&p[i], c1);

				if (i == 0) {
					mxX = p[i].val[0]; mnX = p[i].val[0];
					mxY = p[i].val[1]; mnY = p[i].val[1];
				}
				else{
					if (p[i].val[0] > mxX) mxX = p[i].val[0];
					if (p[i].val[0] < mnX) mnX = p[i].val[0];
					if (p[i].val[1] > mxY) mxY = p[i].val[1];
					if (p[i].val[1] < mnY) mnY = p[i].val[1];
				}
			}
			VERTEX v1 = doMinus(p[2], p[0]);
			VERTEX v2 = doMinus(p[1], p[0]);
			v1 = doCross(v1, v2);
			//if (v1.val[2] < 0) continue;
			int imxX = (int)mxX;
			int imnX = (int)mnX;
			int imxY = (int)mxY;
			int imnY = (int)mnY;
			VERTEX pp[3];
			pp[0] = p[0]; pp[0].val[2] = 0;
			pp[1] = p[1]; pp[1].val[2] = 0;
			pp[2] = p[2]; pp[2].val[2] = 0;
			for (int i = imnX - 1; i <= imxX + 1; i++) {
				if (i < 0 || i >= RESIZED_WIDTH) continue;
				for (int j = imnY - 1; j < imxY + 1; j++) {
					if (j < 0 || j >= RESIZED_HEIGHT) continue;
					VERTEX a;
					a.val[0] = i + 0.5; a.val[1] = j + 0.5; a.val[2] = 0;
					
					if (isInside(pp, a)) 
					{
						double x = i - RESIZED_WIDTH / 2;
						double y = j - RESIZED_HEIGHT / 2;
						x = x / (double)(RESIZED_WIDTH / 2) * c;
						y = y / (double)(RESIZED_HEIGHT / 2) * c;
						double z = eNodes[n].index[m].param[0] * x + eNodes[n].index[m].param[1] * y + eNodes[n].index[m].param[2];
						if (z != 0) {
							z = -eNodes[n].index[m].param[3] / z;
							x = x * z;
							y = y * z;
							if (z > 0) {
								if (z > maxDepth) maxDepth = z;
								if (z < minDepth) minDepth = z;
								if (depth[j][i] == 0) {
									PushVertex(i, j, n, m, x, y, z);
								}
								else if (z < depth[j][i]) {
									PushVertex(i, j, n, m, x, y, z);
								}
							}
						}
					}
				}
			}
		}
	}

	for (uint32_t n = 0; n < tNodes.size(); n++) {
		for (uint32_t m = 0; m < tNodes[n].vertex.size(); m++) {
			tNodes[n].vertex[m].val[2] -= zoomfactor;
		}
	}
}

MATRIX getUVEMatrix(VERTEX *axies, VERTEX eyePos) {
	MATRIX m;

	axies[0].doNormalize();
	axies[1].doNormalize();
	axies[2].doNormalize();
	m.val[0][0] = axies[1].val[0]; m.val[0][1] = axies[1].val[1]; m.val[0][2] = axies[1].val[2];
	m.val[0][3] = -(axies[1].val[0] * eyePos.val[0] + axies[1].val[1] * eyePos.val[1] + axies[1].val[2] * eyePos.val[2]);
	m.val[1][0] = axies[0].val[0]; m.val[1][1] = axies[0].val[1]; m.val[1][2] = axies[0].val[2];
	m.val[1][3] = -(axies[0].val[0] * eyePos.val[0] + axies[0].val[1] * eyePos.val[1] + axies[0].val[2] * eyePos.val[2]);
	m.val[2][0] = axies[2].val[0]; m.val[2][1] = axies[2].val[1]; m.val[2][2] = axies[2].val[2];
	m.val[2][3] = -(axies[2].val[0] * eyePos.val[0] + axies[2].val[1] * eyePos.val[1] + axies[2].val[2] * eyePos.val[2]);
	m.val[3][0] = 0; m.val[3][1] = 0; m.val[3][2] = 0; m.val[3][3] = 1;
	return m;
}

