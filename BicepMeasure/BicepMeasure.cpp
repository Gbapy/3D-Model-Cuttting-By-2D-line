// BicepMeasure.cpp : Defines the entry point for the application.
//
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "framework.h"
#include "BicepMeasure.h"
#include "ImgCvt.h"

#include <commdlg.h>
#include <windowsx.h>

#define MAX_LOADSTRING 100

// Global Variables:
HINSTANCE hInst;                                // current instance
WCHAR szTitle[MAX_LOADSTRING];                  // The title bar text
WCHAR szWindowClass[MAX_LOADSTRING];            // the main window class name

// Forward declarations of functions included in this code module:
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);

char		szFile[MAX_PATH];
const		aiScene* scene = NULL;
double		zoomfactor = 0;
bool		isDrawing = false;
bool		isPlaneAdded = false;

void ShowResultImage(HWND hWnd, BYTE *frm) {
	HDC hdc = GetDC(hWnd);
	HDC hdcMem = CreateCompatibleDC(hdc);
	HGDIOBJ oldBitmap;
	HBITMAP hBmp = CreateBitmap(RESIZED_WIDTH, RESIZED_HEIGHT, 1, 32, frm);
	RECT rect;

	GetClientRect(hWnd, &rect);

	SetStretchBltMode(hdc, HALFTONE);

	oldBitmap = SelectObject(hdcMem, hBmp);

	StretchBlt(hdc, 0, 0, rect.right - rect.left, rect.bottom - rect.top, hdcMem, 0, 0, RESIZED_WIDTH, RESIZED_HEIGHT, SRCCOPY);

	SelectObject(hdcMem, oldBitmap);
	DeleteDC(hdcMem);
	ReleaseDC(hWnd, hdc);
	DeleteDC(hdc);
	DeleteObject(hBmp);
}

void PutColor(UCHAR *src, int width, int height, double x, double y, UCHAR r, UCHAR g, UCHAR b) {
	if (x < 0 || x >= width) return;
	if (y < 0 || y >= height) return;
	src[(int)y * width * 4 + (int)x * 4] = r;
	src[(int)y * width * 4 + (int)x * 4 + 1] = g;
	src[(int)y * width * 4 + (int)x * 4 + 2] = b;
}

void DrawLine(UCHAR *src, int width, int height, double x1, double y1, double x2, double y2, UCHAR r, UCHAR g, UCHAR b) {
	double m = sqrtf((float)((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)));
	double stpX = (x2 - x1) / (double)m;
	double stpY = (y2 - y1) / (double)m;
	double sX = x1;
	double sY = y1;
	for (int i = 0; i < m; i++) {
		sX += stpX;
		sY += stpY;
		PutColor(src, width, height, sX, sY, r, g, b);
	}
}

void drawMesh() {
	double c = sqrt(3.0f);

	memset(renderImg, 0, RESIZED_HEIGHT * RESIZED_WIDTH * 4);

	makeTransformMatrix(x_Rotate, y_Rotate, z_Rotate);
	doTransform(&tNodes, TMat);

	for (uint32_t n = 0; n < tNodes.size(); n++) {
		for (uint32_t m = 0; m < tNodes[n].vertex.size(); m++) {
			tNodes[n].vertex[m].val[2] += zoomfactor;
		}
	}

	for (UINT n = 0; n < model.size(); n++) {
		for (UINT m = 0; m < model[n].index.size(); m++) {
			if (model[n].index[m].isValid == false) continue;
			VERTEX v1 = tNodes[n].vertex[model[n].index[m].idx[0]];
			VERTEX v2 = tNodes[n].vertex[model[n].index[m].idx[1]];
			VERTEX v3 = tNodes[n].vertex[model[n].index[m].idx[2]];
			doProjection(&v1, c); doProjection(&v2, c); doProjection(&v3, c);
			DrawLine((UCHAR *)renderImg, RESIZED_WIDTH, RESIZED_HEIGHT, v1.val[0], v1.val[1], v2.val[0], v2.val[1], 255, 0, 0);
			DrawLine((UCHAR *)renderImg, RESIZED_WIDTH, RESIZED_HEIGHT, v2.val[0], v2.val[1], v3.val[0], v3.val[1], 255, 0, 0);
			DrawLine((UCHAR *)renderImg, RESIZED_WIDTH, RESIZED_HEIGHT, v3.val[0], v3.val[1], v1.val[0], v1.val[1], 255, 0, 0);
		}
	}

	for (uint32_t n = 0; n < tNodes.size(); n++) {
		for (uint32_t m = 0; m < tNodes[n].vertex.size(); m++) {
			tNodes[n].vertex[m].val[2] -= zoomfactor;
		}
	}
}

void coatMaterial() {
	for (int i = 0; i < RESIZED_HEIGHT; i++) {
		for (int j = 0; j < RESIZED_WIDTH; j++) {
			if (indexMap1[i][j] == -1 || indexMap2[i][j] == -1) continue;
			VERTEX v1 = VERTEX(1, 1, 1);
			VERTEX v2 = VERTEX(vertMap[i][j].nor.val[0], vertMap[i][j].nor.val[1], vertMap[i][j].nor.val[2]);
			double m = doCross(v1, v2).getMagnitude();
			m = m * m * 70.0f;
			if (m > 255) m = 255;
			renderImg[i][j * 4] = (BYTE)m;
			renderImg[i][j * 4 + 1] = (BYTE)m;
			renderImg[i][j * 4 + 2] = (BYTE)m;
			renderImg[i][j * 4 + 3] = 1;
		}
	}
}

void get_bounding_box_for_node(const struct aiNode* nd, aiMatrix4x4* trafo)
{
	aiMatrix4x4 prev;
	unsigned int n = 0, t;

	prev = *trafo;
	aiMultiplyMatrix4(trafo, &nd->mTransformation);

	for (; n < nd->mNumMeshes; ++n) {
		const struct aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
		SUBNODE subnode;
		for (t = 0; t < mesh->mNumVertices; ++t) {

			aiVector3D tmp = mesh->mVertices[t];
			aiTransformVecByMatrix4(&tmp, trafo);
			subnode.vertex.push_back(VERTEX(tmp.x, tmp.y, tmp.z));
		}
		for (UINT t = 0; t < mesh->mNumFaces; t++) {
			aiFace tmp = mesh->mFaces[t];
			if (tmp.mNumIndices < 3) continue;
			for (UINT i = 2; i < tmp.mNumIndices; i++) {
				subnode.index.push_back(TRIANGLE(tmp.mIndices[0], tmp.mIndices[i - 1], tmp.mIndices[i]));
			}
		}
		model.push_back(subnode);
	}

	for (n = 0; n < nd->mNumChildren; ++n) {
		get_bounding_box_for_node(nd->mChildren[n], trafo);
	}
	*trafo = prev;
}

int loadasset(const char* path)
{
	/* we are taking one of the postprocessing presets to avoid
	spelling out 20+ single postprocessing flags here. */

	scene = aiImportFile(path, aiProcessPreset_TargetRealtime_Fast);
	aiMatrix4x4 trafo;
	aiIdentityMatrix4(&trafo);

	if (scene) {
		if (scene->mRootNode != NULL)
		{
			get_bounding_box_for_node(scene->mRootNode, &trafo);
		}
		aiReleaseImport(scene);
		return 0;
	}
	return 1;
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
	hInst = hInstance;

	DialogBox(hInstance, MAKEINTRESOURCE(IDD_DLG_MAIN), NULL, (DLGPROC)WndProc);
	return 0;
}

int isConflict(VERTEX v1, VERTEX v2, VERTEX *ret) {
	double x0 = v2.val[1] - v1.val[1];
	double x1 = - v1.val[1];
	double x2 = - v2.val[1];

	if ((x1 < 0 && x2 > 0) || (x1 > 0 && x2 < 0)) {
		ret->val[0] = v1.val[0] + (v2.val[0] - v1.val[0]) * x1 / x0;
		ret->val[2] = v1.val[2] + (v2.val[2] - v1.val[2]) * x1 / x0;
		ret->val[1] = 0;
		return 1;
	}
	return 0;
}

void doSegment() {
	VERTEX axes[3];
	VERTEX eye;
	VERTEX origin[3];
	VERTEX orgEye = VERTEX(0, 0, 0);

	origin[0] = VERTEX(0, 1, 0);
	origin[1] = VERTEX(1, 0, 0);
	origin[2] = VERTEX(0, 0, 1);

	SUBNODE sb = tNodes[tNodes.size() - 1];
	axes[1] = doMinus(sb.vertex[3], sb.vertex[0]);
	axes[2] = doMinus(sb.vertex[1], sb.vertex[0]);
	axes[0] = doCross(axes[1], axes[2]);

	eye = doAverage(sb.vertex[0], sb.vertex[3]);

	tNodes.pop_back(); model.pop_back();

	MATRIX m = getUVEMatrix(axes, eye);

	doTransform(&tNodes, m);
	origin[0] = doTransform(origin[0], m);
	origin[1] = doTransform(origin[1], m);
	origin[2] = doTransform(origin[2], m);
	orgEye = doTransform(orgEye, m);

	for (UINT i = 0; i < tNodes.size(); i++) {
		int m = (int)(model[i].index.size());
		for (int j = 0; j < m; j++) {
			TRIANGLE tr = model[i].index[j];
			VERTEX v1 = tNodes[i].vertex[tr.idx[0]];
			VERTEX v2 = tNodes[i].vertex[tr.idx[1]];
			VERTEX v3 = tNodes[i].vertex[tr.idx[2]];
			v2 = doMinus(v2, v1);
			v3 = doMinus(v3, v1);
			VERTEX v0 = doCross(v2, v3);
			v0.doNormalize();
			vector<VERTEX> sharePoints;
			int unsharedIndex = -1;
			for (int k = 0; k < 3; k++) {
				int p = k + 1 == 3 ? 0 : k + 1;
				v1 = tNodes[i].vertex[tr.idx[k]];
				v2 = tNodes[i].vertex[tr.idx[p]];
				if (isConflict(v1, v2, &v3) == 1) {
					sharePoints.push_back(v3);
				}
				else {
					unsharedIndex = k;
				}
			}
			if (sharePoints.size() == 2 && unsharedIndex != -1) {
				int p = unsharedIndex + 2 >= 3 ? unsharedIndex - 1 : unsharedIndex + 2;
				int k = unsharedIndex + 1 == 3 ? 0 : unsharedIndex + 1;
				v1 = sharePoints[0];
				v2 = sharePoints[1];
				v3 = tNodes[i].vertex[tr.idx[p]];
				v1 = doMinus(v1, v3);
				v2 = doMinus(v2, v3);
				v1 = doCross(v1, v2);
				v1.doNormalize();
				v1 = doMinus(v1, v0);
				if (v1.getMagnitude() < 1E-3) {
					tNodes[i].vertex.push_back(sharePoints[0]);
					tNodes[i].vertex.push_back(sharePoints[1]);
				}
				else {
					tNodes[i].vertex.push_back(sharePoints[1]);
					tNodes[i].vertex.push_back(sharePoints[0]);
				}
				int vc = (int)(tNodes[i].vertex.size());
				model[i].index[j].idx[0] = vc - 2;
				model[i].index[j].idx[1] = vc - 1;
				model[i].index[j].idx[2] = tr.idx[p];
				model[i].index.push_back(TRIANGLE(vc - 2, tr.idx[k], vc - 1));
				model[i].index.push_back(TRIANGLE(vc - 2, tr.idx[unsharedIndex], tr.idx[k]));
			}
		}
	}
	vector<SUBNODE> tmp;

	for (UINT i = 0; i < model.size(); i++) {
		SUBNODE sb;
		for (UINT j = 0; j < model[i].index.size(); j++) {
			TRIANGLE tr = model[i].index[j];
			bool flag = false;

			for (int p = 0; p < 3; p++) {
				if (tNodes[i].vertex[tr.idx[p]].val[1] < 0) {
					model[i].index[j].isValid = false;
					flag = true;
					break;
				}
			}
		}
	}

	m = getUVEMatrix(origin, orgEye);
	doTransform(&tNodes, m);
	doSetVertexNormal();
}
//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE: Processes messages for the main window.
//
//  WM_COMMAND  - process the application menu
//  WM_PAINT    - Paint the main window
//  WM_DESTROY  - post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	static double lXpos = 0;
	static double lYpos = 0;
	static double stX = 0;
	static double stY = 0;

	RECT	rt;

    switch (message)
    {
	case WM_LBUTTONDOWN:
		lXpos = (double)GET_X_LPARAM(lParam);
		lYpos = (double)GET_Y_LPARAM(lParam);
		stX = lXpos;
		stY = lYpos;
		break;
	case WM_LBUTTONUP:
		if (isDrawing) {
			isDrawing = false;
			GetClientRect(GetDlgItem(hWnd, IDC_RENDER), &rt);
			double sX = ((float)RESIZED_WIDTH / (double)rt.right);
			double sY = ((float)RESIZED_HEIGHT / (double)rt.bottom);
			double xPos = (double)GET_X_LPARAM(lParam);
			double yPos = (double)GET_Y_LPARAM(lParam);

			stX -= (float)rt.right * 0.5f; xPos -= (float)rt.right * 0.5;
			stY -= (float)rt.bottom * 0.5f; yPos -= (float)rt.bottom * 0.5;
			//stY = -stY; yPos = -yPos;
			stX *= sX; stY *= sY; xPos *= sX; yPos *= sY;

			SUBNODE sb;
			double c = sqrt(3.0f);

			VERTEX v1 = VERTEX(stX, stY, zoomfactor * 0.6f);
			//VERTEX v2 = VERTEX(stX, stY, zoomfactor * 5.7f);
			//VERTEX v3 = VERTEX(xPos, yPos, zoomfactor * 5.7f);
			VERTEX v4 = VERTEX(xPos, yPos, zoomfactor * 0.6f);
			doFindRealPoint(&v1, c);
			doFindRealPoint(&v4, c);
			VERTEX v2 = v1;
			VERTEX v3 = v4;

			v2.val[2] += zoomfactor * 0.8f;
			v3.val[2] += zoomfactor * 0.8f;

			v1.val[2] -= zoomfactor;
			v2.val[2] -= zoomfactor;
			v3.val[2] -= zoomfactor;
			v4.val[2] -= zoomfactor;

			sb.vertex.push_back(v1);
			sb.vertex.push_back(v2);
			sb.vertex.push_back(v3);
			sb.vertex.push_back(v4);
			sb.index.push_back(TRIANGLE(0, 1, 2));
			sb.index.push_back(TRIANGLE(0, 2, 3));
			tNodes.push_back(sb);
			model.push_back(sb);
			isPlaneAdded = true;
		}
		break;
	case WM_MOUSEMOVE:
		{
			GetClientRect(GetDlgItem(hWnd, IDC_RENDER), &rt);
			double sX = ((float)RESIZED_WIDTH / (double)rt.right);
			double sY = ((float)RESIZED_HEIGHT / (double)rt.bottom);
			double xPos = (double)GET_X_LPARAM(lParam);
			double yPos = (double)GET_Y_LPARAM(lParam);

			xPos -= 10; yPos -= 10;
			if (xPos < rt.left || xPos > rt.right) break;
			if (yPos < rt.top || yPos > rt.bottom) break;

			if (wParam != MK_LBUTTON) break;
			if (isDrawing == false) {
				x_Rotate = (float)(yPos - lYpos) / rt.bottom;
				y_Rotate = (float)(xPos - lXpos) / rt.right;
				drawMesh();
				if (SendMessage(GetDlgItem(hWnd, IDC_MESH), BM_GETCHECK, 0, 0) == false) {
					doRender(zoomfactor, true);
					coatMaterial();
				}
				ShowResultImage(GetDlgItem(hWnd, IDC_RENDER), (BYTE *)renderImg);
			}
			else {
				memcpy(tmpImg, renderImg, RESIZED_HEIGHT * RESIZED_WIDTH * 4);
				DrawLine((UCHAR *)tmpImg, RESIZED_WIDTH, RESIZED_HEIGHT, stX * sX, stY * sY, xPos * sX, yPos * sY, 0, 255, 0);
				ShowResultImage(GetDlgItem(hWnd, IDC_RENDER), (BYTE *)tmpImg);
			}
			lXpos = xPos; lYpos = yPos;
		}
		break;
    case WM_COMMAND:
        {
            int wmId = LOWORD(wParam);
            // Parse the menu selections:
            switch (wmId)
            {
			case IDC_MESH:
				if (SendMessage(GetDlgItem(hWnd, IDC_MESH), BM_GETCHECK, 0, 0)) {
					drawMesh();
				}
				else {
					doRender(zoomfactor, true);
					coatMaterial();
				}
				ShowResultImage(GetDlgItem(hWnd, IDC_RENDER), (BYTE *)renderImg);
				break;
			case IDC_SEGMENT:
				if (isPlaneAdded == false) break;
				doSegment();
				isPlaneAdded = false;
				if (SendMessage(GetDlgItem(hWnd, IDC_MESH), BM_GETCHECK, 0, 0)) {
					drawMesh();
				}
				else {
					doRender(zoomfactor, true);
					coatMaterial();
				}
				ShowResultImage(GetDlgItem(hWnd, IDC_RENDER), (BYTE *)renderImg);
				break;
			case IDC_REMOVE:
				if (isPlaneAdded == false) break;
				tNodes.pop_back();
				model.pop_back();
				isPlaneAdded = false;
				if (SendMessage(GetDlgItem(hWnd, IDC_MESH), BM_GETCHECK, 0, 0)) {
					drawMesh();
				}
				else {
					doRender(zoomfactor, true);
					coatMaterial();
				}
				ShowResultImage(GetDlgItem(hWnd, IDC_RENDER), (BYTE *)renderImg);
				break;
			case IDC_DRAW:
				isDrawing = true;
				break;
            case IDC_BROWSE:
				{
					OPENFILENAME ofn;
					ZeroMemory(&ofn, sizeof(ofn));
					ofn.lStructSize = sizeof(ofn);
					ofn.hwndOwner = hWnd;
					ofn.lpstrFile = szFile;
					// Set lpstrFile[0] to '\0' so that GetOpenFileName does not 
					// use the contents of szFile to initialize itself.
					ofn.lpstrFile[0] = '\0';
					ofn.nMaxFile = sizeof(szFile);
					ofn.lpstrFilter = "obj\0*.obj";
					ofn.nFilterIndex = 1;
					ofn.lpstrFileTitle = NULL;
					ofn.nMaxFileTitle = 0;
					ofn.lpstrInitialDir = NULL;
					ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
					GetOpenFileName(&ofn);

					init3DOpt();

					if (loadasset(szFile) != 0) {
						MessageBox(hWnd, "Failed to load Asset!", "Image2Bin", MB_OK);
						break;
					}
					init_Render();
					zoomfactor = correctCenter(&tNodes);
					zoomfactor *= 3.0f;
					if (SendMessage(GetDlgItem(hWnd, IDC_MESH), BM_GETCHECK, 0, 0)) {
						drawMesh();
					}
					else {
						doRender(zoomfactor, true);
						coatMaterial();
					}
					ShowResultImage(GetDlgItem(hWnd, IDC_RENDER), (BYTE *)renderImg);
				}
                break;
            case IDCANCEL:
                DestroyWindow(hWnd);
				exit(0);
                break;
            default:
                return DefWindowProc(hWnd, message, wParam, lParam);
            }
        }
        break;
    case WM_PAINT:
        {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hWnd, &ps);
            // TODO: Add any drawing code that uses hdc here...
            EndPaint(hWnd, &ps);
        }
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
    }
    return 0;
}
