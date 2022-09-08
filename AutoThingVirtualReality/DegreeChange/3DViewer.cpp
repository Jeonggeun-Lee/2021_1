#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <vector>
#include ".\usr\include\GL\freeglut.h"
#include "BzrCrv.h"

// ������ ũ��
int Width = 800, Height = 800;

// ��� ������ ���� ������
int ManipulateMode = 0; // 1: ȸ��, 2: �̵�
int StartPt[2];
float Axis[3] = {1.0, 0.0, 0.0};
float Angle = 0.0;
float RotMat[16] = {1, 0, 0, 0, 
					0, 1, 0, 0, 
					0, 0, 1, 0, 
					0, 0, 0, 1};
float Zoom = -30.0;
float Pan[3] = { 0.0, 0.0, 0.0 };

// �ݹ� �Լ���
void Reshape(int w, int h);
void Mouse(int button, int state, int x, int y);
void Motion(int x, int y);
void MouseWheel(int button, int dir, int x, int y);
void Render();
void Keyboard(unsigned char key, int x, int y);

// ����� ���� �Լ���
void GetSphereCoord(int x, int y, float *px, float *py, float *pz);
void RenderFloor();
void SetupViewVolume();
void SetupViewTransform();
void CreateMenu(int id);

enum TypeCreate
{
	CREATE_NONE = -1,
	CREATE_BZR_CRV = 0,
};
TypeCreate CreateType = CREATE_NONE;
std::vector<GBzrCrv> BzrCrvList;


////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	// GLUT �ʱ�ȭ(���� Į����� ���)
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA); 

	// ������ ����
	glutInitWindowSize(Width, Height);
	glutCreateWindow("3DViewer-BzrCrvTool");
	
	// �ݹ��Լ� ���
	glutReshapeFunc(Reshape);
	glutMouseFunc(Mouse);
	glutMotionFunc(Motion); // ���콺 ��ư ������ ������ ��, �ڵ����� ȣ��Ǵ� �Լ�
	glutMouseWheelFunc(MouseWheel);	// 
	glutDisplayFunc(Render);
	glutKeyboardFunc(Keyboard);

	// �޴� ����
	GLint CreateMenuId = glutCreateMenu(CreateMenu);
	glutAddMenuEntry("Bezier Curve", 0);
	glutAddMenuEntry("Increase Deg.", 1);
	glutAddMenuEntry("Decrease Deg.", 2);
	glutAddMenuEntry("Interpolation Curve", 3);
	glutAddMenuEntry("Hermite Curve", 4);
	glutAddMenuEntry("Exit", 5);
	glutAttachMenu(GLUT_MIDDLE_BUTTON);
			
	// �޽��� ���� ����
	glutMainLoop();
	return 0;
}

void CreateMenu(int id)
{
	switch (id)
	{
	case 0:
		CreateType = CREATE_BZR_CRV;
		BzrCrvList.push_back(GBzrCrv());
		break;
	case 1: // ���� ���̱�
		BzrCrvList.back().DegIncrease();
		break;
	case 2: // ���� ���߱�
		BzrCrvList.back().DegDecrease();
		break;
	case 3:
		break;
	case 4:
		break;
	case 5:
		exit(0);
	}
	glutPostRedisplay();
}

void Render()
{
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	SetupViewVolume();
	SetupViewTransform();

	// ��� ������
	glMatrixMode(GL_MODELVIEW);

	RenderFloor();
	for (int i = 0; i < BzrCrvList.size(); ++i)
		BzrCrvList[i].Render(200);
			   
	// ���� ���۸��� ���� ���� ��ȯ
	glutSwapBuffers();
}

void Reshape(int w, int h)
{
	// ����Ʈ ��ȯ
	glViewport(0, 0, w, h);
	Width = w;
	Height = h;
}

void SetupViewVolume()
{
	// ���� ���� ����
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30.0, (double)Width / (double)Height, 1.0, 10000.0);
}

void SetupViewTransform()
{
	// �� �� ����� ���� ��ķ� �ʱ�ȭ, M = I
	glMatrixMode(GL_MODELVIEW);	
	glLoadIdentity();

	// �� ��/�ƿ��� ���� ��ȯ, M = I * T_zoom
	glTranslatef(0.0, 0.0, Zoom); 

	// ���ο� ȸ���� ����, M = I * T_zoom * R_new
	glRotatef(Angle, Axis[0], Axis[1], Axis[2]);

	// ���� ȸ���� ����, M = I * T_zoom * R_new * R_old	//   R_n .... * R3 * R2 * R1
	glMultMatrixf(RotMat);	

	// ȸ�� ��� ����, R_old = R_new * R_old
	glGetFloatv(GL_MODELVIEW_MATRIX, RotMat);
	RotMat[12] = RotMat[13] = RotMat[14] = 0.0;

	// �̵� ��ȯ�� ����, M = I * T_zoom * R_new * R_old * T_pan
	glTranslatef(Pan[0], Pan[1], Pan[2]);	
}

void RenderFloor()
{
	glColor3f(0.0f, 0.0f, 0.0f);
	for (float x = -10.0; x <= 10.0; x += 1.0)
	{
		glBegin(GL_LINES);
		glVertex3f(x, 0.0, -10.0f);
		glVertex3f(x, 0.0, 10.0f);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(-10.0f, 0.0, x);
		glVertex3f(10.0f, 0.0, x);
		glEnd();
	}
}

void Keyboard(unsigned char key, int x, int y)
{
	if (key == 27) // ESC Ű...
		CreateType = CREATE_NONE;
}

void Mouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
	{
		StartPt[0] = x; 
		StartPt[1] = y;
		if (button == GLUT_LEFT_BUTTON)
			ManipulateMode = 1;	// ȸ��

		if (button == GLUT_RIGHT_BUTTON)
			ManipulateMode = 2;	// �̵�
	}
	if (state == GLUT_UP)		
	{	
		ManipulateMode = 0;	// ����
		StartPt[0] = StartPt[1] = 0;
		Angle = 0.0;

		// ������ ��� ���� ���̶��...
		if (CreateType == CREATE_BZR_CRV)
		{
			// �ٴ� ������ �������� ���Ѵ�.
			int viewport[4];
			double mv[16], pr[16];	
			glGetIntegerv(GL_VIEWPORT, viewport);
			glGetDoublev(GL_MODELVIEW_MATRIX, mv);
			glGetDoublev(GL_PROJECTION_MATRIX, pr);			
			GPos3 p, q;
			gluUnProject((double)x, viewport[3] - (double)y, 0.0, mv, pr, viewport, &p[0], &p[1], &p[2]);
			gluUnProject((double)x, viewport[3] - (double)y, 0.5, mv, pr, viewport, &q[0], &q[1], &q[2]);
			GLine ray(p, q);
			GPos3 pt;
			GPlane pi(GVec3(0.0, 1.0, 0.0), GPos3(0.0, 0.0, 0.0));	
			::intersect_line_plane(pt, ray, pi);

			// �������� ������ ��� ���������� �߰��Ѵ�.
			BzrCrvList.back().AddCtrlPt(GVec3(pt[0], pt[1], pt[2]));
		}
	}
}

void Motion(int x, int y)
{
	// ȸ����� ȸ�� ���� ���
	if (ManipulateMode == 1)
	{	
		// ���� �� ���� ��ǥ ���
		float px, py, pz, qx, qy, qz;
		GetSphereCoord(StartPt[0], StartPt[1], &px, &py, &pz);
		GetSphereCoord(x, y, &qx, &qy, &qz);

		// ȸ�� ��� ���� ���
		Axis[0] = py * qz -pz * qy; 
		Axis[1] = pz * qx -px * qz; 
		Axis[2] = px * qy -py * qx;
		Angle = 0.0;
		float len = Axis[0] * Axis[0] + Axis[1] * Axis[1] + Axis[2] * Axis[2];
		if (len > 0.000001) // ���� ���� �̻� ó��
			Angle = acos(px * qx + py * qy + pz * qz) * 180.0f / 3.141592f;
	}

	// �̵� ���� ���
	if (ManipulateMode == 2)
	{
		float dx = (float)(x - StartPt[0]) * 0.01f;
		float dy = (float)(StartPt[1] - y) * 0.01f;
		// ȸ�� ��� �� �����
		// R = 0 4 8   invR = 0 1 2
		//     1 5 9          4 5 6    
		//     2 6 10         8 9 10
		// invR * (dx, dy, 0)
		Pan[0] += RotMat[0] * dx + RotMat[1] * dy;
		Pan[1] += RotMat[4] * dx + RotMat[5] * dy;
		Pan[2] += RotMat[8] * dx + RotMat[9] * dy;
	}

	StartPt[0] = x;	// Update startpt as current position
	StartPt[1] = y;
	glutPostRedisplay();
}

/*!
*	\brief ���콺 ��ũ���� ó���ϴ� �ݹ� �Լ�
*
*	\param button[in]	���콺 ��ư ����(GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, GLUT_RIGHT_BUTTON)
*	\param dir[in]		��ũ���� ����
*	\param x[in]		���� ����� (0, 0) �������� ���콺 �������� ���� ��ġ
*	\param y[in]		���� ����� (0, 0) �������� ���콺 �������� ���� ��ġ
*/
void MouseWheel(int button, int dir, int x, int y)
{
	if (dir > 0)
		Zoom += 1.0;
	else
		Zoom -= 1.0;
	glutPostRedisplay();
}

void GetSphereCoord(int x, int y, float *px, float *py, float *pz)
{
	*px = (2.0f * x - Width) / Width;
	*py = (-2.0f * y + Height) / Height;

	float r = (*px) * (*px) + (*py) * (*py); // �������κ����� �Ÿ� ���
	if (r >= 1.0f)
	{
		*px = *px / sqrt(r);
		*py = *py / sqrt(r);
		*pz = 0.0f;
	}
	else
		*pz = sqrt(1.0f - r);  // �Ϲ����� ���
}

