#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "usr\include\GL\freeglut.h"
#include "gmath.h"

// ��ũ�� ��� ����
#define MAX_DEPTH 1
#define H 768
#define W 1024
unsigned char Image[H * W * 3];

std::vector<GSphere> SphereList;
std::vector<GLight> LightList;

// �ݹ� �Լ� ����
void Render();
void Reshape(int w, int h);
void Timer(int id);

// ���� ���� �Լ�
void CreateImage();
GVec3 RayTrace(GLine ray, int depth);
GVec3 Phong(GPos3 P, GVec3 N, GSphere Obj);
bool intersect_line_sphere(GLine ray, int &sidx, double &t);

//�߰� ����0
class MyPlane
{
public:
	GPlane pl = GPlane(GVec3(), GPos3());
	GVec3 Ka;		// �ֺ��� �ݻ���
	GVec3 Kd;		// ���ݻ� ���
	GVec3 Ks;		// ���ݻ� ���	
	double ns;		// ���ݻ� ����
	bool bTransparent;	// ���� ����
	MyPlane(GPlane plane, GVec3 Ka, GVec3 Kd, GVec3 Ks, double ns, bool bTransparent)
	{
		this->pl = plane;
		this->Ka = Ka;
		this->Kd = Kd;
		this->Ks = Ks;
		this->ns = ns;
		this->bTransparent = bTransparent;
	}
};
GVec3 Phong_plane(GPos3 P, MyPlane plane);
GPlane pl(GVec3(0, 1, 0.01), GPos3(0, -150, 400));
MyPlane plane(pl, GVec3(0.2, 0.2, 0.2), GVec3(0.0, 0.7, 0.7), GVec3(0.8, 0.8, 0.8), 8.0, false);

//�߰� ����0 ��

int main(int argc, char **argv)
{
	// OpenGL �ʱ�ȭ, ������ ũ�� ����, ���÷��� ��� ����
	glutInit(&argc, argv);
	glutInitWindowSize(W, H);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	// ������ ���� �� �ݹ� �Լ� ���
	glutCreateWindow("RayTracer");
	glutDisplayFunc(Render);
	glutReshapeFunc(Reshape);
	glutTimerFunc(10, Timer, 0);

	// ���� ����
	GLight Light0;
	Light0.Pos.Set(-500.0, 200.0, 5000.0);
	Light0.Ia.Set(0.2, 0.2, 0.2);
	Light0.Id.Set(1.0, 1.0, 1.0);
	Light0.Is.Set(1.0, 1.0, 1.0);
	LightList.push_back(Light0);

	GLight Light1;
	Light1.Pos.Set(-300.0, 300.0, -1000.0);
	Light1.Ia.Set(0.2, 0.2, 0.2);
	Light1.Id.Set(1.0, 1.0, 1.0);
	Light1.Is.Set(0.8, 0.8, 0.8);
	LightList.push_back(Light1);

	// ��鿡 3���� ���� ��ġ�Ѵ�.
	GSphere Sphere0;
	Sphere0.Pos.Set(0, 0, -500.0);
	Sphere0.Rad = 50.0;
	Sphere0.Ka.Set(0.1, 0.1, 0.1);
	Sphere0.Kd.Set(0.8, 0.8, 0.8);
	Sphere0.Ks.Set(0.9, 0.9, 0.9);
	Sphere0.ns = 8.0;
	Sphere0.bTransparent = true;
	SphereList.push_back(Sphere0);
	
	GSphere Sphere1;
	Sphere1.Pos.Set(-80, 80, -650.0);
	Sphere1.Rad = 50.0;
	Sphere1.Ka.Set(0.2, 0.2, 0.2);
	Sphere1.Kd.Set(0.7, 0.0, 0.0);
	Sphere1.Ks.Set(0.8, 0.8, 0.8);
	Sphere1.ns = 8.0;
	SphereList.push_back(Sphere1);

	GSphere Sphere2;
	Sphere2.Pos.Set(-80, -80, -650.0);
	Sphere2.Rad = 50.0;
	Sphere2.Ka.Set(0.2, 0.2, 0.2);
	Sphere2.Kd.Set(0.0, 0.7, 0.0);
	Sphere2.Ks.Set(0.8, 0.8, 0.8);
	Sphere2.ns = 8.0;		
	SphereList.push_back(Sphere2);

	GSphere Sphere3;
	Sphere3.Pos.Set(80, -80, -650.0);
	Sphere3.Rad = 50.0;
	Sphere3.Ka.Set(0.2, 0.2, 0.2);
	Sphere3.Kd.Set(0.0, 0.0, 0.7);
	Sphere3.Ks.Set(0.8, 0.8, 0.8);
	Sphere3.ns = 8.0;
	SphereList.push_back(Sphere3);

	GSphere Sphere4;
	Sphere4.Pos.Set(80, 80, -650.0);
	Sphere4.Rad = 50.0;
	Sphere4.Ka.Set(0.2, 0.2, 0.2);
	Sphere4.Kd.Set(0.0, 0.7, 0.7);
	Sphere4.Ks.Set(0.8, 0.8, 0.8);
	Sphere4.ns = 8.0;
	SphereList.push_back(Sphere4);

	// �̹����� ����
	CreateImage();

	// �̺�Ʈ�� ó���� ���� ���� ������ �����Ѵ�.
	glutMainLoop();
	
	return 0;
}

void Reshape(int w, int h)
{
	glViewport(0, 0, w, h);
}

void Render()
{
	// Į�� ���ۿ� ���� ���� �����
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Į�� ���ۿ� Image �����͸� ���� �׸���.
	glDrawPixels(W, H, GL_RGB, GL_UNSIGNED_BYTE, Image);
	
	// Į�� ���� ��ȯ�Ѵ�
	glutSwapBuffers();
}

void Timer(int id)
{
	static double theta = 0.0;
	theta += 0.1;
	double x = 70 * cos(theta);
	double y = 70 * sin(theta);
	SphereList[0].Pos[0] = x;
	SphereList[0].Pos[1] = y;
	
	CreateImage();
	glutPostRedisplay();
	glutTimerFunc(1, Timer, 0);
}

void CreateImage()
{
	int x0 = -W / 2;
	int y0 = H / 2 - 1;
	double z = -(H / 2) / tan(M_PI * 15 / 180.0);
	for (int i = 0; i < H; ++i)
	{
		for (int j = 0; j < W; ++j)
		{
			double x = x0 + j;
			double y = y0 - i;
			GLine ray(GPos3(0.0, 0.0, 0.0), GPos3(x, y, z));
			GVec3 Color = RayTrace(ray, 0);

			int idx = ((H - 1 - i) * W + j) * 3;
			unsigned char r = (Color[0] > 1.0) ? 255 : (unsigned int)(Color[0] * 255);
 			unsigned char g = (Color[1] > 1.0) ? 255 : (unsigned int)(Color[1] * 255);
 			unsigned char b = (Color[2] > 1.0) ? 255 : (unsigned int)(Color[2] * 255);
			Image[idx] = r;
			Image[idx + 1] = g;
			Image[idx + 2] = b;
		}
	}
}


//�߰�����2
void make_shadow(GPos3 P, GVec3 &C)
{
	bool blocked;
	for (int i = 0; i < LightList.size(); i++)
	{
		blocked = false;
		GLight light = LightList[i];
		GPos3 L = light.Pos;
		GLine reverse_light_line(P, L);
		int sidx;
		double t;
		GPos3 P_plane;
		blocked = intersect_line_sphere(reverse_light_line, sidx, t);
		if (blocked == true) C -= GVec3(0.05, 0.05, 0.05);
	}

}
//�߰�����2��

//�߰� ����1

void set_plane_color(GLine ray, int depth, GVec3 &C)
{
	GPos3 P;
	if (intersect_line_plane(P, ray, plane.pl))
	{
		GVec3 N, V, R, T;

		N = plane.pl.n; N.Normalize();
		V = ray.v; V.Normalize();
		R = V - 2 * (N*V)*N;
		GLine ray_reflect(P, R);

		C = Phong_plane(P, plane) + 0.3*RayTrace(ray_reflect, depth);

		make_shadow(P, C);
		/*
		double n1 = 1.0f, n2 = 1.05f;
		double a = n1 / n2;
		double cos1 = N * (-V), cos2 = sqrt(1 - a * a*(1 - cos1 * cos1));
		double b = cos2 - a * cos1;
		T = a * V - b * N; T.Normalize();//���� ���� ����
		GLine ray_refract(P, T);

		C = Phong(P, N, SphereList[4]) + 0.3*RayTrace(ray_reflect, depth) + 0.3*RayTrace(ray_refract, depth);
		*/
	}
}

//�߰� ����1 ��

//����0
//���� �������� ���� ������
GPos3 cross_inner_refract(GLine inner_refract, GSphere s)
{
	GPos3 p0 = inner_refract.p;
	GPos3 q = s.Pos;
	double r = s.Rad;
	GVec3 u = p0 - q;
	GVec3 v = inner_refract.v;
	double t = -(u*v) + sqrt(pow(u*v, 2) - (u*u - r * r));
	return inner_refract.Eval(t);
}
//����0 ��

void set_sphere_color(GLine ray, int depth, GVec3 &C)
{
	int sidx; // ������ �����ϴ� ���� ����� ���� �ε���
	double t; // ���������� ������ �Ķ���� t
	if (intersect_line_sphere(ray, sidx, t))
	{
		//������������ �ݻ籤���� �����Ѵ�.
		GPos3 P = ray.Eval(t);
		GVec3 N, V, R, T;

		//����1
		//����, �ü�, �ݻ�, ���� ���� ����
		N = P - SphereList[sidx].Pos; N.Normalize();
		V = ray.v;
		R = V - 2 * (N*V)*N;
		//����1 ��

		GLine ray_reflect(P, R);

		//����2
		//���� ������ �� ���� �������ϱ�
		if (SphereList[sidx].bTransparent == true)
		{
			double n1 = 1.0f, n2 = 1.05f;
			double a = n1 / n2;
			double cos1 = N * (-V), cos2 = sqrt(1 - a * a*(1 - cos1 * cos1));
			double b = cos2 - a * cos1;
			T = a * V - b * N; T.Normalize();// ���� ���� ���� ����
			GLine inner_refract(P, T);// ���� ���� ��

			GPos3 P_out = cross_inner_refract(inner_refract, SphereList[sidx]);//�ܺ� ������
			GVec3 V_out = inner_refract.v; V_out.Normalize();//�ܺ� ���� �Ի� ���� ����
			GVec3 N_out = SphereList[sidx].Pos - P_out; N_out.Normalize();//�ܺ� ���� ����
			a = 1 / a;
			cos1 = N_out * (-V_out), cos2 = sqrt(1 - a * a*(1 - cos1 * cos1));
			b = cos2 - a * cos1;
			GVec3 T_out = a * V_out - b * N_out; T_out.Normalize();
			GLine outter_refract(P_out, T_out);//�ܺ� ���� ��

			C = Phong(P, N, SphereList[sidx]) + 0.3*RayTrace(ray_reflect, depth) + 0.3*RayTrace(outter_refract, depth);
		}
		else
		{
			C = Phong(P, N, SphereList[sidx]) + 0.3*RayTrace(ray_reflect, depth);
		}

		make_shadow(P, C);
		//����2 ��

		/* ���� �־����� �ڵ�
		GLine ray_refract(P, T);

		// ����......

		//���� ���� �����ϰ�, �ݻ� ������ ���� ������ ���Ͽ� ��� ȣ�� �Ѵ�.
		C = Phong(P, N, SphereList[sidx])
			+ 0.3*RayTrace(ray_reflect, depth)
			+ 0.3*RayTrace(ray_refract, depth);

		* ���� �־����� �ڵ� ��
		*/
	}
}

GVec3 RayTrace(GLine ray, int depth)
{
	// ���� ����
	GVec3 C(0.0, 0.0, 0.0);
 	if (depth++ > MAX_DEPTH)
 		return C;
	
	set_plane_color(ray, depth, C);
	set_sphere_color(ray, depth, C);

	return C;
}

/*
* \brief ��鿡�� ������ �����ϴ� ���� �ε��� (sidx)�� �����ϴ� �ð�(t)�� ã�´�.
* \param ray ������ ������
* \param sidx ������ �����ϴ� ���� �ε����� �����
* \param t ������ ���� �����ϴ� �ð�(�Ķ����)�� �����
*
* \return ������ ���� �����ϸ� true, �ƴϸ� false�� ��ȯ�Ѵ�.
*/
bool intersect_line_sphere(GLine ray, int &sidx, double &t)
{
	sidx = -1;
	t = 100000.0;
	int i;
	GSphere s;
	GPos3 p0, q;
	GVec3 u, v = ray.v;
	double r, det, t1, t2;
	for (i = 0; i < SphereList.size(); i++)
	{
		s = SphereList[i];
		p0 = ray.p;
		q = s.Pos;
		u = p0 - q;
		r = s.Rad;
		det = (u*v)*(u*v) - (u*u - r * r);
		if (det >= 0.0)
		{
			t1 = -(u*v) - sqrt(det);
			t2 = -(u*v) + sqrt(det);
			if (t1>0  && t1 < t)
			{
				sidx = i;
				t = t1;
			}
		}
	}
	if (sidx == -1) return false;
	else return true;
}

/*
* \brief ���� ���� ���� ������������ ������ ����Ѵ�.
* \param P �������� ��ǥ
* \param N �������� ����
* \param Obj �����ϴ� �� (������ ���� ����)
*
* \return ���� ������ ��ȯ�Ѵ�.
*/
GVec3 Phong(GPos3 P, GVec3 N, GSphere Obj)
{
	GVec3 C;
	
	int i;
	GLight light;
	GVec3 L, V, R, a, d, s;
	for (i = 0; i < LightList.size(); i++)
	{
		light = LightList[i];
		L = light.Pos - P; L.Normalize();
		V = GPos3(0, 0, 0) - P; V.Normalize();
		R = 2 * (N*L)*N - L; R.Normalize();

		a = GVec3(light.Ia[0] * Obj.Ka[0], light.Ia[1] * Obj.Ka[1], light.Ia[2] * Obj.Ka[2]);
		d = GVec3(light.Id[0] * Obj.Kd[0], light.Id[1] * Obj.Kd[1], light.Id[2] * Obj.Kd[2])*MAX(0, N*L);
		s = GVec3(light.Is[0] * Obj.Ks[0], light.Is[1] * Obj.Ks[1], light.Is[2] * Obj.Ks[2])*pow(MAX(0, V*R), Obj.ns);

		C += a + d + s;
	}
	return C;
}

//�߰�����3
GVec3 Phong_plane (GPos3 P, MyPlane plane)
{
	GVec3 C;

	int i;
	GLight light;
	GVec3 N = plane.pl.n; N.Normalize();
	
	GVec3 L, V, R, a, d, s;
	
	for (i = 0; i < LightList.size(); i++)
	{
		light = LightList[i];
		L = light.Pos - P; L.Normalize();
		V = GPos3(0, 0, 0) - P; V.Normalize();
		R = 2 * (N*L)*N - L; R.Normalize();

		a = GVec3(light.Ia[0] * (plane.Ka)[0], light.Ia[1] * (plane.Ka)[1], light.Ia[2] * (plane.Ka)[2]);
		d = GVec3(light.Id[0] * (plane.Kd)[0], light.Id[1] * (plane.Kd)[1], light.Id[2] * (plane.Kd)[2])*MAX(0, N*L);
		s = GVec3(light.Is[0] * (plane.Ks)[0], light.Is[1] * (plane.Ks)[1], light.Is[2] * (plane.Ks)[2])*pow(MAX(0, V*R), plane.ns);
		
		C += a + d + s;
	}

	return C;
}
//�߰�����3 ��