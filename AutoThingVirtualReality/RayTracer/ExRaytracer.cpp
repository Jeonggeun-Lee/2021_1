#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "usr\include\GL\freeglut.h"
#include "gmath.h"

// 매크로 상수 정의
#define MAX_DEPTH 1
#define H 768
#define W 1024
unsigned char Image[H * W * 3];

std::vector<GSphere> SphereList;
std::vector<GLight> LightList;

// 콜백 함수 선언
void Render();
void Reshape(int w, int h);
void Timer(int id);

// 광선 추적 함수
void CreateImage();
GVec3 RayTrace(GLine ray, int depth);
GVec3 Phong(GPos3 P, GVec3 N, GSphere Obj);
bool intersect_line_sphere(GLine ray, int &sidx, double &t);

//추가 구현0
class MyPlane
{
public:
	GPlane pl = GPlane(GVec3(), GPos3());
	GVec3 Ka;		// 주변광 반사계수
	GVec3 Kd;		// 난반사 계수
	GVec3 Ks;		// 전반사 계수	
	double ns;		// 전반사 지수
	bool bTransparent;	// 투명성 여부
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

//추가 구현0 끝

int main(int argc, char **argv)
{
	// OpenGL 초기화, 윈도우 크기 설정, 디스플레이 모드 설정
	glutInit(&argc, argv);
	glutInitWindowSize(W, H);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	// 윈도우 생성 및 콜백 함수 등록
	glutCreateWindow("RayTracer");
	glutDisplayFunc(Render);
	glutReshapeFunc(Reshape);
	glutTimerFunc(10, Timer, 0);

	// 조명 설정
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

	// 장면에 3개의 구를 배치한다.
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

	// 이미지를 생성
	CreateImage();

	// 이벤트를 처리를 위한 무한 루프로 진입한다.
	glutMainLoop();
	
	return 0;
}

void Reshape(int w, int h)
{
	glViewport(0, 0, w, h);
}

void Render()
{
	// 칼라 버퍼와 깊이 버퍼 지우기
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// 칼라 버퍼에 Image 데이터를 직접 그린다.
	glDrawPixels(W, H, GL_RGB, GL_UNSIGNED_BYTE, Image);
	
	// 칼라 버퍼 교환한다
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


//추가구현2
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
//추가구현2끝

//추가 구현1

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
		T = a * V - b * N; T.Normalize();//굴절 선의 벡터
		GLine ray_refract(P, T);

		C = Phong(P, N, SphereList[4]) + 0.3*RayTrace(ray_reflect, depth) + 0.3*RayTrace(ray_refract, depth);
		*/
	}
}

//추가 구현1 끝

//구현0
//내부 굴절선과 구의 교차점
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
//구현0 끝

void set_sphere_color(GLine ray, int depth, GVec3 &C)
{
	int sidx; // 광선과 교차하는 가장 가까운 구의 인덱스
	double t; // 교차점에서 광선의 파라미터 t
	if (intersect_line_sphere(ray, sidx, t))
	{
		//교차점에서의 반사광선을 생성한다.
		GPos3 P = ray.Eval(t);
		GVec3 N, V, R, T;

		//구현1
		//법선, 시선, 반사, 굴절 벡터 설정
		N = P - SphereList[sidx].Pos; N.Normalize();
		V = ray.v;
		R = V - 2 * (N*V)*N;
		//구현1 끝

		GLine ray_reflect(P, R);

		//구현2
		//구가 투명할 때 굴절 광선구하기
		if (SphereList[sidx].bTransparent == true)
		{
			double n1 = 1.0f, n2 = 1.05f;
			double a = n1 / n2;
			double cos1 = N * (-V), cos2 = sqrt(1 - a * a*(1 - cos1 * cos1));
			double b = cos2 - a * cos1;
			T = a * V - b * N; T.Normalize();// 내부 굴절 선의 벡터
			GLine inner_refract(P, T);// 내부 굴절 선

			GPos3 P_out = cross_inner_refract(inner_refract, SphereList[sidx]);//외부 굴절점
			GVec3 V_out = inner_refract.v; V_out.Normalize();//외부 굴절 입사 선의 방향
			GVec3 N_out = SphereList[sidx].Pos - P_out; N_out.Normalize();//외부 굴절 법선
			a = 1 / a;
			cos1 = N_out * (-V_out), cos2 = sqrt(1 - a * a*(1 - cos1 * cos1));
			b = cos2 - a * cos1;
			GVec3 T_out = a * V_out - b * N_out; T_out.Normalize();
			GLine outter_refract(P_out, T_out);//외부 굴절 선

			C = Phong(P, N, SphereList[sidx]) + 0.3*RayTrace(ray_reflect, depth) + 0.3*RayTrace(outter_refract, depth);
		}
		else
		{
			C = Phong(P, N, SphereList[sidx]) + 0.3*RayTrace(ray_reflect, depth);
		}

		make_shadow(P, C);
		//구현2 끝

		/* 원래 주어졌던 코드
		GLine ray_refract(P, T);

		// 구현......

		//조명 모델을 적용하고, 반사 광선과 굴절 광선에 대하여 재귀 호출 한다.
		C = Phong(P, N, SphereList[sidx])
			+ 0.3*RayTrace(ray_reflect, depth)
			+ 0.3*RayTrace(ray_refract, depth);

		* 원래 주어졌던 코드 끝
		*/
	}
}

GVec3 RayTrace(GLine ray, int depth)
{
	// 종료 조건
	GVec3 C(0.0, 0.0, 0.0);
 	if (depth++ > MAX_DEPTH)
 		return C;
	
	set_plane_color(ray, depth, C);
	set_sphere_color(ray, depth, C);

	return C;
}

/*
* \brief 장면에서 광선과 교차하는 구의 인덱스 (sidx)와 교차하는 시간(t)를 찾는다.
* \param ray 광선의 방정식
* \param sidx 광선과 교차하는 구의 인덱스가 저장됨
* \param t 광선이 구와 교차하는 시간(파라미터)가 저장됨
*
* \return 광선이 구와 교차하면 true, 아니면 false를 반환한다.
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
* \brief 조명 모델을 통해 교차점에서의 색상을 계산한다.
* \param P 교차점의 좌표
* \param N 교차점의 법선
* \param Obj 교차하는 구 (재질의 정보 포함)
*
* \return 계산된 색상을 반환한다.
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

//추가구현3
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
//추가구현3 끝