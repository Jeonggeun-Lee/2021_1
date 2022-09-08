#include "BzrCrv.h"

/*!
*	\brief	베지에 곡선의 제어점을 추가한다.
*
*	\param	pt[in]	추가할 제어점
*/
void GBzrCrv::AddCtrlPt(GVec3 pt)
{
	m_Points.push_back(pt);
}

/*!
*	\brief 곡선 위의 점 C(t)와 접선 벡터 C'(t)를 계산한다.
*
*	\param	t[in]	계산 파라미터
*	\param	p[out]	곡선 위의 점 C(t)가 저장된다.
*	\param	v[out]	접선 벡터 C'(t)가 저장된다.
*/
void GBzrCrv::Eval(double t, GVec3 &p, GVec3 &v)
{
	std::vector<GVec3> Points = m_Points;
	int NumPts = (int)Points.size();

	// de_Casteljau 알고리즘을 수행한다.
	for (int i = 1; i < NumPts; ++i) // i = 1, 2, 3 ... NumPts - 1
	{
		for (int j = 0; j < NumPts - i; ++j)
			Points[j] = (1.0 - t) * Points[j] + t * Points[j + 1];
		
		if (i == NumPts - 2)
			v = (NumPts - 1) * (Points[1] - Points[0]);
	}
	p = Points[0];
}

/*!
*	\brief	곡선을 렌더링 한다.
*
*	\param	NumOfPts[in]	곡선 위의 점의 수
*/
void GBzrCrv::Render(int NumOfPts)
{
	if (m_Points.empty())
		return;

	int NumPts = (int)m_Points.size();	

	// 제어점을 렌더링 한다.
	glColor3d(1.0, 0.0, 0.0);
	glPointSize(5.0f);
	glBegin(GL_POINTS);
	{
		for (int i = 0; i < NumPts; ++i)
			glVertex3d(m_Points[i][0], m_Points[i][1], m_Points[i][2]);
	}
	glEnd();

	// 제어다각형을 렌더링 한다.
	glColor3d(0.0, 1.0, 0.0);
	glLineWidth(2.0);
	glBegin(GL_LINE_STRIP);
	{
		for (int i = 0; i < NumPts; ++i)
			glVertex3d(m_Points[i][0], m_Points[i][1], m_Points[i][2]);
	}
	glEnd();

	// 곡선을 렌더링 한다.
	glColor3f(0.0, 0.0, 1.0);
	glLineWidth(3.0);
	glBegin(GL_LINE_STRIP);
	{
		GVec3 p, v;
		for (int i = 0; i < NumOfPts; ++i)
		{
			double t = (double)i / (double)(NumOfPts - 1); // 0 <= t <= 1
			Eval(t, p, v);
			glVertex3d(p[0], p[1], p[2]);
		}
	}
	glEnd();
	glLineWidth(1.0);
	glPointSize(1.0);
}

/*!
*	\brief	곡선의 차수를 높인다.
*/
void GBzrCrv::DegIncrease()
{
	// 구현하세요.
}

/*!
*	\brief	곡선의 차수를 낮춘다.
*/
void GBzrCrv::DegDecrease()
{
	// 구현하세요.
}
