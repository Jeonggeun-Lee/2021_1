#include "BzrCrv.h"

/*!
*	\brief	������ ��� �������� �߰��Ѵ�.
*
*	\param	pt[in]	�߰��� ������
*/
void GBzrCrv::AddCtrlPt(GVec3 pt)
{
	m_Points.push_back(pt);
}

/*!
*	\brief � ���� �� C(t)�� ���� ���� C'(t)�� ����Ѵ�.
*
*	\param	t[in]	��� �Ķ����
*	\param	p[out]	� ���� �� C(t)�� ����ȴ�.
*	\param	v[out]	���� ���� C'(t)�� ����ȴ�.
*/
void GBzrCrv::Eval(double t, GVec3 &p, GVec3 &v)
{
	std::vector<GVec3> Points = m_Points;
	int NumPts = (int)Points.size();

	// de_Casteljau �˰����� �����Ѵ�.
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
*	\brief	��� ������ �Ѵ�.
*
*	\param	NumOfPts[in]	� ���� ���� ��
*/
void GBzrCrv::Render(int NumOfPts)
{
	if (m_Points.empty())
		return;

	int NumPts = (int)m_Points.size();	

	// �������� ������ �Ѵ�.
	glColor3d(1.0, 0.0, 0.0);
	glPointSize(5.0f);
	glBegin(GL_POINTS);
	{
		for (int i = 0; i < NumPts; ++i)
			glVertex3d(m_Points[i][0], m_Points[i][1], m_Points[i][2]);
	}
	glEnd();

	// ����ٰ����� ������ �Ѵ�.
	glColor3d(0.0, 1.0, 0.0);
	glLineWidth(2.0);
	glBegin(GL_LINE_STRIP);
	{
		for (int i = 0; i < NumPts; ++i)
			glVertex3d(m_Points[i][0], m_Points[i][1], m_Points[i][2]);
	}
	glEnd();

	// ��� ������ �Ѵ�.
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
*	\brief	��� ������ ���δ�.
*/
void GBzrCrv::DegIncrease()
{
	// �����ϼ���.
}

/*!
*	\brief	��� ������ �����.
*/
void GBzrCrv::DegDecrease()
{
	// �����ϼ���.
}
