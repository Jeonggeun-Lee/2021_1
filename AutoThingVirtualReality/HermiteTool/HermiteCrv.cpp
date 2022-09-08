#include "HermiteCrv.h"

void GHermiteCrv::AddInterpPt(GVec3 pt)
{
	// 기존의 곡선 리스트를 제거한다.
	m_Crvs.clear();

	// 보간점을 추가하고
	m_Points.push_back(pt);
	
	// 새로운 허밋 보간 곡선을 생성하여 곡선 리스트에 추가한다.
	if (m_Points.size() > 1)
	{
		// 1. 보간 파라미터를 결정한다.
		std::vector<float> params;//파라미터들을 저장할 벡터
		float sum = 0;//제 0 파라미터 값
		params.push_back(sum);//제 0 파라미터 저장
		for (int i = 1; i < m_Points.size(); i++)//제 1 ~ 제 n-1 파라미터에 대하여(단, n = m_Points.size() )
		{
			sum += dist(m_Points[i], m_Points[i - 1]);//제 i와 제 i-1 파라미터의 차를 누적
			params.push_back(sum);//제 i 파라미터를 저장
		}

		// 2. 접선 벡터를 결정한다.
		std::vector<GVec3> tangents;//접선 벡터들을 저장할 벡터
		tangents.push_back(GVec3(0, 0, 0));//제 0 접선 벡터(0 벡터) 저장
		GVec3 tangent;//접선 벡터 계산 결과 저장 임시 변수
		for (int i = 1; i < m_Points.size() - 1; i++)//제 1 ~ 제 n-2 접선 벡터에 대하여(단, n = m_Points.size() )
		{
			tangent = (m_Points[i + 1] - m_Points[i - 1]) / (params[i + 1] - params[i - 1]);//제 i 접선 벡터 계산
			tangents.push_back(tangent);//제 i 접선 벡터 저장
		}
		tangents.push_back(GVec3(0, 0, 0));//제 n-1 접선 벡터(0 벡터) 저장

		// 3. 각 구간의 베지에 곡선을 생성하여 리스트에 추가한다.
		for (int i = 0; i < m_Points.size() - 1; i++)//제 0 ~ 제 n-2 베지에 곡선에 대하여(단, n = m_Points.size() )
		{
			std::vector<GVec3> control_points;//제 i 베지에 곡선의 제어점들 저장 벡터
			control_points.push_back(m_Points[i]);//제 0 제어점 저장
			control_points.push_back(m_Points[i] + (params[i + 1] - params[i]) / 3 * tangents[i]);//제 1 제어점 저장
			control_points.push_back(m_Points[i+1] - (params[i + 1] - params[i]) / 3 * tangents[i+1]);//제 2 제어점 저장
			control_points.push_back(m_Points[i + 1]);//제 3 제어점 저장
			m_Crvs.push_back(GBzrCrv(control_points));//제 i 베지에 곡선 저장
		}
	}
}

void GHermiteCrv::Render(int NumOfPts)
{
	if (m_Points.empty())
		return;	

	// 보간점을 렌더링 한다.
	glColor3d(1.0, 0.0, 0.0);
	glPointSize(5.0f);
	glBegin(GL_POINTS);
	{
		for (int i = 0; i < m_Points.size(); ++i)
			glVertex3d(m_Points[i][0], m_Points[i][1], m_Points[i][2]);
	}
	glEnd();

	// 보간 곡선을 렌더링 한다.
	glColor3d(0.0, 0.0, 1.0);
	glLineWidth(3.0);
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < m_Crvs.size(); ++i)
	{
		for (int j = 0; j < NumOfPts; ++j)
		{
			double t = (double)j / (double)(NumOfPts - 1);
			GVec3 p, v;
			m_Crvs[i].Eval(t, p, v);
			glVertex3dv(p.V);
		}
	}
	glEnd();	
	glLineWidth(1.0);
}
