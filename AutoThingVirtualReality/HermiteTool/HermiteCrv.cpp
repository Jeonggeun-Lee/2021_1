#include "HermiteCrv.h"

void GHermiteCrv::AddInterpPt(GVec3 pt)
{
	// ������ � ����Ʈ�� �����Ѵ�.
	m_Crvs.clear();

	// �������� �߰��ϰ�
	m_Points.push_back(pt);
	
	// ���ο� ��� ���� ��� �����Ͽ� � ����Ʈ�� �߰��Ѵ�.
	if (m_Points.size() > 1)
	{
		// 1. ���� �Ķ���͸� �����Ѵ�.
		std::vector<float> params;//�Ķ���͵��� ������ ����
		float sum = 0;//�� 0 �Ķ���� ��
		params.push_back(sum);//�� 0 �Ķ���� ����
		for (int i = 1; i < m_Points.size(); i++)//�� 1 ~ �� n-1 �Ķ���Ϳ� ���Ͽ�(��, n = m_Points.size() )
		{
			sum += dist(m_Points[i], m_Points[i - 1]);//�� i�� �� i-1 �Ķ������ ���� ����
			params.push_back(sum);//�� i �Ķ���͸� ����
		}

		// 2. ���� ���͸� �����Ѵ�.
		std::vector<GVec3> tangents;//���� ���͵��� ������ ����
		tangents.push_back(GVec3(0, 0, 0));//�� 0 ���� ����(0 ����) ����
		GVec3 tangent;//���� ���� ��� ��� ���� �ӽ� ����
		for (int i = 1; i < m_Points.size() - 1; i++)//�� 1 ~ �� n-2 ���� ���Ϳ� ���Ͽ�(��, n = m_Points.size() )
		{
			tangent = (m_Points[i + 1] - m_Points[i - 1]) / (params[i + 1] - params[i - 1]);//�� i ���� ���� ���
			tangents.push_back(tangent);//�� i ���� ���� ����
		}
		tangents.push_back(GVec3(0, 0, 0));//�� n-1 ���� ����(0 ����) ����

		// 3. �� ������ ������ ��� �����Ͽ� ����Ʈ�� �߰��Ѵ�.
		for (int i = 0; i < m_Points.size() - 1; i++)//�� 0 ~ �� n-2 ������ ��� ���Ͽ�(��, n = m_Points.size() )
		{
			std::vector<GVec3> control_points;//�� i ������ ��� �������� ���� ����
			control_points.push_back(m_Points[i]);//�� 0 ������ ����
			control_points.push_back(m_Points[i] + (params[i + 1] - params[i]) / 3 * tangents[i]);//�� 1 ������ ����
			control_points.push_back(m_Points[i+1] - (params[i + 1] - params[i]) / 3 * tangents[i+1]);//�� 2 ������ ����
			control_points.push_back(m_Points[i + 1]);//�� 3 ������ ����
			m_Crvs.push_back(GBzrCrv(control_points));//�� i ������ � ����
		}
	}
}

void GHermiteCrv::Render(int NumOfPts)
{
	if (m_Points.empty())
		return;	

	// �������� ������ �Ѵ�.
	glColor3d(1.0, 0.0, 0.0);
	glPointSize(5.0f);
	glBegin(GL_POINTS);
	{
		for (int i = 0; i < m_Points.size(); ++i)
			glVertex3d(m_Points[i][0], m_Points[i][1], m_Points[i][2]);
	}
	glEnd();

	// ���� ��� ������ �Ѵ�.
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
