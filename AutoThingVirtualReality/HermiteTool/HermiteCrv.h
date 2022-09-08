#pragma once
#include <vector>
#include "gmath.h"
#include ".\usr\include\GL\freeglut.h"
#include "BzrCrv.h"

class GHermiteCrv
{
public:
	/*! \brief ������ �迭 */
	std::vector<GVec3> m_Points;

	/*! \brief �������� ������ ������ ��� ����Ʈ */
	std::vector<GBzrCrv> m_Crvs;	

public:
	// ������ �� �Ҹ���
	GHermiteCrv() {};
	~GHermiteCrv() {};

	// ��� �Լ�
	void AddInterpPt(GVec3 pt);
	void Render(int NumOfPts);
};
