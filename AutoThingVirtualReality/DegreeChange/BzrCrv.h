#pragma once
#include <vector>
#include "gmath.h"
#include ".\usr\include\GL\freeglut.h"

/*!
*	\brief ������ ��� ǥ���ϴ� Ŭ����
*/
class GBzrCrv
{
public:
	/*! \brief ������ �迭 (��� ���� = ������ �� - 1) */
	std::vector<GVec3>	m_Points;

public:
	// ������ �� �Ҹ���
	GBzrCrv() {};
	GBzrCrv(std::vector<GVec3> Points) { m_Points = Points; }
	~GBzrCrv() {};

public:
	// ��� �Լ�
	void AddCtrlPt(GVec3 pt);
	void Eval(double t, GVec3 &p, GVec3 &v);
	void Render(int NumOfPts);
	void DegIncrease();
	void DegDecrease();
};

