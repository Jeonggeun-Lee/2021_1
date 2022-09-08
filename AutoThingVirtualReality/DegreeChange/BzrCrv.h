#pragma once
#include <vector>
#include "gmath.h"
#include ".\usr\include\GL\freeglut.h"

/*!
*	\brief 베지에 곡선을 표현하는 클래스
*/
class GBzrCrv
{
public:
	/*! \brief 제어점 배열 (곡선의 차수 = 제어점 수 - 1) */
	std::vector<GVec3>	m_Points;

public:
	// 생성자 및 소멸자
	GBzrCrv() {};
	GBzrCrv(std::vector<GVec3> Points) { m_Points = Points; }
	~GBzrCrv() {};

public:
	// 멤버 함수
	void AddCtrlPt(GVec3 pt);
	void Eval(double t, GVec3 &p, GVec3 &v);
	void Render(int NumOfPts);
	void DegIncrease();
	void DegDecrease();
};

