#pragma once
#include <vector>
#include "gmath.h"
#include ".\usr\include\GL\freeglut.h"
#include "BzrCrv.h"

class GHermiteCrv
{
public:
	/*! \brief 보간점 배열 */
	std::vector<GVec3> m_Points;

	/*! \brief 보간점을 지나는 베지에 곡선의 리스트 */
	std::vector<GBzrCrv> m_Crvs;	

public:
	// 생성자 및 소멸자
	GHermiteCrv() {};
	~GHermiteCrv() {};

	// 멤버 함수
	void AddInterpPt(GVec3 pt);
	void Render(int NumOfPts);
};
