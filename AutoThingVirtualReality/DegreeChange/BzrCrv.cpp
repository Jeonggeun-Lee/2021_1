//학번: 2017113547
//이름: 이정근
//날짜: 2021-04-03

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
	int numPoints = m_Points.size();//원래 곡선의 제어점의 개수
	int n = numPoints - 1;//원래 제어점의 마지막 첨수
	std::vector<GVec3> newPoints;//새로운 제어점 벡터
	newPoints.push_back(m_Points[0]);//새로운 제 0 제어점 = 원래 제 0 제어점
	for (int i = 1; i <= n; i++) {
		//새로운 제 i 제어점 = i / (n + 1) * 원래 제 i-1 제어점 + (1 - i/(n + 1))*원래 제 i 제어점
		newPoints.push_back( GVec3(((double)i) / (n + 1) * m_Points[i - 1] + (1 - ((double)i) / (n + 1))*m_Points[i]) );
	}
	newPoints.push_back(m_Points[n]);//새로운 제 n+1 제어점 = 원래 제 n 제어점
	m_Points = newPoints;//원래 제어점 벡터를 새로운 제어점 벡터로 치환
}

/*
*	\brief 행렬을 출력
*/

void PrintMat(std::vector<std::vector<float>> mat) {
	for (int i = 0; i < mat.size(); i++) {
		for (int j = 0; j < mat[0].size(); j++) {
			printf("%5.2f ", mat[i][j]);
		}
		std::cout << std::endl;
	}
}

/*
\ breif 포인트들을 출력
*/
void PrintPoints(std::vector<GVec3> points) {
	std::cout << "[" << std::endl;
	for (int i = 0; i < points.size(); i++) {
		std::cout << "( " << points[i][0] << ", " << points[i][1] << ", " << points[i][2] << " )," << std::endl;
	}
	std::cout << "]" << std::endl;
}


/*!
*	\brief n차 베지에 곡선의 제어점에 곱하여 n+1차 베지에 곡선의 제어점이 나오게 하는 행렬 생성
*	\param	n[in]	곡선의 차수
*	\param	result[out]	결과행렬 n+2 by n+1
*/
void MakeMat(int n, std::vector< std::vector<float>>& result) {
	std::vector<float> row0;//제 1 행
	for (int i = 0; i <= n; i++) {
		row0.push_back(0);//제 1 행의 모든 원소를 0으로 초기화
	}
	row0[0] = 1;//1행 1열의 원소 1
	result.push_back(row0);//제 1 행을 행렬에 추가

	//제 2 행부터 제 n+1 행
	for (int i = 1; i <= n; i++) {
		std::vector<float> row;
		for (int j = 0; j <= n; j++) {
			row.push_back(0);
		}
		row[i - 1] = i / (float)(n + 1);//제 i 열 원소 = i/(n+1)
		row[i] = (n - (i - 1)) / (float)(n + 1);//제 i+1 열 원소 = (n-(i-1))/(n+1)
		result.push_back(row);
	}

	//제 n+1 행
	std::vector<float> rowLast;
	for (int i = 0; i <= n; i++) {
		rowLast.push_back(0);
	}
	rowLast[n] = 1;//제 n+1 열 원소 = 1
	result.push_back(rowLast);
}


/*!
*	\brief 전치행렬을 구한다.
*	\param	m[in]	행수
*	\param	n[in]	열수
*	\param	mat[in]	대상행렬
*	\param	result[out]	결과행렬 n by m
*/
void Transpose(int m, int n,
	std::vector<std::vector<float>> mat,
	std::vector<std::vector<float>>& result) {
	for (int i = 0; i < n; i++) {
		std::vector<float> row;
		for (int j = 0; j < m; j++) {
			row.push_back(mat[j][i]);
		}
		result.push_back(row);
	}
}

/*!
*	\brief	두 행렬의 곱을 구한다.
*	\param	ㅣ[in]	첫 행렬의 행수
*	\param	m[in]	첫 행렬의 열수 = 둘째 행렬의 행수
*	\param	n[in]	둘째 행렬의 열수
*	\param	a[in]	첫 행렬
*	\param	b[in]	둘째 행렬
*	\param	result[out]	결과행렬 ㅣ by n
*/
void MatMultiply(int l, int m, int n,
	std::vector<std::vector<float>> a,
	std::vector<std::vector<float>> b,
	std::vector<std::vector<float>>& result) {
	
	for (int i = 0; i < l; i++) {
		std::vector<float> row;
		for (int j = 0; j < n; j++) {
			row.push_back(0);
		}
		result.push_back(row);
	}
	for (int i = 0; i < l; i++) {
		for (int j = 0; j < n; j++) {
			for (int k = 0; k < m; k++) {
				result[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}

/*!
*	\brief	역행렬을 구한다.
*	\param	n[in]	대상 행렬의 행렬수
*	\param	mat[in]	대상 행렬
*	\param	result[out]	결과행렬 n by n
*/
void Inverse(int n,
	std::vector<std::vector<float>> mat,
	std::vector<std::vector<float>>& result) {
	
	//결과 행렬의 모든 원소를 0으로 초기화
	for (int i = 0; i < n; i++) {
		std::vector<float> row;
		for (int j = 0; j < n; j++) {
			row.push_back(0);
		}
		row[i] = 1;
		result.push_back(row);
	}

	//결과 행렬의 대각성분을 1로 만듬
	//결과 행렬은 단위행렬이 됨
	for (int i = 0; i < n; i++) {
		result[i][i] = 1;
	}

	//가우스-조단 소거법으로 역행렬을 구한다.
	float s, t;
	for (int i = 0; i < n; i++) {
		s = mat[i][i];//대상행렬의 (i,i)성분 값
		for (int j = 0; j < n; j++) {
			mat[i][j] = mat[i][j] / s; //대상 행렬의 i행 모든 원소를 s로 나눈다.
			result[i][j] = result[i][j] / s;//결과 행렬의 i행 모든 원소를 s로 나눈다.
		}
		
		//대상 행렬의 i열에 대하여 i행 성분만 남기고 나머지는 0으로 소거한다.
		for (int j = 0; j < n; j++) {
			if (j != i) {
				t = mat[j][i];//대상 행렬의 (j,i) 성분 값
				std::vector<float> scaledRow;
				std::vector<float> scaledRowResult;
				//scaledRow를 mat의 i행으로 만든다
				//scaledRowResult를 result의 i행으로 만든다
				for (int k = 0; k < n; k++) {
					scaledRow.push_back(mat[i][k]);
					scaledRowResult.push_back(result[i][k]);
				}
				//scaledRow를 t배 해서 mat의 j행에서 뺀다
				//scaledRowResult를 t배 해서 result의 j행에서 뺀다
				for (int k = 0; k < n; k++) {
					scaledRow[k] = t * scaledRow[k];
					scaledRowResult[k] = t * scaledRowResult[k];
					mat[j][k] -= scaledRow[k];
					result[j][k] -= scaledRowResult[k];
				}
			}
		}
	}
}
/*!
*	\brief	행렬과 점들의 벡터를 곱한다.
*	\param	m[in]	행렬의 행수
*	\param	n[in]	행렬의 열수 = 점들의 갯수
*	\param	mat[in]	행렬
*	\param	points[in]	점들의 벡터
*	\param	result[out]	결과 점들의 벡터
*/
void MatPointsMultiply(int m, int n, 
	std::vector<std::vector<float>> mat,
	std::vector<GVec3> points,
	std::vector<GVec3>& result) {

	for (int i = 0; i < m; i++) {
		result.push_back(GVec3(0, 0, 0));
	}

	for (int i = 0; i < m; i++) {
		GVec3 sum(0, 0, 0);
		for (int j = 0; j < n; j++) {
			sum += mat[i][j] * points[j];
		}
		result[i] = sum;
	}
}

/*!
*	\brief	곡선의 차수를 낮춘다.
*/
void GBzrCrv::DegDecrease()
{
	// 구현하세요.
	if (m_Points.size() == 1) return;//제어점이 하나뿐일 때는 아무것도 하지 않고 바로 반환

	//원래 제어점을 출력
	std::cout << "Original Points = " << std::endl;
	PrintPoints(m_Points);
	
	int numPoints = m_Points.size();//원래 곡선의 제어점의 개수
	int n = numPoints - 2;//새로운 제어점의 마지막 첨수
	
	//원래 제어점과 낮은 차수의 제어점의 관계를 나타내는 행렬 D를 만듬
	//D*낮은 차수의 제어점들의 벡터 = 원래 차수의 제어점들의 벡터
	std::vector<std::vector<float>> D;
	MakeMat(n, D);
	
	//D의 전치행렬 DT를 구함
	std::vector<std::vector<float>> DT;
	int numRow, numCol;
	numRow = D.size();
	numCol = D[0].size();
	Transpose(numRow, numCol, D, DT);

	//DT와 D의 곱 DTD를 구함
	std::vector<std::vector<float>> DTD;
	MatMultiply(numCol, numRow, numCol, DT, D, DTD);

	//DTD의 역행렬 DTDInv를 구함
	std::vector<std::vector<float>> DTDInv;
	Inverse(numCol, DTD, DTDInv);

	//DTDInv와 DT의 곱 DTDInvDT를 구함
	std::vector<std::vector<float>> DTDInvDT;
	MatMultiply(numCol, numCol, numRow, DTDInv, DT, DTDInvDT);

	//DTDInvDT와 원래 제어점들 m_Points를 곱하여 낮은 차수의 제어점들 newPoints를 구함
	std::vector<GVec3> newPoints;
	MatPointsMultiply(numCol, numPoints, DTDInvDT, m_Points, newPoints);
	
	//원래 제어점들을 낮은 차수의 제어점들로 치환
	m_Points = newPoints;

	// D, DT, DTD, DTDInv, DTD*DTDInv, DTDInvDT, 낮은 차수의 제어점들을 출력해 봄
	std::cout << "D = " << std::endl;
	PrintMat(D);
	std::cout << "DT = " << std::endl;
	PrintMat(DT);
	std::cout << "DTD = " << std::endl;
	PrintMat(DTD);
	std::cout << "DTDInv = " << std::endl;
	PrintMat(DTDInv);
	std::cout << "DTD * DTDInv = " << std::endl;
	std::vector<std::vector<float>> identity;
	MatMultiply(numCol, numCol, numCol, DTD, DTDInv, identity);
	PrintMat(identity);

	std::cout << "DTDInvDT = " << std::endl;
	PrintMat(DTDInvDT);

	std::cout << "New Points = " << std::endl;
	PrintPoints(newPoints);
	
}