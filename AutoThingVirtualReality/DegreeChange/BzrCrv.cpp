//�й�: 2017113547
//�̸�: ������
//��¥: 2021-04-03

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
	int numPoints = m_Points.size();//���� ��� �������� ����
	int n = numPoints - 1;//���� �������� ������ ÷��
	std::vector<GVec3> newPoints;//���ο� ������ ����
	newPoints.push_back(m_Points[0]);//���ο� �� 0 ������ = ���� �� 0 ������
	for (int i = 1; i <= n; i++) {
		//���ο� �� i ������ = i / (n + 1) * ���� �� i-1 ������ + (1 - i/(n + 1))*���� �� i ������
		newPoints.push_back( GVec3(((double)i) / (n + 1) * m_Points[i - 1] + (1 - ((double)i) / (n + 1))*m_Points[i]) );
	}
	newPoints.push_back(m_Points[n]);//���ο� �� n+1 ������ = ���� �� n ������
	m_Points = newPoints;//���� ������ ���͸� ���ο� ������ ���ͷ� ġȯ
}

/*
*	\brief ����� ���
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
\ breif ����Ʈ���� ���
*/
void PrintPoints(std::vector<GVec3> points) {
	std::cout << "[" << std::endl;
	for (int i = 0; i < points.size(); i++) {
		std::cout << "( " << points[i][0] << ", " << points[i][1] << ", " << points[i][2] << " )," << std::endl;
	}
	std::cout << "]" << std::endl;
}


/*!
*	\brief n�� ������ ��� �������� ���Ͽ� n+1�� ������ ��� �������� ������ �ϴ� ��� ����
*	\param	n[in]	��� ����
*	\param	result[out]	������ n+2 by n+1
*/
void MakeMat(int n, std::vector< std::vector<float>>& result) {
	std::vector<float> row0;//�� 1 ��
	for (int i = 0; i <= n; i++) {
		row0.push_back(0);//�� 1 ���� ��� ���Ҹ� 0���� �ʱ�ȭ
	}
	row0[0] = 1;//1�� 1���� ���� 1
	result.push_back(row0);//�� 1 ���� ��Ŀ� �߰�

	//�� 2 ����� �� n+1 ��
	for (int i = 1; i <= n; i++) {
		std::vector<float> row;
		for (int j = 0; j <= n; j++) {
			row.push_back(0);
		}
		row[i - 1] = i / (float)(n + 1);//�� i �� ���� = i/(n+1)
		row[i] = (n - (i - 1)) / (float)(n + 1);//�� i+1 �� ���� = (n-(i-1))/(n+1)
		result.push_back(row);
	}

	//�� n+1 ��
	std::vector<float> rowLast;
	for (int i = 0; i <= n; i++) {
		rowLast.push_back(0);
	}
	rowLast[n] = 1;//�� n+1 �� ���� = 1
	result.push_back(rowLast);
}


/*!
*	\brief ��ġ����� ���Ѵ�.
*	\param	m[in]	���
*	\param	n[in]	����
*	\param	mat[in]	������
*	\param	result[out]	������ n by m
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
*	\brief	�� ����� ���� ���Ѵ�.
*	\param	��[in]	ù ����� ���
*	\param	m[in]	ù ����� ���� = ��° ����� ���
*	\param	n[in]	��° ����� ����
*	\param	a[in]	ù ���
*	\param	b[in]	��° ���
*	\param	result[out]	������ �� by n
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
*	\brief	������� ���Ѵ�.
*	\param	n[in]	��� ����� ��ļ�
*	\param	mat[in]	��� ���
*	\param	result[out]	������ n by n
*/
void Inverse(int n,
	std::vector<std::vector<float>> mat,
	std::vector<std::vector<float>>& result) {
	
	//��� ����� ��� ���Ҹ� 0���� �ʱ�ȭ
	for (int i = 0; i < n; i++) {
		std::vector<float> row;
		for (int j = 0; j < n; j++) {
			row.push_back(0);
		}
		row[i] = 1;
		result.push_back(row);
	}

	//��� ����� �밢������ 1�� ����
	//��� ����� ��������� ��
	for (int i = 0; i < n; i++) {
		result[i][i] = 1;
	}

	//���콺-���� �ҰŹ����� ������� ���Ѵ�.
	float s, t;
	for (int i = 0; i < n; i++) {
		s = mat[i][i];//�������� (i,i)���� ��
		for (int j = 0; j < n; j++) {
			mat[i][j] = mat[i][j] / s; //��� ����� i�� ��� ���Ҹ� s�� ������.
			result[i][j] = result[i][j] / s;//��� ����� i�� ��� ���Ҹ� s�� ������.
		}
		
		//��� ����� i���� ���Ͽ� i�� ���и� ����� �������� 0���� �Ұ��Ѵ�.
		for (int j = 0; j < n; j++) {
			if (j != i) {
				t = mat[j][i];//��� ����� (j,i) ���� ��
				std::vector<float> scaledRow;
				std::vector<float> scaledRowResult;
				//scaledRow�� mat�� i������ �����
				//scaledRowResult�� result�� i������ �����
				for (int k = 0; k < n; k++) {
					scaledRow.push_back(mat[i][k]);
					scaledRowResult.push_back(result[i][k]);
				}
				//scaledRow�� t�� �ؼ� mat�� j�࿡�� ����
				//scaledRowResult�� t�� �ؼ� result�� j�࿡�� ����
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
*	\brief	��İ� ������ ���͸� ���Ѵ�.
*	\param	m[in]	����� ���
*	\param	n[in]	����� ���� = ������ ����
*	\param	mat[in]	���
*	\param	points[in]	������ ����
*	\param	result[out]	��� ������ ����
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
*	\brief	��� ������ �����.
*/
void GBzrCrv::DegDecrease()
{
	// �����ϼ���.
	if (m_Points.size() == 1) return;//�������� �ϳ����� ���� �ƹ��͵� ���� �ʰ� �ٷ� ��ȯ

	//���� �������� ���
	std::cout << "Original Points = " << std::endl;
	PrintPoints(m_Points);
	
	int numPoints = m_Points.size();//���� ��� �������� ����
	int n = numPoints - 2;//���ο� �������� ������ ÷��
	
	//���� �������� ���� ������ �������� ���踦 ��Ÿ���� ��� D�� ����
	//D*���� ������ ���������� ���� = ���� ������ ���������� ����
	std::vector<std::vector<float>> D;
	MakeMat(n, D);
	
	//D�� ��ġ��� DT�� ����
	std::vector<std::vector<float>> DT;
	int numRow, numCol;
	numRow = D.size();
	numCol = D[0].size();
	Transpose(numRow, numCol, D, DT);

	//DT�� D�� �� DTD�� ����
	std::vector<std::vector<float>> DTD;
	MatMultiply(numCol, numRow, numCol, DT, D, DTD);

	//DTD�� ����� DTDInv�� ����
	std::vector<std::vector<float>> DTDInv;
	Inverse(numCol, DTD, DTDInv);

	//DTDInv�� DT�� �� DTDInvDT�� ����
	std::vector<std::vector<float>> DTDInvDT;
	MatMultiply(numCol, numCol, numRow, DTDInv, DT, DTDInvDT);

	//DTDInvDT�� ���� �������� m_Points�� ���Ͽ� ���� ������ �������� newPoints�� ����
	std::vector<GVec3> newPoints;
	MatPointsMultiply(numCol, numPoints, DTDInvDT, m_Points, newPoints);
	
	//���� ���������� ���� ������ ��������� ġȯ
	m_Points = newPoints;

	// D, DT, DTD, DTDInv, DTD*DTDInv, DTDInvDT, ���� ������ ���������� ����� ��
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