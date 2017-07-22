#include "stdafx.h"
#include "Coordinate.h"
#include "Algorithm.h"


Coordinate::Coordinate() {
	for (int i = 2; i >= 0; --i) {
		origin[i] = 0;
		euler[i] = 0;
		for (int j = 2; j >= 0; --j) {
			if (i == j) {
				Ttransf[i][j] = 1;
				Etransf[i][j] = 1;
			}
			else {
				Ttransf[i][j] = 0;
				Etransf[i][j] = 0;

			}
		}
	}
}


Coordinate::Coordinate(const double *og, const double *el) {
	double rx[3][3] = { 0 };
	double ry[3][3] = { 0 };
	double rz[3][3] = { 0 };
	for (int i = 2; i >= 0; --i) {
		origin[i] = og[i];
		euler[i] = el[i];
	}
	// roll
	rx[0][0] = 1;
	rx[1][1] = cos(euler[0]);
	rx[2][2] = rx[1][1];
	rx[1][2] = sin(euler[0]);
	rx[2][1] = -sin(euler[0]);
	// pitch
	ry[0][0] = cos(euler[1]);
	ry[1][1] = 1;
	ry[2][2] = ry[0][0];
	ry[0][2] = -sin(euler[1]);
	ry[2][0] = sin(euler[1]);
	// yaw
	rz[0][0] = cos(euler[2]);
	rz[1][1] = rz[0][0];
	rz[2][2] = 1;
	rz[0][1] = sin(euler[2]);
	rz[1][0] = -sin(euler[2]);

	// Ttransf
	double temp[3][3] = { 0 };
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3, 1, *rx, 3, *ry, 3, 0, *temp, 3);
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3, 1, *temp, 3, *rz, 3, 0, *Ttransf, 3);
	// Etransf
	Etransf[0][0] = 1;
	Etransf[0][1] = Etransf[1][0] = Etransf[2][0] = 0;
	Etransf[0][2] = -sin(euler[1]);
	Etransf[1][1] = cos(euler[0]);
	Etransf[1][2] = sin(euler[0])*cos(euler[1]);
	Etransf[2][1] = -sin(euler[0]);
	Etransf[2][2] = cos(euler[0])*cos(euler[1]);


}


Coordinate::Coordinate(const float *og, const float *el) {
	float rx[3][3] = { 0 };
	float ry[3][3] = { 0 };
	float rz[3][3] = { 0 };
	for (int i = 2; i >= 0; --i) {
		origin[i] = og[i];
		euler[i] = el[i];
	}
	// roll
	rx[0][0] = 1;
	rx[1][1] = cos(euler[0]);
	rx[2][2] = rx[1][1];
	rx[1][2] = sin(euler[0]);
	rx[2][1] = -sin(euler[0]);
	// pitch
	ry[0][0] = cos(euler[1]);
	ry[1][1] = 1;
	ry[2][2] = ry[0][0];
	ry[0][2] = -sin(euler[1]);
	ry[2][0] = sin(euler[1]);
	// yaw
	rz[0][0] = cos(euler[2]);
	rz[1][1] = rz[0][0];
	rz[2][2] = 1;
	rz[0][1] = sin(euler[2]);
	rz[1][0] = -sin(euler[2]);

	// Ttransf
	float temp[3][3] = { 0 };
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3, 1, *rx, 3, *ry, 3, 0, *temp, 3);
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3, 1, *temp, 3, *rz, 3, 0, *Ttransf, 3);
	// Etransf
	Etransf[0][0] = 1;
	Etransf[0][1] = Etransf[1][0] = Etransf[2][0] = 0;
	Etransf[0][2] = -sin(euler[1]);
	Etransf[1][1] = cos(euler[0]);
	Etransf[1][2] = sin(euler[0])*cos(euler[1]);
	Etransf[2][1] = -sin(euler[0]);
	Etransf[2][2] = cos(euler[0])*cos(euler[1]);


}


Coordinate::Coordinate(const Coordinate &A) {
	//cout << "Copy constructor." << endl;
	for (int i = 2; i >= 0; --i) {
		origin[i] = A.origin[i];
		euler[i] = A.euler[i];
		for (int j = 2; j >= 0; --j) {
			Ttransf[i][j] = A.Ttransf[i][j];
			Etransf[i][j] = A.Etransf[i][j];
		}
	}
}


Coordinate::~Coordinate() {
	for (int i = 2; i >= 0; --i) {
		origin[i] = 0;
		euler[i] = 0;
		for (int j = 2; j >= 0; --j) {
			if (i == j) {
				Ttransf[i][j] = 1;
				Etransf[i][j] = 1;
			}
			else {
				Ttransf[i][j] = 0;
				Etransf[i][j] = 0;
			}
			
		}
	}
}


inline void Coordinate::Transfer(double t12[3][3], double r12[3], const Coordinate &coord1, const Coordinate &coord2) {
	// return transfer transform from coord1 to coord2
	double *origin1_ptr = coord1.origin;
	double *origin2_ptr = coord2.origin;
	double t1[3][3], t2[3][3];
	double temp[3][3];

	for (int i = 2; i >= 0; --i) {
		r12[i] = *(origin2_ptr + i) - *(origin1_ptr + i);
		for (int j = 2; j >= 0; --j) {
			temp[i][j] = coord1.Ttransf[i][j];

		}

	}
	int ipiv[3] = { 0 };
	Inverse(*temp, ipiv, 3, "col");
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3, 1, *temp, 3, *coord2.Ttransf, 3, 0, *t12, 3);

}


inline void Coordinate::Transfer(float t12[3][3], float r12[3], const Coordinate &coord1, const Coordinate &coord2) {
	// return transfer transform from coord1 to coord2
	float *origin1_ptr = coord1.origin;
	float *origin2_ptr = coord2.origin;
	float t1[3][3], t2[3][3];
	float temp[3][3];

	for (int i = 2; i >= 0; --i) {
		r12[i] = *(origin2_ptr + i) - *(origin1_ptr + i);
		for (int j = 2; j >= 0; --j) {
			temp[i][j] = coord1.Ttransf[i][j];

		}

	}
	int ipiv[3] = { 0 };
	Inverse(*temp, ipiv, 3, "col");
	cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3, 1, *temp, 3, *coord2.Ttransf, 3, 0, *t12, 3);

}