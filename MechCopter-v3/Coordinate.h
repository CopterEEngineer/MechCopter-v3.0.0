#ifndef Coordinate_h
#define Coordinate_h

//#define myTYPE float //double
#define EPSLON 0.000001


#include "MatrixTemplate.h"


class Coordinate {

public:
	myTYPE origin[3], euler[3];
	myTYPE Ttransf[3][3], Etransf[3][3];

	Coordinate();

	Coordinate(const double *x, const double *y);

	Coordinate(const float *x, const float *y);

	Coordinate(const Coordinate &A);

	~Coordinate();

	void SetCoordinate(const myTYPE *og, const myTYPE *el){ Coordinate(og, el); }
	
	inline void Transfer(double t12[3][3], double r12[3], const Coordinate &coord1, const Coordinate &coord2);

	inline void Transfer(float t12[3][3], float r12[3], const Coordinate &coord1, const Coordinate &coord2);

};



#endif // !Coordinate_h
