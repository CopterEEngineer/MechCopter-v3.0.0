#ifndef Coordinate_h
#define Coordinate_h

//#define myTYPE float //double
#define EPSLON 0.000001


#include "MatrixTemplate.h"


class Coordinate {

public:
	Matrix1<myTYPE> origin;
	Matrix2<myTYPE> euler;

	Coordinate() {
		origin.allocate(3);
		euler.allocate(3, 3);
	}


	Coordinate(Matrix1<myTYPE> &A, Matrix2<myTYPE> &B) {
		origin = A;
		euler = B;
	}


	Coordinate(const Coordinate &A) {
		cout << "Copy constructor." << endl;
		origin = A.origin;
		euler = A.euler;
	}


	~Coordinate() {
		origin.deallocate();
		euler.deallocate();
	}


	inline void setcoordinate(const Matrix1<myTYPE> &A, const Matrix2<myTYPE> &B) {
		origin = A;
		euler = B;
	}


	inline Matrix2<myTYPE> transfer(const Coordinate &A) { 
		Matrix2<myTYPE> temp;
		return temp; }


	inline Matrix1<myTYPE> translate(const Coordinate &A) { 
		Matrix1<myTYPE> temp;
		return temp;
	}


	inline Coordinate update(void) { 
		Coordinate temp;
		return temp; }

};


// member functions definition



#endif // !Coordinate_h
