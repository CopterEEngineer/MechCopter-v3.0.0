// MechCopter-v3.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "MatrixTemplate.h"
#include "Component.h"
#include <time.h>


#include "UnitTest.h"


const int isize = 2000;
const int jsize = 2000;
const int COUNT = 70000;


int main()
{

	//Rotor mrotor("main");
	Matrix1<myTYPE> X(jsize), Y(jsize), Z(jsize);
	//myTYPE *aa, *bb, *cc;
	clock_t tStart;
	int icount = 0;
	myTYPE XX[jsize], YY[jsize], ZZ[jsize], maxvalue;
	myTYPE *p, *q;
	myTYPE a, b;


	test_random(X);
	_sleep(100);
	test_random(Y);
	ttest(XX, YY);
	test_vectormultip(X, Y);

	//test_for(100000);


	printf("\n");
	printf("Completed.\n");
	system("pause");
	return 0;
}




