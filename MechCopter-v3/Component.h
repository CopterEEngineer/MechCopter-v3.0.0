#ifndef Component_h
#define Component_h

#include "MatrixTemplate.h"
#include "Coordinate.h"

#define UL496
#define TIP
//#define ROT
//#define NER
#define RIGID
#define QUSICSD

#ifdef UL496
#define CL_I 40
#define CL_J 12
#define CD_I 52
#define CD_J 12
#define CM_I 52
#define CM_J 11
#define NBLADE 2
#endif // UL496

#ifdef NDEBUG // release configuration
#define DISABLE_REVISE_SIZE 1
#endif // NDEBUG

#ifdef _DEBUG // debug configuration
#define DISABLE_REVISE_SIZE 0
#endif // DISABLE_REVISE_SIZE

#define MAX_SIZE 21600


class MRotor {
private:
	//private members
	// input variables
	int nb, nstation, nk, nf, ns, nbn, kwtip, kwrot, naf, nnr;
	myTYPE eflap, khub, del, pitchroot, radius, bt, rroot;
	myTYPE dfitip, dfirot, dfiner;
	myTYPE sl, bl, wl, phi, thetai, psi;
	myTYPE precone, omega;
	myTYPE sigma, gama, a0;	
	Matrix2<myTYPE> rastation, ristation, azstation, chord, twist, sweep;
	Matrix1<myTYPE> beta;
	myTYPE iflap, m1, rtip, rc0, outboard;
	myTYPE mul;

	Matrix2<myTYPE> cltc, cdtc, cmtc;

	// output variables
	myTYPE lambag, power;
	Matrix2<myTYPE> bflap, dbflap, sfth;
	Matrix2<myTYPE> ut, un, up, ua, ma_n;
	Matrix2<myTYPE> incidn, cl, cd, cirlb;
	Matrix2<myTYPE> lambdi, lambdh, lambdt, lambdx, lambdy;
	Matrix2<myTYPE> tipstr, rotstr, shdstr, trlstr;
	Matrix1<myTYPE> veltpp;
	Matrix3<myTYPE> bladedeform, tipgeometry;

	// member functions	
	void VortexBonStr(void);
	void VortexTipStr(void);
	void VortexRotStr(void);
	void VortexNerStr(void) { ; }
	void AeroDynaCoef(void);

	void BladeDeformation(void) { ; }

	void PredWakeGeometry(void);
	void FreeWakeGeometry(void) { ; }
	void InducedVelCompte(void);
	void InducedVelCompte_v1(void);
	void InducedVelCompte_v2(void);


protected:
	//protected members

public:
	myTYPE mug, vtipa, vsound, rho;
	Matrix1<myTYPE> velb;
	Matrix1<myTYPE> sita, force_rf, force_cg, moment_rf, moment_cg;
	Coordinate bodycoord, hubfxcoord, hubrtcoord, bladecoord, tppcoord;

	MRotor() {
		cout << "Construtor." << endl;
#ifdef UL496
		// input variables
		nf = 72;
		ns = 41;
		if (DISABLE_REVISE_SIZE) {
			if (nf*ns > MAX_SIZE) {
				printf("nf %d and ns %d make beyond of %d \n", nf, ns, MAX_SIZE);
				exit(EXIT_FAILURE);
			}
		}

		nb = 2;
		radius = 11.5;
		eflap = 0;
		kwtip = 5;
		nk = kwtip * nf;
		outboard = 0.3;
		rc0 = 0.004852173913043;
		sweep.allocate(nf, ns);
		twist.allocate(nf, ns);
		chord.allocate(nf, ns);
		twist.input("twist.txt");
		chord.input("chord.txt");

		cltc.allocate(CL_I, CL_J); 
		cdtc.allocate(CD_I, CD_J);
		cltc.input("vr7_cl_c81.txt");
		cdtc.input("vr7_cd_c81.txt");

		azstation.allocate(nf, ns);
		rastation.allocate(nf, ns);
		azstation.input("azstation.txt");
		rastation.input("rastation.txt");

		beta.allocate(3);
		sita.allocate(3);
		beta(0) = 0.052359877559830;
		beta(1) = 0.027308777635547;
		beta(2) = 0.010192964224605;
		sita(0) = 0.421327527798454;
		sita(1) = 0.010003154091981;
		sita(2) = -0.100954246359817;

		omega = 54.977871437821380;
		vtipa = omega * radius;
		vsound = 1115.48;
		mul = 0.176465041274557;
		lambdh.allocate(nf, ns);
		lambdh.input("lambdh.txt");

		tipgeometry.allocate(nk, nf, 3);
		tipgeometry.input("tipgeometry.txt");

		bladedeform.allocate(nf, ns, 3);
		bladedeform.input("bladedeform.txt");

		// output variables
		bflap.allocate(nf, ns);
		dbflap.allocate(nf, ns);
		sfth.allocate(nf, ns);
		ut.allocate(nf, ns);
		un.allocate(nf, ns);
		up.allocate(nf, ns);
		ua.allocate(nf, ns);
		ma_n.allocate(nf, ns);
		incidn.allocate(nf, ns);
		cl.allocate(nf, ns);
		cd.allocate(nf, ns);
		cirlb.allocate(nf, ns);
		tipstr.allocate(nk, nf);
		lambdi.allocate(nf, ns);
		lambdx.allocate(nf, ns);
		lambdy.allocate(nf, ns);
		

#endif // UL496
		
		

	}


	MRotor(const MRotor &A) {
		cout << "Copy constructor." << endl;
	}


	~MRotor() {
		cout << "Destructor." << endl;
		sweep.deallocate();
		twist.deallocate();
		chord.deallocate();
		cltc.deallocate();
		cdtc.deallocate();
		beta.deallocate();
		sita.deallocate();
		azstation.deallocate();
		rastation.deallocate();
		lambdh.deallocate();
		tipgeometry.deallocate();
		bladedeform.deallocate();

		bflap.deallocate();
		dbflap.deallocate();
		sfth.deallocate();
		ut.deallocate();
		un.deallocate();
		up.deallocate();
		ua.deallocate();
		ma_n.deallocate();
		incidn.deallocate();
		cl.deallocate();
		cd.deallocate();
		cirlb.deallocate();
		tipstr.deallocate();
		lambdi.deallocate();
		lambdx.deallocate();
		lambdy.deallocate();


		nf = 0;
		ns = 0;
		radius = 0;
		eflap = 0;
		omega = 0;
		vsound = 0;
		mul = 0;
	}


	void InducedVelCalcPreWake(void);


	void testMRotor() {
		clock_t tStart;
		cout << ns << ", " << nf << endl;
		//cltc.output("cltc.output", 2);
		//cdtc.output("cdtc.output", 2);

		// input variables of AeroDynaCoef()
		//ma_n.input("ma_n.txt", ma_n);
		//incidn.input("incidn.txt", incidn);
		//ma_n.output("ma_n.output", 4);
		//incidn.output("incidn.output", 4);
		//AeroDynaCoef();
		//cl.output("cl.output", 4);
		//cd.output("cd.output", 4);

		// input variables of VortexBonStr
		//azstation.output("azstation.output", 4);
		//rastation.output("rastation.output", 4);
		//twist.output("twist.output", 4);
		//chord.output("chord.output", 4);
		//lambdh.output("lambdh.output", 4);
		//sweep.output("sweep.output", 2);
		//beta.output("beta.output", 4);
		//sita.output("sita.output", 4);
		//VortexBonStr();
		//cirlb.output("cirlb.output", 4);

		// input variables of VortexTipStr()
		//VortexBonStr();
		//VortexTipStr();
		//tipstr.output("tipstr.output", 4);

		// input variables of InducedVelCompte()
		//tipgeometry.output("tipgeometry.output", 4);
		//Matrix2<myTYPE> tipgeoexpand(3, (kwtip * nf + 1)*nf);
		//Matrix3<myTYPE> tempgeo((kwtip * nf + 1), nf, 3);
		//tipgeoexpand = tipgeometry.reshape(3, 3, (kwtip * nf + 1)*nf);
		//for (int k = 0; k < 3; ++k) {
		//	for (int j = 0; j < (kwtip * nf + 1)*nf; ++j) {
		//		tempgeo(j % (kwtip * nf + 1), j / (kwtip * nf + 1), k) = tipgeoexpand(k, j);
		//	}
		//}
		//tempgeo.output("tempgeo.output", 4);
		//tipgeoexpand.output("tipgeoexpand.output", 4);

		tStart = clock();
		int icount = 0;
		for (int i = 0; i < 10; ++i) {
			InducedVelCalcPreWake();
			++icount;
		}		
		printf("%fs \n", (double)(clock() - tStart) / CLOCKS_PER_SEC); 
		printf("%d\n", icount);
		//bladedeform.output("bladedeform.output", 4);
		lambdi.output("lambdi3.output", 4);
		lambdx.output("lambdx3.output", 4);
		lambdy.output("lambdy3.output", 4);
		//printf("%f\n", lambdi.sum());
		



		



	}

};

#endif