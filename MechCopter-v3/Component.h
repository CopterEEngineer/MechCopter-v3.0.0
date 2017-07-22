#ifndef Component_h
#define Component_h

#include "MatrixTemplate.h"
#include "Algorithm.h"
#include "Coordinate.h"

#define UL496
#define TIP
//#define ROT
//#define NER
#define PREWAKE //FREEWAKE

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


class System
{
	// Abstract class
protected:
	// performance parameters etc.
	myTYPE omega, power, torque;

public:
	System() {
		cout << "Construtor." << endl;
		omega = 0;
		power = 0;
		torque = 0;
	}

	System(const System &S) {
		cout << "Copy constructor." << endl;
		omega = S.omega;
		power = S.power;
		torque = S.torque;
	}

	~System() {
		cout << "Destructor." << endl;
		omega = 0;
		power = 0;
		torque = 0;
	}

	virtual myTYPE SetOmega() = 0;
	virtual myTYPE SetPower() = 0;
	virtual myTYPE SetTorque() = 0;
	myTYPE GetOmemga() { return omega; }
	myTYPE GetPower() { return power; }
	myTYPE GetTorque() { return torque; }
};


class Copter :private System
{
public:
	myTYPE rho, vsound;
	// state variables at gc reference coordinate
	myTYPE vel[3], omg[3], dvel[3], domg[3];
	Coordinate refcoord; // self reference coordinate at ground reference coordinate

	Copter();

	Copter(const Copter &H);

	~Copter();

	void Assemble(void);

	void TrimSolver(void);

	void TransientSolver(void);

private:
	// member functions
	myTYPE SetOmega();
	myTYPE SetPower();
	myTYPE SetTorque();

};


class Component :public Copter
{
	// Abstract class
private:

protected:
	myTYPE airforce[3], airmoment[3];

public:
	
	Component();

	Component(const Component &C);

	~Component();

	virtual void SetAirfm() = 0;

	void SetStates(const myTYPE *v, const myTYPE *w, const myTYPE *dv, const myTYPE *dw, Coordinate base) {
		myTYPE tbc[3][3], tbo[3][3];
		myTYPE rbc[3];
		int ipiv[3];

		refcoord.Transfer(tbc, rbc, base, refcoord);
		for (int i = 2; i >= 0; --i) {
			vel[i] = v[i] + w[i] * rbc[i];
			dvel[i] = 0; // 加速度怎么转换
			domg[i] = 0;
		}
#ifdef USE_DOUBLE
		cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1, *tbc, 3, vel, 1, 0, vel, 1);
#else
		cblas_sgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1, *tbc, 3, vel, 1, 0, vel, 1);

#endif // USE_DOUBLE

		//cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, 3, 1, *) // 角速度的转换关系是？
		//加速度怎么转换

	}


	void GetAirfm(myTYPE f[3], myTYPE m[3]) {
		for (int i = 2; i >= 0; --i) {
			f[i] = airforce[i];
			m[i] = airmoment[i];
		}
	}


	void GetStates(myTYPE v[3], myTYPE w[3], myTYPE dv[3], myTYPE dw[3]) {
		for (int i = 2; i >= 0; --i) {
			v[i] = vel[i];
			w[i] = omg[i];
			dv[i] = dvel[i];
			dw[i] = domg[i];
		}
	}

};


class Fuselage :public Component
{
private:
	myTYPE dragA;
	
public:
	Fuselage();

	Fuselage(const Fuselage &F);

	~Fuselage();

	// member functions
	inline void SetAirfm(void);
	
};


class Wing :public Component
{
private:
#ifdef UL496
	char *type;
	myTYPE a0, cd0, cd1, cd2;
	myTYPE span, chord, taper;
	//Coordinate refcoord[2]; //overide refcoord member derived from Copter 
#endif // UL496


public:
	Wing();

	Wing(const char *s, int num);

	Wing(const Wing &W);

	~Wing();

	// member functions
	inline void SetAirfm(void);
};


class Rotor:public Copter
{
private:
	int kwtip, kwrot, nk, nf, ns, nbn, naf, nnr;
	myTYPE eflap, khub, del, pitchroot, radius, bt, rroot;
	//myTYPE sl, bl, wl, phi, thetai, psi;
	myTYPE precone, omega;
	myTYPE sigma, gama, a0;
	Matrix2<myTYPE> rastation, ristation, azstation, chord, twist, sweep;
	myTYPE iflap, m1, rtip, rc0, outboard;
	//myTYPE dfitip, dfirot, dfiner;
	myTYPE mul, vtipa;

	Matrix2<myTYPE> cltc, cdtc, cmtc; //
	myTYPE lambag, power, torque;
	Matrix2<myTYPE> bflap, dbflap, sfth;
	Matrix2<myTYPE> ut, un, up, ua, ma_n;
	Matrix2<myTYPE> incidn, cl, cd, cirlb;
	Matrix2<myTYPE> lambdi, lambdh, lambdt, lambdx, lambdy;
	Matrix2<myTYPE> tipstr, rotstr, shdstr, trlstr;
	//Matrix1<myTYPE> veltpp;
	Matrix3<myTYPE> bladedeform, tipgeometry;
	myTYPE beta[3];
	//myTYPE origin[3], euler[3][3]; 
	Coordinate hubfxcoord, hubrtcoord, bladecoord, tppcoord;


	// member functions	
	inline void SetAirfm(void);

	void AvrgInducedVel(void);

	void VortexBonStr(void);

	void VortexTipStr(void);

	void VortexRotStr(void);

	void VortexNerStr(void) { ; }

	void AeroDynaCoef(void);

	void BladeDeformation(void) { ; }

	void PredWakeGeometry(void);

	void FreeWakeGeometry(void) { ; }

	void InducedVelCalc(void);

	void InducedVelCalc_Prewake_v0(void);

	void InducedVelCalc_Prewake_v1(void);

	void InducedVelCalc_Prewake_v2(void);

	void InducedVelCalc_Prewake_v3(void);

	void AirFMCompt(void);

public:
	myTYPE sita[3];

	Rotor(const char *type);

	Rotor(const Rotor &A);

	~Rotor();

	inline myTYPE GePower() { return power; }
	inline myTYPE GetTorque() { return torque; }
	inline void testRotor(int count) {
		clock_t tStart;
		tStart = clock();
		int icount = 0;
		for (int i = 0; i < count; ++i) {
//			InducedVelCalcPreWake();
			++icount;
		}
		printf("%fs \n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
		printf("%d\n", icount);
		/*lambdi.output("lambdi3.output", 4);
		lambdx.output("lambdx3.output", 4);
		lambdy.output("lambdy3.output", 4);*/
	}

};


#endif