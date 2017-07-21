#include "stdafx.h"
#include "Component.h"



void MRotor::InducedVelCalcPreWake(void) {

	// Bound circulation
	VortexBonStr();

	// tip vortices strength
#ifdef TIP
	VortexTipStr();

#endif // TIP

	// root vortices strength
#ifdef ROT
	VortexRotStr();
#endif // ROT

	// near wake strength
#ifdef NER
	VortexNerStr();
#endif // NER

	// compute induced velocity by prescribed wake	
	//InducedVelCompte();
	//InducedVelCompte_v1();
	InducedVelCompte_v2();


}


void MRotor::AeroDynaCoef(void) {
	Matrix2<myTYPE> incidn_temp(nf, ns);
	
	// transfer to degree unit
	incidn_temp = incidn / PI * 180.0;

#ifdef UL496	
	cl = cltc.interplinear_fast(cltc(step(0, cltc.NI - 1), 0), cltc(0, step(0, cltc.NJ - 1)), incidn_temp, ma_n);
	cd = cdtc.interplinear_fast(cdtc(step(0, cdtc.NI - 1), 0), cdtc(0, step(0, cdtc.NJ - 1)), incidn_temp, ma_n);
#elif UH60A
	;
#endif 


}


void MRotor::VortexBonStr(void) {
	// input variables	
	//Matrix2<myTYPE> bflap(nf, ns), dbflap(nf, ns), sfth(nf, ns);
	// output variables
	//Matrix2<myTYPE> ut(nf, ns), un(nf, ns), up(nf, ns), ua(nf, ns), ma_n(nf, ns);
	//Matrix2<myTYPE> incidn(nf, ns), cl(nf, ns), cd(nf, ns), cirlb(nf, ns);
	// others
	myTYPE df, dr;
	
	df = 2 * PI / nf; // azstation(1, 0) - azstation(0, 0);
	dr = rastation(0, 1) - rastation(0, 0);
	
	//flap motion
#ifdef QUSICSD
#ifdef RIGID
	bflap = mcos(azstation) * beta(1) + msin(azstation) * beta(2) + beta(0);
	dbflap = (msin(azstation) * beta(1) - mcos(azstation) * beta(2)) * (-omega);	
	/*bflap.output("bflap.output", 4);
	dbflap.output("dbflap.output", 4);*/
#endif // RIGID
#endif // RIGID

	// blade pitch
	sfth = mcos(azstation) * sita(1) + msin(azstation) * sita(2) + sita(0);
	//sfth.output();

	// air velocity
	ut = msin(azstation) * mul + (rastation - eflap) * mcos(bflap) + eflap;
	un = ut * mcos(sweep);
	up = msin(bflap) * mcos(azstation) * mul + mcos(bflap) * lambdh + (rastation - eflap) * dbflap / omega;

	un = un * vtipa; 
	up = up * vtipa; 
	ua = msqrt(un * un + up * up);
	ma_n = ua / vsound;
	ma_n = mmax(ma_n, 0.0);
	ma_n = mmin(ma_n, 1.0);

	// attack of angle
	incidn = sfth + twist - atan2(up, un);
	for (int i = 0; i < nf*ns; ++i) {
		if (incidn.v_p[i] > PI) incidn.v_p[i] -= 2.0 * PI;
		else if (incidn.v_p[i] < -PI) incidn.v_p[i] += 2.0 * PI; 
	}

	// air coeficients
	AeroDynaCoef();

	// Bound circulation
	cirlb = ua * cl * chord * 0.5;

}


void MRotor::VortexTipStr(void) {
	// output variables
	//Matrix2<myTYPE> tipstr;
	// others
	int igen = 0, istr = 0;
	Matrix1<int> idns;

	// allocate memory
	istr = outboard * ns;
	//tipstr.allocate(nk, nf);
	idns.allocate(ns-istr);

	// obtian tip vortices strengths	
	idns = step(istr, ns - 1);
	for (int j = nf - 1; j >= 0; --j) {
		for (int i = nk - 1; i >= 0; --i) {
			igen = j - i;
			while (igen < 0) igen += nf;
			tipstr(i, j) = cirlb(igen, idns).findmax();
		}
	}
}


void MRotor::VortexRotStr(void) {
	// output variables

	// others
	int igen = 0;

	// obtain root vortices strengths
	for (int j = nf - 1; j >= 0; --j) {
		for (int i = nk - 1; i >= 0; --i) {
			igen = j - i;
			while (igen < 0) igen += nf;
			rotstr(i, j) = cirlb(igen, 0);
		}
	}
}


void MRotor::PredWakeGeometry(void) {
	
	// output variables
	Matrix3<myTYPE> tipgeometry;
	// input variables and others
	Matrix2<myTYPE> transferMatrixfrombody(3, 3), transferMatrixfromblade(3, 3);
	Matrix1<myTYPE> veltpp;
	myTYPE r0, z0, x0, xv, yv, zv, xe, df;
	myTYPE ka, E, kx, ky, ky3, k0;
	myTYPE m, n;

	// update tppcord based on wind velocity (velb)
	tppcoord.update();

	// transfer air velocity from body coordinate to tpp coordinate
	transferMatrixfrombody = tppcoord.transfer(bodycoord);
	veltpp = transferMatrixfrombody.matrixmultiply(velb); // nondim	

	// shed point at tpp
	r0 = rtip * cos(beta(0));
	z0 = (rtip - radius) * sin(beta(0));

	
	// Beddoes prescribed wake parameters obtained
	m = veltpp(0); n = veltpp(2) + lambag;
	if (((m > 0) && (n > 0)) || ((m > 0) && (n < 0))) { ka = atan(m / n); }
	else if ((m < 0) && (n > 0)) { ka = atan(m / n) + PI; }
	else if ((m < 0) && (n < 0)) { ka = atan(m / n) - PI; }
	else if ((m == 0) && (n > 0)) { ka = 0.0; }
	else if ((m == 0) && (n < 0)) { ka = PI; }
	else if ((m > 0) && (n == 0)) { ka = PI / 2; }
	else if ((m < 0) && (n == 0)) { ka = -PI / 2; }
	else { ka = 0.0; }
	E = ka / 2.0;
	kx = E; ky = -2.0 * veltpp(0); ky3 = -kx; k0 = 1.0 - 8.0 * ky3 / 15.0 / PI;

	// allocate memory
	tipgeometry.allocate(nk, nf, 3);

	// compute tip vortices geometry in TPP coord
	df = 2.0 * PI / nf; xe = 0;
	for (int j = nf - 1; j >= 0; --j) {
		for (int i = nk - 1; i >= 0; --i) {
			x0 = r0 * cos((j - i) * df);
			xv = x0 + veltpp(0) * i * df;
			yv = r0 * sin((j - i) * df);
			if (xv * xv + yv * yv < 1.01) {
				zv = -lambag * (k0 + ky3 * Abs(yv*yv*yv) + ky * yv) * i*df - lambag / 2.0 * kx * (x0 + xv) * i*df;
			}
			else {
				xe = sqrt(1.0 - yv*yv);
				zv = -lambag / veltpp(0) * (k0 + ky3 * Abs(yv*yv*yv) + ky * yv) * (xe - x0) - lambag / 2.0 * kx * (x0 + xe) * i*df;
				zv -= 2.0 * lambag / veltpp(0) * (k0 + ky3 * Abs(yv*yv*yv) + ky * yv) * (xv - xe);
			}
			tipgeometry(i, j, 1) = xv;
			tipgeometry(i, j, 2) = yv;
			tipgeometry(i, j, 3) = zv + z0;
		}
	}

#ifdef ROT
	;
#endif // ROT
#ifdef NER
	;
#endif // NER


	
}


void MRotor::InducedVelCompte(void) {
	// input variables and others
	Matrix3<myTYPE> tipgeoexpand_athub;
	Matrix2<myTYPE> tipgeoexpand;
	Matrix2<myTYPE> transferMatrixfromtpp(3, 3);

	myTYPE xblade, yblade, zblade, ri0, rj0, rk0, ri1, rj1, rk1;
	int ib = 0;
	myTYPE rcros[3], r0len, r1len, rdot, height, geofunc;
	// output varaibles
	Matrix2<myTYPE> temp_lambdi, temp_lambdx, temp_lambdy;
	myTYPE temp_temp_lambdi, temp_temp_lambdx, temp_temp_lambdy;

	// transfer geometry from tpp to hub coordinate
	//transferMatrixfromtpp = hubfxcoord.transfer(tppcoord);
	transferMatrixfromtpp(0, 0) = 1;
	transferMatrixfromtpp(0, 1) = 0;
	transferMatrixfromtpp(0, 2) = -beta(1);
	transferMatrixfromtpp(1, 0) = 0;
	transferMatrixfromtpp(1, 1) = 1;
	transferMatrixfromtpp(1, 2) = -beta(2);
	transferMatrixfromtpp(2, 0) = beta(1);
	transferMatrixfromtpp(2, 1) = beta(2);
	transferMatrixfromtpp(2, 2) = 1;
	//transferMatrixfromtpp.output();
	tipgeoexpand = tipgeometry.reshape(3, 3, nk*nf); // 应该要减去在TPP坐标系下TPP原点指向HUB原点的向量
	//tipgeoexpand = transferMatrixfromtpp.matrixmultiply(tipgeoexpand);
	tipgeoexpand = transferMatrixfromtpp.matrixmultiplyTP(tipgeoexpand);
	//tipgeoexpand_athub = tipgeoexpand.reshape(nk, nf, 3);
	tipgeoexpand_athub.allocate(nk, nf, 3);
	for (int k = 0; k < 3; ++k) {
		for (int j = 0; j < nk*nf; ++j) {
			tipgeoexpand_athub(j % nk, j / nk, k) = tipgeoexpand(k, j);
		}
	}
	// tipgeoexpand_athub.output("tipgeoexpand_athub.output", 4);

	// compute induced velocity
	//temp_lambdi.allocate(nf, ns);
	//temp_lambdx.allocate(nf, ns);
	//temp_lambdy.allocate(nf, ns);
	lambdi.allocate(nf, ns);
	lambdx.allocate(nf, ns);
	lambdy.allocate(nf, ns);
	Matrix2<myTYPE> temp_geofunc_x(nk, ns), temp_geofunc_y(nk, ns), temp_geofunc_z(nk, ns);
	for (int w = nb - 1; w >= 0; --w) {
		for (int k = ns - 1; k >= 0; --k) {
			for (int j = nf - 1; j >= 0; --j) {
				ib = j + w * (nf / nb);
				for (;;) {
					if (ib >= nf) ib -= nf;
					else { break; }
				}
				xblade = bladedeform(ib, k, 0);
				yblade = bladedeform(ib, k, 1);
				zblade = bladedeform(ib, k, 2);
				//temp_temp_lambdi = temp_lambdi(j, k);
				//temp_temp_lambdx = temp_lambdx(j, k);
				//temp_temp_lambdy = temp_lambdy(j, k);
				temp_temp_lambdi = lambdi(j, k);
				temp_temp_lambdx = lambdx(j, k);
				temp_temp_lambdy = lambdy(j, k);
				for (int i = nk - 2; i >= 0; --i) {
					ri0 = xblade - tipgeoexpand_athub(i, j, 0);
					rj0 = yblade - tipgeoexpand_athub(i, j, 1);
					rk0 = zblade - tipgeoexpand_athub(i, j, 2);
					ri1 = xblade - tipgeoexpand_athub(i + 1, j, 0);
					rj1 = yblade - tipgeoexpand_athub(i + 1, j, 1);
					rk1 = zblade - tipgeoexpand_athub(i + 1, j, 2);
					cross(rcros, ri0, rj0, rk0, ri1, rj1, rk1); 
					height = norm(rcros[0], rcros[1], rcros[2]) / norm(ri0 - ri1, rj0 - rj1, rk0 - rk1);
					r0len = norm(ri0, rj0, rk0);
					r1len = norm(ri1, rj1, rk1);
					rdot = dot(ri0, rj0, rk0, ri1, rj1, rk1);
					geofunc = (1.0 / r0len + 1.0 / r1len) / (rdot + r0len*r1len) * 0.5*(tipstr(i, j) + tipstr(i + 1, j));
					geofunc *= pow(height, 2) / sqrt(pow(height, 4) + pow(rc0, 4));
					temp_temp_lambdi += 0.25 / PI * rcros[2] * geofunc / radius / vtipa;
					temp_temp_lambdx += 0.25 / PI * rcros[0] * geofunc / radius / vtipa;
					temp_temp_lambdy += 0.25 / PI * rcros[1] * geofunc / radius / vtipa;

					// debug
					temp_geofunc_x(i, k) = rcros[0] * geofunc / (0.5*(tipstr(i, j) + tipstr(i + 1, j)));
					temp_geofunc_y(i, k) = rcros[1] * geofunc / (0.5*(tipstr(i, j) + tipstr(i + 1, j)));
					temp_geofunc_z(i, k) = rcros[2] * geofunc / (0.5*(tipstr(i, j) + tipstr(i + 1, j)));
				}
				//temp_lambdi(j, k) = temp_temp_lambdi;
				//temp_lambdx(j, k) = temp_temp_lambdx;
				//temp_lambdy(j, k) = temp_temp_lambdy;
				lambdi(j, k) = temp_temp_lambdi;
				lambdx(j, k) = temp_temp_lambdx;
				lambdy(j, k) = temp_temp_lambdy;
			}
		}
		// 随 w 的变化存两遍，所以存的是w = 0的值
		//temp_geofunc_x.output("temp_geofunc_x.output", 4);
		//temp_geofunc_y.output("temp_geofunc_y.output", 4);
		//temp_geofunc_z.output("temp_geofunc_z.output", 4);
	}
	//lambdi = temp_lambdi;
	//lambdx = temp_lambdx;
	//lambdy = temp_lambdy;
	//printf("%f, %f, %f\n", rcros[0], rcros[1], rcros[2]);
	//system("pause");
	//printf("version 0: %f\n", lambdi.sum());
}


void MRotor::InducedVelCompte_v1(void) {
	// input variables

	// output variables
	myTYPE temp_lambdi, temp_lambdx, temp_lambdy;
	// others	
	myTYPE xblade, yblade, zblade, ri0, rj0, rk0, ri1, rj1, rk1;
	myTYPE r0len, r1len, rdot, height, height2, rc04, geofunc;
	myTYPE rcros[3];
	Matrix3<myTYPE> tipgeoexpand_athub;
	Matrix2<myTYPE> tipgeoexpand, transferMatrixfromtpp(3, 3);

	// allocate memory
	lambdi.allocate(nf, ns);
	lambdx.allocate(nf, ns);
	lambdy.allocate(nf, ns);
	rc04 = rc0 * rc0 * rc0 * rc0;

	// transfer geometry from tpp to hub coordinate
	//transferMatrixfromtpp = hubfxcoord.transfer(tppcoord);
	transferMatrixfromtpp(0, 0) = 1;
	transferMatrixfromtpp(0, 1) = 0;
	transferMatrixfromtpp(0, 2) = -beta(1);
	transferMatrixfromtpp(1, 0) = 0;
	transferMatrixfromtpp(1, 1) = 1;
	transferMatrixfromtpp(1, 2) = -beta(2);
	transferMatrixfromtpp(2, 0) = beta(1);
	transferMatrixfromtpp(2, 1) = beta(2);
	transferMatrixfromtpp(2, 2) = 1;
	//transferMatrixfromtpp.output();
	tipgeoexpand = tipgeometry.reshape(3, 3, nk*nf); // 应该要减去在TPP坐标系下TPP原点指向HUB原点的向量
	//tipgeoexpand = transferMatrixfromtpp.matrixmultiply(tipgeoexpand);
	tipgeoexpand = transferMatrixfromtpp.matrixmultiplyTP(tipgeoexpand);
	//tipgeoexpand_athub = tipgeoexpand.reshape(nk, nf, 3);
	tipgeoexpand_athub.allocate(nk, nf, 3);
	for (int k = 0; k < 3; ++k) {
		for (int j = 0; j < nk*nf; ++j) {
			tipgeoexpand_athub(j % nk, j / nk, k) = tipgeoexpand(k, j);
		}
	}

	//Matrix2<myTYPE> temp_geofunc_x(nk, ns), temp_geofunc_y(nk, ns), temp_geofunc_z(nk, ns);
	for (int iz = nf - 1; iz >= 0; --iz) {
		// pre allocate memory based on nblade
#ifdef UL496
		int iz2;
		myTYPE str0_b0, str0_b1, str1_b0, str1_b1;
		iz2 = iz + (int)(nf / NBLADE);
		if (iz2 >= nf) { iz2 -= nf; }
		//tipstr.output("tipstr2.output", 4);
		str0_b0 = tipstr(nk - 1, iz);
		str0_b1 = tipstr(nk - 1, iz2);
#elif UH60A
		int iz2 = 0;
		int iz3 = 0;
#endif // UL496		
		
		for (int ir = ns - 1; ir >= 0; --ir) {
			temp_lambdi = 0;
			temp_lambdx = 0;
			temp_lambdy = 0;		
			xblade = bladedeform(iz, ir, 0);
			yblade = bladedeform(iz, ir, 1);
			zblade = bladedeform(iz, ir, 2);
#ifdef UL496
			// balde 1 $ik == nk - 1$ element points to blade 1 
			ri0 = xblade - tipgeoexpand_athub(nk - 1, iz, 0);
			rj0 = yblade - tipgeoexpand_athub(nk - 1, iz, 1);
			rk0 = zblade - tipgeoexpand_athub(nk - 1, iz, 2);
			r0len = norm(ri0, rj0, rk0);
			for (int ik = nk - 2; ik >= 0; --ik) {
				// !previous! element
				// strength
				str1_b0 = tipstr(ik, iz);
				// geometry
				ri1 = xblade - tipgeoexpand_athub(ik, iz, 0); //tip_athub可以移出去，用一个list来存不同blade的对象
				rj1 = yblade - tipgeoexpand_athub(ik, iz, 1);
				rk1 = zblade - tipgeoexpand_athub(ik, iz, 2);
				cross(rcros, ri0, rj0, rk0, ri1, rj1, rk1);
				height = norm(rcros[0], rcros[1], rcros[2]) / norm(ri0 - ri1, rj0 - rj1, rk0 - rk1);
				height2 = height * height;

				r1len = norm(ri1, rj1, rk1);
				rdot = dot(ri0, rj0, rk0, ri1, rj1, rk1);
				geofunc = -(1.0 / r0len + 1.0 / r1len) / (rdot + r0len * r1len) * 0.5*(str1_b0 + str0_b0);				
				geofunc *= height2 / sqrt(height2 * height2 + rc04);
				temp_lambdi += 0.25 / PI * rcros[2] * geofunc / radius / vtipa;
				temp_lambdx += 0.25 / PI * rcros[0] * geofunc / radius / vtipa;
				temp_lambdy += 0.25 / PI * rcros[1] * geofunc / radius / vtipa;
				// save current element related variables
				str0_b0 = str1_b0;
				ri0 = ri1;
				rj0 = rj1;
				rk0 = rk1;
				r0len = r1len;

				// debug
				//temp_geofunc_x(ik, ir) = rcros[0] * geofunc / (0.5*(str1_b0 + str0_b0));
				//temp_geofunc_y(ik, ir) = rcros[1] * geofunc / (0.5*(str1_b0 + str0_b0));
				//temp_geofunc_z(ik, ir) = rcros[2] * geofunc / (0.5*(str1_b0 + str0_b0));

			}
			// balde 2 $ik == nk - 1$ element points to blade 1 
			ri0 = xblade - tipgeoexpand_athub(nk - 1, iz2, 0);
			rj0 = yblade - tipgeoexpand_athub(nk - 1, iz2, 1);
			rk0 = zblade - tipgeoexpand_athub(nk - 1, iz2, 2);
			r0len = norm(ri0, rj0, rk0);
			for (int ik = nk - 2; ik >= 0; --ik) {
				// !previous!element
				// strength
				str1_b1 = tipstr(ik, iz2);
				ri1 = xblade - tipgeoexpand_athub(ik, iz2, 0);
				rj1 = yblade - tipgeoexpand_athub(ik, iz2, 1);
				rk1 = zblade - tipgeoexpand_athub(ik, iz2, 2);
				cross(rcros, ri0, rj0, rk0, ri1, rj1, rk1);
				height = norm(rcros[0], rcros[1], rcros[2]) / norm(ri0 - ri1, rj0 - rj1, rk0 - rk1);
				height2 = height * height;

				r1len = norm(ri1, rj1, rk1);
				rdot = dot(ri0, rj0, rk0, ri1, rj1, rk1);
				geofunc = -(1.0 / r0len + 1.0 / r1len) / (rdot + r0len * r1len) * 0.5*(str1_b1 + str0_b1);
				geofunc *= height2 / sqrt(height2 * height2 + rc04);
				temp_lambdi += 0.25 / PI * rcros[2] * geofunc / radius / vtipa;
				temp_lambdx += 0.25 / PI * rcros[0] * geofunc / radius / vtipa;
				temp_lambdy += 0.25 / PI * rcros[1] * geofunc / radius / vtipa;
				// save current element related variables
				str0_b1 = str1_b1;
				ri0 = ri1;
				rj0 = rj1;
				rk0 = rk1;
				r0len = r1len;
			}
#elif UH60A
			;

#endif // UL496
			lambdi(iz, ir) = temp_lambdi;
			lambdx(iz, ir) = temp_lambdx;
			lambdy(iz, ir) = temp_lambdy;
		}
	}
	//temp_geofunc_x.output("temp_geofunc_x2.output", 4);
	//temp_geofunc_y.output("temp_geofunc_y2.output", 4);
	//temp_geofunc_z.output("temp_geofunc_z2.output", 4);
	//printf("version 1: %f\n", lambdi.sum());
}


void MRotor::InducedVelCompte_v2(void) {
	// input variables

	// output variables
	myTYPE temp_lambdi, temp_lambdx, temp_lambdy;
	myTYPE lambdi_vect[MAX_SIZE], lambdx_vect[MAX_SIZE], lambdy_vect[MAX_SIZE];
	// others	
	myTYPE xblade, yblade, zblade, ri0, rj0, rk0, ri1, rj1, rk1;
	myTYPE *tipgeoexpand_x_athub[NBLADE], *tipgeoexpand_y_athub[NBLADE], *tipgeoexpand_z_athub[NBLADE];
	myTYPE r0len, r1len, rdot, height, height2, rc04, geofunc;
	myTYPE rcros[3], _temp;
	Matrix3<myTYPE> tipgeoexpand_athub;
	Matrix2<myTYPE> tipgeoexpand, transferMatrixfromtpp(3, 3);
	myTYPE *lambdi_vp = lambdi.v_p;
	myTYPE *lambdx_vp = lambdx.v_p;
	myTYPE *lambdy_vp = lambdy.v_p;

	// allocate memory
	rc04 = rc0 * rc0 * rc0 * rc0;
	// transfer geometry from tpp to hub coordinate
	//transferMatrixfromtpp = hubfxcoord.transfer(tppcoord);
	transferMatrixfromtpp(0, 0) = 1;
	transferMatrixfromtpp(0, 1) = 0;
	transferMatrixfromtpp(0, 2) = -beta(1);
	transferMatrixfromtpp(1, 0) = 0;
	transferMatrixfromtpp(1, 1) = 1;
	transferMatrixfromtpp(1, 2) = -beta(2);
	transferMatrixfromtpp(2, 0) = beta(1);
	transferMatrixfromtpp(2, 1) = beta(2);
	transferMatrixfromtpp(2, 2) = 1;
	tipgeoexpand = tipgeometry.reshape(3, 3, nk*nf); // 应该要减去在TPP坐标系下TPP原点指向HUB原点的向量
	tipgeoexpand = transferMatrixfromtpp.matrixmultiplyTP(tipgeoexpand); // 改成MKL
	tipgeoexpand_athub.allocate(nk, nf, 3);
	for (int k = 0; k < 3; ++k) {
		for (int j = 0; j < nk*nf; ++j) {
			tipgeoexpand_athub(j % nk, j / nk, k) = tipgeoexpand(k, j);
		}
	}

	myTYPE *str0_b0_ptr, *str0_b1_ptr, *str1_b0_ptr, *str1_b1_ptr;
	myTYPE *tipstr_vpp = tipstr.v_p;
	myTYPE *tipgeo_vpp = tipgeoexpand_athub.v_p;
	int tipstr_NI = tipstr.NI;
	int tipgeo_NI = nk;
	int tipgeo_NJ = nf;
	for (int iz = nf - 1; iz >= 0; --iz) {
		// pre allocate memory based on nblade
#ifdef UL496
		int iz2;
		iz2 = iz + (int)(nf / NBLADE);
		if (iz2 >= nf) { iz2 -= nf; }
		//tipstr.output("tipstr2.output", 4);
		str0_b0_ptr = tipstr_vpp + (nk - 1 + iz*tipstr_NI); //tipstr(nk - 1, iz);
		str0_b1_ptr = tipstr_vpp + (nk - 1 + iz2*tipstr_NI);//tipstr(nk - 1, iz2);

		tipgeoexpand_x_athub[0] = tipgeo_vpp + ((nk - 1) + 0 + iz*tipgeo_NI); //tipgeoexpand_athub(nk - 1, iz, 0);
		tipgeoexpand_y_athub[0] = tipgeo_vpp + ((nk - 1) + tipgeo_NI*tipgeo_NJ + iz*tipgeo_NI); // tipgeoexpand_athub(nk - 1, iz, 1);
		tipgeoexpand_z_athub[0] = tipgeo_vpp + ((nk - 1) + 2 * tipgeo_NI*tipgeo_NJ + iz*tipgeo_NI); // tipgeoexpand_athub(nk - 1, iz, 2);
		tipgeoexpand_x_athub[1] = tipgeo_vpp + ((nk - 1) + 0 + iz2*tipgeo_NI); //tipgeoexpand_athub(nk - 1, iz2, 0);
		tipgeoexpand_y_athub[1] = tipgeo_vpp + ((nk - 1) + tipgeo_NI*tipgeo_NJ + iz2*tipgeo_NI); // tipgeoexpand_athub(nk - 1, iz2, 1);
		tipgeoexpand_z_athub[1] = tipgeo_vpp + ((nk - 1) + 2 * tipgeo_NI*tipgeo_NJ + iz2*tipgeo_NI); // tipgeoexpand_athub(nk - 1, iz2, 2);
#elif UH60A
		int iz2 = 0;
		int iz3 = 0;
#endif // UL496		

		for (int ir = ns - 1; ir >= 0; --ir) {
			temp_lambdi = 0;
			temp_lambdx = 0;
			temp_lambdy = 0;
			xblade = bladedeform(iz, ir, 0);
			yblade = bladedeform(iz, ir, 1);
			zblade = bladedeform(iz, ir, 2);
#ifdef UL496
			myTYPE *temp_str0_b0_ptr = str0_b0_ptr;
			myTYPE *temp_str0_b1_ptr = str0_b1_ptr;
			myTYPE *temp_tipgeoexpand_x_athub[NBLADE];
			myTYPE *temp_tipgeoexpand_y_athub[NBLADE];
			myTYPE *temp_tipgeoexpand_z_athub[NBLADE];
			// balde 1 $ik == nk - 1$ element points to blade 1 
			temp_tipgeoexpand_x_athub[0] = tipgeoexpand_x_athub[0];
			temp_tipgeoexpand_y_athub[0] = tipgeoexpand_y_athub[0];
			temp_tipgeoexpand_z_athub[0] = tipgeoexpand_z_athub[0];

			ri0 = xblade - (*(temp_tipgeoexpand_x_athub[0]));
			rj0 = yblade - (*(temp_tipgeoexpand_y_athub[0]));
			rk0 = zblade - (*(temp_tipgeoexpand_z_athub[0]));
			r0len = norm(ri0, rj0, rk0);
			for (int ik = nk - 2; ik >= 0; --ik) {
				// strength				
				//temp_str1_b0_ptr = temp_str0_b0_ptr - 1; // !previous! element
				// geometry
				ri1 = xblade - (*(--temp_tipgeoexpand_x_athub[0]));
				rj1 = yblade - (*(--temp_tipgeoexpand_y_athub[0]));
				rk1 = zblade - (*(--temp_tipgeoexpand_z_athub[0]));
				cross(rcros, ri0, rj0, rk0, ri1, rj1, rk1);
				//height = norm(rcros[0], rcros[1], rcros[2]) / norm(ri0 - ri1, rj0 - rj1, rk0 - rk1);
				height2 = (rcros[0] * rcros[0] + rcros[1] * rcros[1] + rcros[2] * rcros[2]) / ((ri0 - ri1)*(ri0 - ri1) + (rj0 - rj1)*(rj0 - rj1) + (rk0 - rk1)*(rk0 - rk1));
				//height2 = height * height;

				r1len = norm(ri1, rj1, rk1);
				rdot = dot(ri0, rj0, rk0, ri1, rj1, rk1);
				geofunc = -(1.0 / r0len + 1.0 / r1len) / (rdot + r0len * r1len);
				//geofunc *= height2 / sqrt(height2 * height2 + rc04);
				geofunc *= (1 - 0.5 * rc04 / height2 / height2);
				geofunc *= 0.5*(*(--temp_str0_b0_ptr) + (*temp_str0_b0_ptr));
				_temp = 0.25 / PI * geofunc / radius / vtipa;

				temp_lambdx += rcros[0] * _temp;
				temp_lambdy += rcros[1] * _temp;
				temp_lambdi += rcros[2] * _temp;

				// save current element related variables
				//str0_b0_ptr = str1_b0_ptr;
				//--temp_str0_b0_ptr;
				ri0 = ri1;
				rj0 = rj1;
				rk0 = rk1;
				r0len = r1len;
			}
			// balde 2 $ik == nk - 1$ element points to blade 1 
			temp_tipgeoexpand_x_athub[1] = tipgeoexpand_x_athub[1];
			temp_tipgeoexpand_y_athub[1] = tipgeoexpand_y_athub[1];
			temp_tipgeoexpand_z_athub[1] = tipgeoexpand_z_athub[1];

			ri0 = xblade - (*(temp_tipgeoexpand_x_athub[1]));
			rj0 = yblade - (*(temp_tipgeoexpand_y_athub[1]));
			rk0 = zblade - (*(temp_tipgeoexpand_z_athub[1]));
			r0len = norm(ri0, rj0, rk0);
			for (int ik = nk - 2; ik >= 0; --ik) {
				// strength
				//str1_b1_ptr = str0_b1_ptr - 1; // !previous!element
				// geometry
				ri1 = xblade - (*(--temp_tipgeoexpand_x_athub[1]));
				rj1 = yblade - (*(--temp_tipgeoexpand_y_athub[1]));
				rk1 = zblade - (*(--temp_tipgeoexpand_z_athub[1]));
				cross(rcros, ri0, rj0, rk0, ri1, rj1, rk1);
				//height = norm(rcros[0], rcros[1], rcros[2]) / norm(ri0 - ri1, rj0 - rj1, rk0 - rk1);
				height2 = (rcros[0] * rcros[0] + rcros[1] * rcros[1] + rcros[2] * rcros[2]) / ((ri0 - ri1)*(ri0 - ri1) + (rj0 - rj1)*(rj0 - rj1) + (rk0 - rk1)*(rk0 - rk1));
				//height2 = height * height;

				r1len = norm(ri1, rj1, rk1);
				rdot = dot(ri0, rj0, rk0, ri1, rj1, rk1);
				geofunc = -(1.0 / r0len + 1.0 / r1len) / (rdot + r0len * r1len);
				//geofunc *= height2 / sqrt(height2 * height2 + rc04);
				geofunc *= (1 - 0.5 * rc04 / height2 / height2);
				geofunc *= 0.5*(*(--temp_str0_b1_ptr) + (*temp_str0_b1_ptr)); // get value first,  left shift and get value
				_temp = 0.25 / PI * geofunc / radius / vtipa;

				temp_lambdx += rcros[0] * _temp;
				temp_lambdy += rcros[1] * _temp;
				temp_lambdi += rcros[2] * _temp;

				// save current element related variables
				//str0_b0_ptr = str1_b0_ptr;
				//--temp_str0_b1_ptr;
				ri0 = ri1;
				rj0 = rj1;
				rk0 = rk1;
				r0len = r1len;
			}
#elif UH60A
			;

#endif // UL496
			
			if (!DISABLE_REVISE_SIZE) {
				lambdi(iz, ir) = temp_lambdi;
				lambdx(iz, ir) = temp_lambdx;
				lambdy(iz, ir) = temp_lambdy;
			}
			else {
				lambdi(iz, ir) = temp_lambdi;
				lambdx(iz, ir) = temp_lambdx;
				lambdy(iz, ir) = temp_lambdy;
			}
		}
	}
	//printf("version 2: %f\n", lambdi.sum());
}

