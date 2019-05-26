
#include "LowPassFilter.h"


LowPassFilter::LowPassFilter(double Ts, double wn, double xi)
{
	bInit_ = false;

	dYk_ = dYk_1_ = dYk_2_ = 0.;

	dUk_1_ =  dUk_2_ = 0.;

	set(Ts, wn, xi);
}

LowPassFilter::~LowPassFilter()
{}

void LowPassFilter::init(double val)
{
	dYk_ = dYk_1_ =  dYk_2_ = val;
	dUk_1_ =  dUk_2_ = 0;

	bInit_ = true;
}


void LowPassFilter::set(double Ts, double wn, double xi)
{
	/// computed by using the MATLAB file digitalImplementationSecondOrderFilter.m

    	//parameters initialization 
	dTs_ = Ts;
	dWn_ = wn;
	dXi_ = xi;

	double Ts2 = Ts*Ts;
	double wn2 = wn*wn;

	dC1_ = -(2.*(Ts2*wn2 - 4.))/(Ts2*wn2 + 4.*xi*Ts*wn + 4.);
	
	dC2_ = (8.*Ts*wn*xi)/(Ts2*wn2 + 4.*xi*Ts*wn + 4.) - 1.;
	
	dC3_ = (Ts2*wn2)/(Ts2*wn2 + 4.*xi*Ts*wn + 4.);
	
	dC4_ = (2.*Ts2*wn2)/(Ts2*wn2 + 4*xi*Ts*wn + 4.);

	dC5_ = (Ts2*wn2)/(Ts2*wn2 + 4*xi*Ts*wn + 4);
}


void LowPassFilter::setTs(double Ts)
{
	set(dTs_, dWn_, dXi_);	
}

void LowPassFilter::setWn(double wn)
{
	set(dTs_, wn, dXi_);
}


///--------------------------------------------------------------------------------------
///--------------------------------------------------------------------------------------


LowPassFilter1stOrd::LowPassFilter1stOrd(double k_in, double tau_in,  double Tc_in)
{
	bInit_ = false;
	dUk_1_ = 0.; 
	dYk_ = dYk_1_=0.;

	set( k_in, tau_in, Tc_in);
}

void LowPassFilter1stOrd::init(double uk)
{
	dYk_ = dYk_1_  = uk;
	dUk_1_  = 0;

	bInit_ = true;
}

void LowPassFilter1stOrd::set(double k_in, double tau_in,  double Tc_in)
{
	dK_   = k_in;
	dTau_ = tau_in;
	dTc_  = Tc_in;
	
	//parameters initialization 
	
	dC0_ = dTc_/(dTc_+2.*dTau_);
	dC1_ = dC0_;
	dC2_ =(-dTc_+2.*dTau_)/(dTc_+2.*dTau_);
}
