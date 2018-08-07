// *************************************
// C++ routines for this program are taken from
// a translation of the FORTRAN code written by
// U.S. Department of Commerce NTIA/ITS
// Institute for Telecommunication Sciences
// *****************
// Irregular Terrain Model (ITM) (Longley-Rice)
// *************************************



#include <math.h>
#include <complex>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <stdint.h>

#ifndef	__LIBRADIO_ITM_H__
#define	__LIBRADIO_ITM_H__

#define THIRD  (1.0/3.0)
#define ITMVERSION (7.0)

extern "C" {

struct tcomplex {
  double tcreal;
  double tcimag;
};

struct prop_type {
  double aref;
  double dist;
  double hg[2];
  double wn;
  double dh;
  double ens;
  double gme;
  double zgndreal;
  double zgndimag;
  double he[2];
  double dl[2];
  double the[2];
  int kwx;
  int mdp;
};

struct propv_type {
  double sgc;
  int lvar;
  int mdvar;
  int klim;
};

struct propa_type {
  double dlsa;
  double dx;
  double ael;
  double ak1;
  double ak2;
  double aed;
  double emd;
  double aes;
  double ems;
  double dls[2];
  double dla;
  double tha;
};

void point_to_point(double elev[], double tht_m, double rht_m,
                                      double eps_dielect, double sgm_conductivity, double eno_ns_surfref,
                                      double frq_mhz, int radio_climate, int pol, double conf, double rel,
                                      double &dbloss, char *strmode, int &errnum);

void point_to_pointMDH(double elev[], double tht_m, double rht_m,
                                         double eps_dielect, double sgm_conductivity, double eno_ns_surfref,
                                         double frq_mhz, int radio_climate, int pol, double timepct, double locpct, double confpct,
                                         double &dbloss, int &propmode, double &deltaH, int &errnum);

void area(long ModVar, double deltaH, double tht_m, double rht_m,
                            double dist_km, int TSiteCriteria, int RSiteCriteria,
                            double eps_dielect, double sgm_conductivity, double eno_ns_surfref,
                            double frq_mhz, int radio_climate, int pol, double pctTime, double pctLoc,
                            double pctConf, double &dbloss, char *strmode, int &errnum);

void point_to_pointDH(double elev[], double tht_m, double rht_m,
                                        double eps_dielect, double sgm_conductivity, double eno_ns_surfref,
                                        double frq_mhz, int radio_climate, int pol, double conf, double rel,
                                        double &dbloss, double &deltaH, int &errnum);
double ITMAreadBLoss(long ModVar, double deltaH, double tht_m, double rht_m,
                                       double dist_km, int TSiteCriteria, int RSiteCriteria,
                                       double eps_dielect, double sgm_conductivity, double eno_ns_surfref,
                                       double frq_mhz, int radio_climate, int pol, double pctTime, double pctLoc,
                                       double pctConf);

}

inline double deg2rad(double d) {
  return d * 3.1415926535897 / 180.0;
}

inline double curve(double const &c1, double const &c2, double const &x1,
             double const &x2, double const &x3, double const &de) {
  return (c1 + c2 / (1.0 + pow((de - x2) / x3, 2.0))) * pow(de / x1, 2.0) /
         (1.0 + pow(de / x1, 2.0));
}

inline double abq_alos(std::complex<double> r) { return r.real() * r.real() + r.imag() * r.imag(); }

inline double qerf(const double &z) {
  double b1 = 0.319381530, b2 = -0.356563782, b3 = 1.781477937;
  double b4 = -1.821255987, b5 = 1.330274429;
  double rp = 4.317008, rrt2pi = 0.398942280;
  double t, x, qerfv;
  x = z;
  t = fabs(x);
  if (t >= 10.0)
    qerfv = 0.0;
  else {
    t = rp / (t + rp);
    qerfv = exp(-0.5 * x * x) * rrt2pi * ((((b5 * t + b4) * t + b3) * t + b2) * t + b1) * t;
  }
  if (x < 0.0) qerfv = 1.0 - qerfv;
  return qerfv;
}


inline int mymin(const int &i, const int &j) {
  if (i < j)
    return i;
  else
    return j;
}

inline int mymax(const int &i, const int &j) {
  if (i > j)
    return i;
  else
    return j;
}

inline double mymin(const double &a, const double &b) {
  if (a < b)
    return a;
  else
    return b;
}

inline double mymax(const double &a, const double &b) {
  if (a > b)
    return a;
  else
    return b;
}

inline double FORTRAN_DIM(const double &x, const double &y) { // This performs the FORTRAN DIM function.
  // result is x-y if x is greater than y; otherwise result is 0.0
  if (x > y)
    return x - y;
  else
    return 0.0;
}

inline double aknfe(const double &v2) {
  double a;
  if (v2 < 5.76)
    a = 6.02 + 9.11 * sqrt(v2) - 1.27 * v2;
  else
    a = 12.953 + 4.343 * log(v2);
  return a;
}


inline double qerfi(double q) {
  double x, t, v;
  double c0 = 2.515516698;
  double c1 = 0.802853;
  double c2 = 0.010328;
  double d1 = 1.432788;
  double d2 = 0.189269;
  double d3 = 0.001308;

  x = 0.5 - q;
  t = mymax(0.5 - fabs(x), 0.000001);
  t = sqrt(-2.0 * log(t));
  v = t - ((c2 * t + c1) * t + c0) / (((d3 * t + d2) * t + d1) * t + 1.0);
  if (x < 0.0) v = -v;
  return v;
}

inline double fht(const double &x, const double &pk) {
  double w, fhtv;
  if (x < 200.0) {
    w = -log(pk);
    if (pk < 1e-5 || x * pow(w, 3.0) > 5495.0) {
      fhtv = -117.0;
      if (x > 1.0)
        fhtv = 17.372 * log(x) + fhtv;
    }
    else
      fhtv = 2.5e-5 * x * x / pk - 8.686 * w - 15.0;
  }
  else {
    fhtv = 0.05751 * x - 4.343 * log(x);
    if (x < 2000.0) {
      w = 0.0134 * x * exp(-0.005 * x);
      fhtv = (1.0 - w) * fhtv + w * (17.372 * log(x) - 117.0);
    }
  }
  return fhtv;
}

inline double h0f(double r, double et) {
  double a[5] = {25.0, 80.0, 177.0, 395.0, 705.0};
  double b[5] = {24.0, 45.0, 68.0, 80.0, 105.0};
  double q, x;
  int it;
  double h0fv;
  it = (int) et;
  if (it <= 0) {
    it = 1;
    q = 0.0;
  }
  else if (it >= 5) {
    it = 5;
    q = 0.0;
  }
  else
    q = et - it;
  x = pow(1.0 / r, 2.0);
  h0fv = 4.343 * log((a[it - 1] * x + b[it - 1]) * x + 1.0);
  if (q != 0.0)
    h0fv = (1.0 - q) * h0fv + q * 4.343 * log((a[it] * x + b[it]) * x + 1.0);
  return h0fv;
}

inline double ahd(double td) {
  int i;
  double a[3] = {133.4, 104.6, 71.8};
  double b[3] = {0.332e-3, 0.212e-3, 0.157e-3};
  double c[3] = {-4.343, -1.086, 2.171};
  if (td <= 10e3)
    i = 0;
  else if (td <= 70e3)
    i = 1;
  else
    i = 2;
  return a[i] + b[i] * td + c[i] * log(td);
}

#endif	/* __LIBRADIO_ITM_H__ */
