#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7570101262000998390) {
   out_7570101262000998390[0] = delta_x[0] + nom_x[0];
   out_7570101262000998390[1] = delta_x[1] + nom_x[1];
   out_7570101262000998390[2] = delta_x[2] + nom_x[2];
   out_7570101262000998390[3] = delta_x[3] + nom_x[3];
   out_7570101262000998390[4] = delta_x[4] + nom_x[4];
   out_7570101262000998390[5] = delta_x[5] + nom_x[5];
   out_7570101262000998390[6] = delta_x[6] + nom_x[6];
   out_7570101262000998390[7] = delta_x[7] + nom_x[7];
   out_7570101262000998390[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8378965514508472172) {
   out_8378965514508472172[0] = -nom_x[0] + true_x[0];
   out_8378965514508472172[1] = -nom_x[1] + true_x[1];
   out_8378965514508472172[2] = -nom_x[2] + true_x[2];
   out_8378965514508472172[3] = -nom_x[3] + true_x[3];
   out_8378965514508472172[4] = -nom_x[4] + true_x[4];
   out_8378965514508472172[5] = -nom_x[5] + true_x[5];
   out_8378965514508472172[6] = -nom_x[6] + true_x[6];
   out_8378965514508472172[7] = -nom_x[7] + true_x[7];
   out_8378965514508472172[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2284935391344495239) {
   out_2284935391344495239[0] = 1.0;
   out_2284935391344495239[1] = 0;
   out_2284935391344495239[2] = 0;
   out_2284935391344495239[3] = 0;
   out_2284935391344495239[4] = 0;
   out_2284935391344495239[5] = 0;
   out_2284935391344495239[6] = 0;
   out_2284935391344495239[7] = 0;
   out_2284935391344495239[8] = 0;
   out_2284935391344495239[9] = 0;
   out_2284935391344495239[10] = 1.0;
   out_2284935391344495239[11] = 0;
   out_2284935391344495239[12] = 0;
   out_2284935391344495239[13] = 0;
   out_2284935391344495239[14] = 0;
   out_2284935391344495239[15] = 0;
   out_2284935391344495239[16] = 0;
   out_2284935391344495239[17] = 0;
   out_2284935391344495239[18] = 0;
   out_2284935391344495239[19] = 0;
   out_2284935391344495239[20] = 1.0;
   out_2284935391344495239[21] = 0;
   out_2284935391344495239[22] = 0;
   out_2284935391344495239[23] = 0;
   out_2284935391344495239[24] = 0;
   out_2284935391344495239[25] = 0;
   out_2284935391344495239[26] = 0;
   out_2284935391344495239[27] = 0;
   out_2284935391344495239[28] = 0;
   out_2284935391344495239[29] = 0;
   out_2284935391344495239[30] = 1.0;
   out_2284935391344495239[31] = 0;
   out_2284935391344495239[32] = 0;
   out_2284935391344495239[33] = 0;
   out_2284935391344495239[34] = 0;
   out_2284935391344495239[35] = 0;
   out_2284935391344495239[36] = 0;
   out_2284935391344495239[37] = 0;
   out_2284935391344495239[38] = 0;
   out_2284935391344495239[39] = 0;
   out_2284935391344495239[40] = 1.0;
   out_2284935391344495239[41] = 0;
   out_2284935391344495239[42] = 0;
   out_2284935391344495239[43] = 0;
   out_2284935391344495239[44] = 0;
   out_2284935391344495239[45] = 0;
   out_2284935391344495239[46] = 0;
   out_2284935391344495239[47] = 0;
   out_2284935391344495239[48] = 0;
   out_2284935391344495239[49] = 0;
   out_2284935391344495239[50] = 1.0;
   out_2284935391344495239[51] = 0;
   out_2284935391344495239[52] = 0;
   out_2284935391344495239[53] = 0;
   out_2284935391344495239[54] = 0;
   out_2284935391344495239[55] = 0;
   out_2284935391344495239[56] = 0;
   out_2284935391344495239[57] = 0;
   out_2284935391344495239[58] = 0;
   out_2284935391344495239[59] = 0;
   out_2284935391344495239[60] = 1.0;
   out_2284935391344495239[61] = 0;
   out_2284935391344495239[62] = 0;
   out_2284935391344495239[63] = 0;
   out_2284935391344495239[64] = 0;
   out_2284935391344495239[65] = 0;
   out_2284935391344495239[66] = 0;
   out_2284935391344495239[67] = 0;
   out_2284935391344495239[68] = 0;
   out_2284935391344495239[69] = 0;
   out_2284935391344495239[70] = 1.0;
   out_2284935391344495239[71] = 0;
   out_2284935391344495239[72] = 0;
   out_2284935391344495239[73] = 0;
   out_2284935391344495239[74] = 0;
   out_2284935391344495239[75] = 0;
   out_2284935391344495239[76] = 0;
   out_2284935391344495239[77] = 0;
   out_2284935391344495239[78] = 0;
   out_2284935391344495239[79] = 0;
   out_2284935391344495239[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1508310104116332664) {
   out_1508310104116332664[0] = state[0];
   out_1508310104116332664[1] = state[1];
   out_1508310104116332664[2] = state[2];
   out_1508310104116332664[3] = state[3];
   out_1508310104116332664[4] = state[4];
   out_1508310104116332664[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1508310104116332664[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1508310104116332664[7] = state[7];
   out_1508310104116332664[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8054310567907954818) {
   out_8054310567907954818[0] = 1;
   out_8054310567907954818[1] = 0;
   out_8054310567907954818[2] = 0;
   out_8054310567907954818[3] = 0;
   out_8054310567907954818[4] = 0;
   out_8054310567907954818[5] = 0;
   out_8054310567907954818[6] = 0;
   out_8054310567907954818[7] = 0;
   out_8054310567907954818[8] = 0;
   out_8054310567907954818[9] = 0;
   out_8054310567907954818[10] = 1;
   out_8054310567907954818[11] = 0;
   out_8054310567907954818[12] = 0;
   out_8054310567907954818[13] = 0;
   out_8054310567907954818[14] = 0;
   out_8054310567907954818[15] = 0;
   out_8054310567907954818[16] = 0;
   out_8054310567907954818[17] = 0;
   out_8054310567907954818[18] = 0;
   out_8054310567907954818[19] = 0;
   out_8054310567907954818[20] = 1;
   out_8054310567907954818[21] = 0;
   out_8054310567907954818[22] = 0;
   out_8054310567907954818[23] = 0;
   out_8054310567907954818[24] = 0;
   out_8054310567907954818[25] = 0;
   out_8054310567907954818[26] = 0;
   out_8054310567907954818[27] = 0;
   out_8054310567907954818[28] = 0;
   out_8054310567907954818[29] = 0;
   out_8054310567907954818[30] = 1;
   out_8054310567907954818[31] = 0;
   out_8054310567907954818[32] = 0;
   out_8054310567907954818[33] = 0;
   out_8054310567907954818[34] = 0;
   out_8054310567907954818[35] = 0;
   out_8054310567907954818[36] = 0;
   out_8054310567907954818[37] = 0;
   out_8054310567907954818[38] = 0;
   out_8054310567907954818[39] = 0;
   out_8054310567907954818[40] = 1;
   out_8054310567907954818[41] = 0;
   out_8054310567907954818[42] = 0;
   out_8054310567907954818[43] = 0;
   out_8054310567907954818[44] = 0;
   out_8054310567907954818[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8054310567907954818[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8054310567907954818[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8054310567907954818[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8054310567907954818[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8054310567907954818[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8054310567907954818[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8054310567907954818[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8054310567907954818[53] = -9.8000000000000007*dt;
   out_8054310567907954818[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8054310567907954818[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8054310567907954818[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8054310567907954818[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8054310567907954818[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8054310567907954818[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8054310567907954818[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8054310567907954818[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8054310567907954818[62] = 0;
   out_8054310567907954818[63] = 0;
   out_8054310567907954818[64] = 0;
   out_8054310567907954818[65] = 0;
   out_8054310567907954818[66] = 0;
   out_8054310567907954818[67] = 0;
   out_8054310567907954818[68] = 0;
   out_8054310567907954818[69] = 0;
   out_8054310567907954818[70] = 1;
   out_8054310567907954818[71] = 0;
   out_8054310567907954818[72] = 0;
   out_8054310567907954818[73] = 0;
   out_8054310567907954818[74] = 0;
   out_8054310567907954818[75] = 0;
   out_8054310567907954818[76] = 0;
   out_8054310567907954818[77] = 0;
   out_8054310567907954818[78] = 0;
   out_8054310567907954818[79] = 0;
   out_8054310567907954818[80] = 1;
}
void h_25(double *state, double *unused, double *out_3075084501222235420) {
   out_3075084501222235420[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8710286078482917812) {
   out_8710286078482917812[0] = 0;
   out_8710286078482917812[1] = 0;
   out_8710286078482917812[2] = 0;
   out_8710286078482917812[3] = 0;
   out_8710286078482917812[4] = 0;
   out_8710286078482917812[5] = 0;
   out_8710286078482917812[6] = 1;
   out_8710286078482917812[7] = 0;
   out_8710286078482917812[8] = 0;
}
void h_24(double *state, double *unused, double *out_1935496555183779272) {
   out_1935496555183779272[0] = state[4];
   out_1935496555183779272[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1267611485494201492) {
   out_1267611485494201492[0] = 0;
   out_1267611485494201492[1] = 0;
   out_1267611485494201492[2] = 0;
   out_1267611485494201492[3] = 0;
   out_1267611485494201492[4] = 1;
   out_1267611485494201492[5] = 0;
   out_1267611485494201492[6] = 0;
   out_1267611485494201492[7] = 0;
   out_1267611485494201492[8] = 0;
   out_1267611485494201492[9] = 0;
   out_1267611485494201492[10] = 0;
   out_1267611485494201492[11] = 0;
   out_1267611485494201492[12] = 0;
   out_1267611485494201492[13] = 0;
   out_1267611485494201492[14] = 1;
   out_1267611485494201492[15] = 0;
   out_1267611485494201492[16] = 0;
   out_1267611485494201492[17] = 0;
}
void h_30(double *state, double *unused, double *out_4525715893851118587) {
   out_4525715893851118587[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4182589748355309614) {
   out_4182589748355309614[0] = 0;
   out_4182589748355309614[1] = 0;
   out_4182589748355309614[2] = 0;
   out_4182589748355309614[3] = 0;
   out_4182589748355309614[4] = 1;
   out_4182589748355309614[5] = 0;
   out_4182589748355309614[6] = 0;
   out_4182589748355309614[7] = 0;
   out_4182589748355309614[8] = 0;
}
void h_26(double *state, double *unused, double *out_1713163790581265204) {
   out_1713163790581265204[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4968782759608861588) {
   out_4968782759608861588[0] = 0;
   out_4968782759608861588[1] = 0;
   out_4968782759608861588[2] = 0;
   out_4968782759608861588[3] = 0;
   out_4968782759608861588[4] = 0;
   out_4968782759608861588[5] = 0;
   out_4968782759608861588[6] = 0;
   out_4968782759608861588[7] = 1;
   out_4968782759608861588[8] = 0;
}
void h_27(double *state, double *unused, double *out_8368896868694250417) {
   out_8368896868694250417[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2007826436554884703) {
   out_2007826436554884703[0] = 0;
   out_2007826436554884703[1] = 0;
   out_2007826436554884703[2] = 0;
   out_2007826436554884703[3] = 1;
   out_2007826436554884703[4] = 0;
   out_2007826436554884703[5] = 0;
   out_2007826436554884703[6] = 0;
   out_2007826436554884703[7] = 0;
   out_2007826436554884703[8] = 0;
}
void h_29(double *state, double *unused, double *out_475957373809727058) {
   out_475957373809727058[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4692821092669701798) {
   out_4692821092669701798[0] = 0;
   out_4692821092669701798[1] = 1;
   out_4692821092669701798[2] = 0;
   out_4692821092669701798[3] = 0;
   out_4692821092669701798[4] = 0;
   out_4692821092669701798[5] = 0;
   out_4692821092669701798[6] = 0;
   out_4692821092669701798[7] = 0;
   out_4692821092669701798[8] = 0;
}
void h_28(double *state, double *unused, double *out_613072303312590348) {
   out_613072303312590348[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6656451364235028049) {
   out_6656451364235028049[0] = 1;
   out_6656451364235028049[1] = 0;
   out_6656451364235028049[2] = 0;
   out_6656451364235028049[3] = 0;
   out_6656451364235028049[4] = 0;
   out_6656451364235028049[5] = 0;
   out_6656451364235028049[6] = 0;
   out_6656451364235028049[7] = 0;
   out_6656451364235028049[8] = 0;
}
void h_31(double *state, double *unused, double *out_3092029538748898883) {
   out_3092029538748898883[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4342574657375510112) {
   out_4342574657375510112[0] = 0;
   out_4342574657375510112[1] = 0;
   out_4342574657375510112[2] = 0;
   out_4342574657375510112[3] = 0;
   out_4342574657375510112[4] = 0;
   out_4342574657375510112[5] = 0;
   out_4342574657375510112[6] = 0;
   out_4342574657375510112[7] = 0;
   out_4342574657375510112[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7570101262000998390) {
  err_fun(nom_x, delta_x, out_7570101262000998390);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8378965514508472172) {
  inv_err_fun(nom_x, true_x, out_8378965514508472172);
}
void car_H_mod_fun(double *state, double *out_2284935391344495239) {
  H_mod_fun(state, out_2284935391344495239);
}
void car_f_fun(double *state, double dt, double *out_1508310104116332664) {
  f_fun(state,  dt, out_1508310104116332664);
}
void car_F_fun(double *state, double dt, double *out_8054310567907954818) {
  F_fun(state,  dt, out_8054310567907954818);
}
void car_h_25(double *state, double *unused, double *out_3075084501222235420) {
  h_25(state, unused, out_3075084501222235420);
}
void car_H_25(double *state, double *unused, double *out_8710286078482917812) {
  H_25(state, unused, out_8710286078482917812);
}
void car_h_24(double *state, double *unused, double *out_1935496555183779272) {
  h_24(state, unused, out_1935496555183779272);
}
void car_H_24(double *state, double *unused, double *out_1267611485494201492) {
  H_24(state, unused, out_1267611485494201492);
}
void car_h_30(double *state, double *unused, double *out_4525715893851118587) {
  h_30(state, unused, out_4525715893851118587);
}
void car_H_30(double *state, double *unused, double *out_4182589748355309614) {
  H_30(state, unused, out_4182589748355309614);
}
void car_h_26(double *state, double *unused, double *out_1713163790581265204) {
  h_26(state, unused, out_1713163790581265204);
}
void car_H_26(double *state, double *unused, double *out_4968782759608861588) {
  H_26(state, unused, out_4968782759608861588);
}
void car_h_27(double *state, double *unused, double *out_8368896868694250417) {
  h_27(state, unused, out_8368896868694250417);
}
void car_H_27(double *state, double *unused, double *out_2007826436554884703) {
  H_27(state, unused, out_2007826436554884703);
}
void car_h_29(double *state, double *unused, double *out_475957373809727058) {
  h_29(state, unused, out_475957373809727058);
}
void car_H_29(double *state, double *unused, double *out_4692821092669701798) {
  H_29(state, unused, out_4692821092669701798);
}
void car_h_28(double *state, double *unused, double *out_613072303312590348) {
  h_28(state, unused, out_613072303312590348);
}
void car_H_28(double *state, double *unused, double *out_6656451364235028049) {
  H_28(state, unused, out_6656451364235028049);
}
void car_h_31(double *state, double *unused, double *out_3092029538748898883) {
  h_31(state, unused, out_3092029538748898883);
}
void car_H_31(double *state, double *unused, double *out_4342574657375510112) {
  H_31(state, unused, out_4342574657375510112);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
