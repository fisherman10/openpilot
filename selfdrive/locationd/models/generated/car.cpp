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
void err_fun(double *nom_x, double *delta_x, double *out_2829633900128880649) {
   out_2829633900128880649[0] = delta_x[0] + nom_x[0];
   out_2829633900128880649[1] = delta_x[1] + nom_x[1];
   out_2829633900128880649[2] = delta_x[2] + nom_x[2];
   out_2829633900128880649[3] = delta_x[3] + nom_x[3];
   out_2829633900128880649[4] = delta_x[4] + nom_x[4];
   out_2829633900128880649[5] = delta_x[5] + nom_x[5];
   out_2829633900128880649[6] = delta_x[6] + nom_x[6];
   out_2829633900128880649[7] = delta_x[7] + nom_x[7];
   out_2829633900128880649[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7762599449520589818) {
   out_7762599449520589818[0] = -nom_x[0] + true_x[0];
   out_7762599449520589818[1] = -nom_x[1] + true_x[1];
   out_7762599449520589818[2] = -nom_x[2] + true_x[2];
   out_7762599449520589818[3] = -nom_x[3] + true_x[3];
   out_7762599449520589818[4] = -nom_x[4] + true_x[4];
   out_7762599449520589818[5] = -nom_x[5] + true_x[5];
   out_7762599449520589818[6] = -nom_x[6] + true_x[6];
   out_7762599449520589818[7] = -nom_x[7] + true_x[7];
   out_7762599449520589818[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1808600300430226605) {
   out_1808600300430226605[0] = 1.0;
   out_1808600300430226605[1] = 0;
   out_1808600300430226605[2] = 0;
   out_1808600300430226605[3] = 0;
   out_1808600300430226605[4] = 0;
   out_1808600300430226605[5] = 0;
   out_1808600300430226605[6] = 0;
   out_1808600300430226605[7] = 0;
   out_1808600300430226605[8] = 0;
   out_1808600300430226605[9] = 0;
   out_1808600300430226605[10] = 1.0;
   out_1808600300430226605[11] = 0;
   out_1808600300430226605[12] = 0;
   out_1808600300430226605[13] = 0;
   out_1808600300430226605[14] = 0;
   out_1808600300430226605[15] = 0;
   out_1808600300430226605[16] = 0;
   out_1808600300430226605[17] = 0;
   out_1808600300430226605[18] = 0;
   out_1808600300430226605[19] = 0;
   out_1808600300430226605[20] = 1.0;
   out_1808600300430226605[21] = 0;
   out_1808600300430226605[22] = 0;
   out_1808600300430226605[23] = 0;
   out_1808600300430226605[24] = 0;
   out_1808600300430226605[25] = 0;
   out_1808600300430226605[26] = 0;
   out_1808600300430226605[27] = 0;
   out_1808600300430226605[28] = 0;
   out_1808600300430226605[29] = 0;
   out_1808600300430226605[30] = 1.0;
   out_1808600300430226605[31] = 0;
   out_1808600300430226605[32] = 0;
   out_1808600300430226605[33] = 0;
   out_1808600300430226605[34] = 0;
   out_1808600300430226605[35] = 0;
   out_1808600300430226605[36] = 0;
   out_1808600300430226605[37] = 0;
   out_1808600300430226605[38] = 0;
   out_1808600300430226605[39] = 0;
   out_1808600300430226605[40] = 1.0;
   out_1808600300430226605[41] = 0;
   out_1808600300430226605[42] = 0;
   out_1808600300430226605[43] = 0;
   out_1808600300430226605[44] = 0;
   out_1808600300430226605[45] = 0;
   out_1808600300430226605[46] = 0;
   out_1808600300430226605[47] = 0;
   out_1808600300430226605[48] = 0;
   out_1808600300430226605[49] = 0;
   out_1808600300430226605[50] = 1.0;
   out_1808600300430226605[51] = 0;
   out_1808600300430226605[52] = 0;
   out_1808600300430226605[53] = 0;
   out_1808600300430226605[54] = 0;
   out_1808600300430226605[55] = 0;
   out_1808600300430226605[56] = 0;
   out_1808600300430226605[57] = 0;
   out_1808600300430226605[58] = 0;
   out_1808600300430226605[59] = 0;
   out_1808600300430226605[60] = 1.0;
   out_1808600300430226605[61] = 0;
   out_1808600300430226605[62] = 0;
   out_1808600300430226605[63] = 0;
   out_1808600300430226605[64] = 0;
   out_1808600300430226605[65] = 0;
   out_1808600300430226605[66] = 0;
   out_1808600300430226605[67] = 0;
   out_1808600300430226605[68] = 0;
   out_1808600300430226605[69] = 0;
   out_1808600300430226605[70] = 1.0;
   out_1808600300430226605[71] = 0;
   out_1808600300430226605[72] = 0;
   out_1808600300430226605[73] = 0;
   out_1808600300430226605[74] = 0;
   out_1808600300430226605[75] = 0;
   out_1808600300430226605[76] = 0;
   out_1808600300430226605[77] = 0;
   out_1808600300430226605[78] = 0;
   out_1808600300430226605[79] = 0;
   out_1808600300430226605[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6453172057652161082) {
   out_6453172057652161082[0] = state[0];
   out_6453172057652161082[1] = state[1];
   out_6453172057652161082[2] = state[2];
   out_6453172057652161082[3] = state[3];
   out_6453172057652161082[4] = state[4];
   out_6453172057652161082[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6453172057652161082[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6453172057652161082[7] = state[7];
   out_6453172057652161082[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2576361439132911649) {
   out_2576361439132911649[0] = 1;
   out_2576361439132911649[1] = 0;
   out_2576361439132911649[2] = 0;
   out_2576361439132911649[3] = 0;
   out_2576361439132911649[4] = 0;
   out_2576361439132911649[5] = 0;
   out_2576361439132911649[6] = 0;
   out_2576361439132911649[7] = 0;
   out_2576361439132911649[8] = 0;
   out_2576361439132911649[9] = 0;
   out_2576361439132911649[10] = 1;
   out_2576361439132911649[11] = 0;
   out_2576361439132911649[12] = 0;
   out_2576361439132911649[13] = 0;
   out_2576361439132911649[14] = 0;
   out_2576361439132911649[15] = 0;
   out_2576361439132911649[16] = 0;
   out_2576361439132911649[17] = 0;
   out_2576361439132911649[18] = 0;
   out_2576361439132911649[19] = 0;
   out_2576361439132911649[20] = 1;
   out_2576361439132911649[21] = 0;
   out_2576361439132911649[22] = 0;
   out_2576361439132911649[23] = 0;
   out_2576361439132911649[24] = 0;
   out_2576361439132911649[25] = 0;
   out_2576361439132911649[26] = 0;
   out_2576361439132911649[27] = 0;
   out_2576361439132911649[28] = 0;
   out_2576361439132911649[29] = 0;
   out_2576361439132911649[30] = 1;
   out_2576361439132911649[31] = 0;
   out_2576361439132911649[32] = 0;
   out_2576361439132911649[33] = 0;
   out_2576361439132911649[34] = 0;
   out_2576361439132911649[35] = 0;
   out_2576361439132911649[36] = 0;
   out_2576361439132911649[37] = 0;
   out_2576361439132911649[38] = 0;
   out_2576361439132911649[39] = 0;
   out_2576361439132911649[40] = 1;
   out_2576361439132911649[41] = 0;
   out_2576361439132911649[42] = 0;
   out_2576361439132911649[43] = 0;
   out_2576361439132911649[44] = 0;
   out_2576361439132911649[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2576361439132911649[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2576361439132911649[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2576361439132911649[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2576361439132911649[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2576361439132911649[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2576361439132911649[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2576361439132911649[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2576361439132911649[53] = -9.8000000000000007*dt;
   out_2576361439132911649[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2576361439132911649[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2576361439132911649[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2576361439132911649[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2576361439132911649[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2576361439132911649[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2576361439132911649[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2576361439132911649[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2576361439132911649[62] = 0;
   out_2576361439132911649[63] = 0;
   out_2576361439132911649[64] = 0;
   out_2576361439132911649[65] = 0;
   out_2576361439132911649[66] = 0;
   out_2576361439132911649[67] = 0;
   out_2576361439132911649[68] = 0;
   out_2576361439132911649[69] = 0;
   out_2576361439132911649[70] = 1;
   out_2576361439132911649[71] = 0;
   out_2576361439132911649[72] = 0;
   out_2576361439132911649[73] = 0;
   out_2576361439132911649[74] = 0;
   out_2576361439132911649[75] = 0;
   out_2576361439132911649[76] = 0;
   out_2576361439132911649[77] = 0;
   out_2576361439132911649[78] = 0;
   out_2576361439132911649[79] = 0;
   out_2576361439132911649[80] = 1;
}
void h_25(double *state, double *unused, double *out_2629926194421007623) {
   out_2629926194421007623[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5004037656767691018) {
   out_5004037656767691018[0] = 0;
   out_5004037656767691018[1] = 0;
   out_5004037656767691018[2] = 0;
   out_5004037656767691018[3] = 0;
   out_5004037656767691018[4] = 0;
   out_5004037656767691018[5] = 0;
   out_5004037656767691018[6] = 1;
   out_5004037656767691018[7] = 0;
   out_5004037656767691018[8] = 0;
}
void h_24(double *state, double *unused, double *out_1503054954568587628) {
   out_1503054954568587628[0] = state[4];
   out_1503054954568587628[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1571534149823827083) {
   out_1571534149823827083[0] = 0;
   out_1571534149823827083[1] = 0;
   out_1571534149823827083[2] = 0;
   out_1571534149823827083[3] = 0;
   out_1571534149823827083[4] = 1;
   out_1571534149823827083[5] = 0;
   out_1571534149823827083[6] = 0;
   out_1571534149823827083[7] = 0;
   out_1571534149823827083[8] = 0;
   out_1571534149823827083[9] = 0;
   out_1571534149823827083[10] = 0;
   out_1571534149823827083[11] = 0;
   out_1571534149823827083[12] = 0;
   out_1571534149823827083[13] = 0;
   out_1571534149823827083[14] = 1;
   out_1571534149823827083[15] = 0;
   out_1571534149823827083[16] = 0;
   out_1571534149823827083[17] = 0;
}
void h_30(double *state, double *unused, double *out_8132025166865176366) {
   out_8132025166865176366[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1912652684723925737) {
   out_1912652684723925737[0] = 0;
   out_1912652684723925737[1] = 0;
   out_1912652684723925737[2] = 0;
   out_1912652684723925737[3] = 0;
   out_1912652684723925737[4] = 1;
   out_1912652684723925737[5] = 0;
   out_1912652684723925737[6] = 0;
   out_1912652684723925737[7] = 0;
   out_1912652684723925737[8] = 0;
}
void h_26(double *state, double *unused, double *out_7710313586035677599) {
   out_7710313586035677599[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8745540975641747242) {
   out_8745540975641747242[0] = 0;
   out_8745540975641747242[1] = 0;
   out_8745540975641747242[2] = 0;
   out_8745540975641747242[3] = 0;
   out_8745540975641747242[4] = 0;
   out_8745540975641747242[5] = 0;
   out_8745540975641747242[6] = 0;
   out_8745540975641747242[7] = 1;
   out_8745540975641747242[8] = 0;
}
void h_27(double *state, double *unused, double *out_2371747073239838022) {
   out_2371747073239838022[0] = state[3];
}
void H_27(double *state, double *unused, double *out_262110627076499174) {
   out_262110627076499174[0] = 0;
   out_262110627076499174[1] = 0;
   out_262110627076499174[2] = 0;
   out_262110627076499174[3] = 1;
   out_262110627076499174[4] = 0;
   out_262110627076499174[5] = 0;
   out_262110627076499174[6] = 0;
   out_262110627076499174[7] = 0;
   out_262110627076499174[8] = 0;
}
void h_29(double *state, double *unused, double *out_4399088153110512914) {
   out_4399088153110512914[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1975473353946050207) {
   out_1975473353946050207[0] = 0;
   out_1975473353946050207[1] = 1;
   out_1975473353946050207[2] = 0;
   out_1975473353946050207[3] = 0;
   out_1975473353946050207[4] = 0;
   out_1975473353946050207[5] = 0;
   out_1975473353946050207[6] = 0;
   out_1975473353946050207[7] = 0;
   out_1975473353946050207[8] = 0;
}
void h_28(double *state, double *unused, double *out_1204003272163696285) {
   out_1204003272163696285[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7057872371015580781) {
   out_7057872371015580781[0] = 1;
   out_7057872371015580781[1] = 0;
   out_7057872371015580781[2] = 0;
   out_7057872371015580781[3] = 0;
   out_7057872371015580781[4] = 0;
   out_7057872371015580781[5] = 0;
   out_7057872371015580781[6] = 0;
   out_7057872371015580781[7] = 0;
   out_7057872371015580781[8] = 0;
}
void h_31(double *state, double *unused, double *out_2472739526069878478) {
   out_2472739526069878478[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4973391694890730590) {
   out_4973391694890730590[0] = 0;
   out_4973391694890730590[1] = 0;
   out_4973391694890730590[2] = 0;
   out_4973391694890730590[3] = 0;
   out_4973391694890730590[4] = 0;
   out_4973391694890730590[5] = 0;
   out_4973391694890730590[6] = 0;
   out_4973391694890730590[7] = 0;
   out_4973391694890730590[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2829633900128880649) {
  err_fun(nom_x, delta_x, out_2829633900128880649);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7762599449520589818) {
  inv_err_fun(nom_x, true_x, out_7762599449520589818);
}
void car_H_mod_fun(double *state, double *out_1808600300430226605) {
  H_mod_fun(state, out_1808600300430226605);
}
void car_f_fun(double *state, double dt, double *out_6453172057652161082) {
  f_fun(state,  dt, out_6453172057652161082);
}
void car_F_fun(double *state, double dt, double *out_2576361439132911649) {
  F_fun(state,  dt, out_2576361439132911649);
}
void car_h_25(double *state, double *unused, double *out_2629926194421007623) {
  h_25(state, unused, out_2629926194421007623);
}
void car_H_25(double *state, double *unused, double *out_5004037656767691018) {
  H_25(state, unused, out_5004037656767691018);
}
void car_h_24(double *state, double *unused, double *out_1503054954568587628) {
  h_24(state, unused, out_1503054954568587628);
}
void car_H_24(double *state, double *unused, double *out_1571534149823827083) {
  H_24(state, unused, out_1571534149823827083);
}
void car_h_30(double *state, double *unused, double *out_8132025166865176366) {
  h_30(state, unused, out_8132025166865176366);
}
void car_H_30(double *state, double *unused, double *out_1912652684723925737) {
  H_30(state, unused, out_1912652684723925737);
}
void car_h_26(double *state, double *unused, double *out_7710313586035677599) {
  h_26(state, unused, out_7710313586035677599);
}
void car_H_26(double *state, double *unused, double *out_8745540975641747242) {
  H_26(state, unused, out_8745540975641747242);
}
void car_h_27(double *state, double *unused, double *out_2371747073239838022) {
  h_27(state, unused, out_2371747073239838022);
}
void car_H_27(double *state, double *unused, double *out_262110627076499174) {
  H_27(state, unused, out_262110627076499174);
}
void car_h_29(double *state, double *unused, double *out_4399088153110512914) {
  h_29(state, unused, out_4399088153110512914);
}
void car_H_29(double *state, double *unused, double *out_1975473353946050207) {
  H_29(state, unused, out_1975473353946050207);
}
void car_h_28(double *state, double *unused, double *out_1204003272163696285) {
  h_28(state, unused, out_1204003272163696285);
}
void car_H_28(double *state, double *unused, double *out_7057872371015580781) {
  H_28(state, unused, out_7057872371015580781);
}
void car_h_31(double *state, double *unused, double *out_2472739526069878478) {
  h_31(state, unused, out_2472739526069878478);
}
void car_H_31(double *state, double *unused, double *out_4973391694890730590) {
  H_31(state, unused, out_4973391694890730590);
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
