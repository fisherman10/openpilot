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
void err_fun(double *nom_x, double *delta_x, double *out_706472028070702947) {
   out_706472028070702947[0] = delta_x[0] + nom_x[0];
   out_706472028070702947[1] = delta_x[1] + nom_x[1];
   out_706472028070702947[2] = delta_x[2] + nom_x[2];
   out_706472028070702947[3] = delta_x[3] + nom_x[3];
   out_706472028070702947[4] = delta_x[4] + nom_x[4];
   out_706472028070702947[5] = delta_x[5] + nom_x[5];
   out_706472028070702947[6] = delta_x[6] + nom_x[6];
   out_706472028070702947[7] = delta_x[7] + nom_x[7];
   out_706472028070702947[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4384673494611099764) {
   out_4384673494611099764[0] = -nom_x[0] + true_x[0];
   out_4384673494611099764[1] = -nom_x[1] + true_x[1];
   out_4384673494611099764[2] = -nom_x[2] + true_x[2];
   out_4384673494611099764[3] = -nom_x[3] + true_x[3];
   out_4384673494611099764[4] = -nom_x[4] + true_x[4];
   out_4384673494611099764[5] = -nom_x[5] + true_x[5];
   out_4384673494611099764[6] = -nom_x[6] + true_x[6];
   out_4384673494611099764[7] = -nom_x[7] + true_x[7];
   out_4384673494611099764[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_399698239415973843) {
   out_399698239415973843[0] = 1.0;
   out_399698239415973843[1] = 0;
   out_399698239415973843[2] = 0;
   out_399698239415973843[3] = 0;
   out_399698239415973843[4] = 0;
   out_399698239415973843[5] = 0;
   out_399698239415973843[6] = 0;
   out_399698239415973843[7] = 0;
   out_399698239415973843[8] = 0;
   out_399698239415973843[9] = 0;
   out_399698239415973843[10] = 1.0;
   out_399698239415973843[11] = 0;
   out_399698239415973843[12] = 0;
   out_399698239415973843[13] = 0;
   out_399698239415973843[14] = 0;
   out_399698239415973843[15] = 0;
   out_399698239415973843[16] = 0;
   out_399698239415973843[17] = 0;
   out_399698239415973843[18] = 0;
   out_399698239415973843[19] = 0;
   out_399698239415973843[20] = 1.0;
   out_399698239415973843[21] = 0;
   out_399698239415973843[22] = 0;
   out_399698239415973843[23] = 0;
   out_399698239415973843[24] = 0;
   out_399698239415973843[25] = 0;
   out_399698239415973843[26] = 0;
   out_399698239415973843[27] = 0;
   out_399698239415973843[28] = 0;
   out_399698239415973843[29] = 0;
   out_399698239415973843[30] = 1.0;
   out_399698239415973843[31] = 0;
   out_399698239415973843[32] = 0;
   out_399698239415973843[33] = 0;
   out_399698239415973843[34] = 0;
   out_399698239415973843[35] = 0;
   out_399698239415973843[36] = 0;
   out_399698239415973843[37] = 0;
   out_399698239415973843[38] = 0;
   out_399698239415973843[39] = 0;
   out_399698239415973843[40] = 1.0;
   out_399698239415973843[41] = 0;
   out_399698239415973843[42] = 0;
   out_399698239415973843[43] = 0;
   out_399698239415973843[44] = 0;
   out_399698239415973843[45] = 0;
   out_399698239415973843[46] = 0;
   out_399698239415973843[47] = 0;
   out_399698239415973843[48] = 0;
   out_399698239415973843[49] = 0;
   out_399698239415973843[50] = 1.0;
   out_399698239415973843[51] = 0;
   out_399698239415973843[52] = 0;
   out_399698239415973843[53] = 0;
   out_399698239415973843[54] = 0;
   out_399698239415973843[55] = 0;
   out_399698239415973843[56] = 0;
   out_399698239415973843[57] = 0;
   out_399698239415973843[58] = 0;
   out_399698239415973843[59] = 0;
   out_399698239415973843[60] = 1.0;
   out_399698239415973843[61] = 0;
   out_399698239415973843[62] = 0;
   out_399698239415973843[63] = 0;
   out_399698239415973843[64] = 0;
   out_399698239415973843[65] = 0;
   out_399698239415973843[66] = 0;
   out_399698239415973843[67] = 0;
   out_399698239415973843[68] = 0;
   out_399698239415973843[69] = 0;
   out_399698239415973843[70] = 1.0;
   out_399698239415973843[71] = 0;
   out_399698239415973843[72] = 0;
   out_399698239415973843[73] = 0;
   out_399698239415973843[74] = 0;
   out_399698239415973843[75] = 0;
   out_399698239415973843[76] = 0;
   out_399698239415973843[77] = 0;
   out_399698239415973843[78] = 0;
   out_399698239415973843[79] = 0;
   out_399698239415973843[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_764333901801294821) {
   out_764333901801294821[0] = state[0];
   out_764333901801294821[1] = state[1];
   out_764333901801294821[2] = state[2];
   out_764333901801294821[3] = state[3];
   out_764333901801294821[4] = state[4];
   out_764333901801294821[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_764333901801294821[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_764333901801294821[7] = state[7];
   out_764333901801294821[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3701654446068242986) {
   out_3701654446068242986[0] = 1;
   out_3701654446068242986[1] = 0;
   out_3701654446068242986[2] = 0;
   out_3701654446068242986[3] = 0;
   out_3701654446068242986[4] = 0;
   out_3701654446068242986[5] = 0;
   out_3701654446068242986[6] = 0;
   out_3701654446068242986[7] = 0;
   out_3701654446068242986[8] = 0;
   out_3701654446068242986[9] = 0;
   out_3701654446068242986[10] = 1;
   out_3701654446068242986[11] = 0;
   out_3701654446068242986[12] = 0;
   out_3701654446068242986[13] = 0;
   out_3701654446068242986[14] = 0;
   out_3701654446068242986[15] = 0;
   out_3701654446068242986[16] = 0;
   out_3701654446068242986[17] = 0;
   out_3701654446068242986[18] = 0;
   out_3701654446068242986[19] = 0;
   out_3701654446068242986[20] = 1;
   out_3701654446068242986[21] = 0;
   out_3701654446068242986[22] = 0;
   out_3701654446068242986[23] = 0;
   out_3701654446068242986[24] = 0;
   out_3701654446068242986[25] = 0;
   out_3701654446068242986[26] = 0;
   out_3701654446068242986[27] = 0;
   out_3701654446068242986[28] = 0;
   out_3701654446068242986[29] = 0;
   out_3701654446068242986[30] = 1;
   out_3701654446068242986[31] = 0;
   out_3701654446068242986[32] = 0;
   out_3701654446068242986[33] = 0;
   out_3701654446068242986[34] = 0;
   out_3701654446068242986[35] = 0;
   out_3701654446068242986[36] = 0;
   out_3701654446068242986[37] = 0;
   out_3701654446068242986[38] = 0;
   out_3701654446068242986[39] = 0;
   out_3701654446068242986[40] = 1;
   out_3701654446068242986[41] = 0;
   out_3701654446068242986[42] = 0;
   out_3701654446068242986[43] = 0;
   out_3701654446068242986[44] = 0;
   out_3701654446068242986[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3701654446068242986[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3701654446068242986[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3701654446068242986[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3701654446068242986[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3701654446068242986[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3701654446068242986[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3701654446068242986[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3701654446068242986[53] = -9.8000000000000007*dt;
   out_3701654446068242986[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3701654446068242986[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3701654446068242986[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3701654446068242986[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3701654446068242986[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3701654446068242986[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3701654446068242986[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3701654446068242986[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3701654446068242986[62] = 0;
   out_3701654446068242986[63] = 0;
   out_3701654446068242986[64] = 0;
   out_3701654446068242986[65] = 0;
   out_3701654446068242986[66] = 0;
   out_3701654446068242986[67] = 0;
   out_3701654446068242986[68] = 0;
   out_3701654446068242986[69] = 0;
   out_3701654446068242986[70] = 1;
   out_3701654446068242986[71] = 0;
   out_3701654446068242986[72] = 0;
   out_3701654446068242986[73] = 0;
   out_3701654446068242986[74] = 0;
   out_3701654446068242986[75] = 0;
   out_3701654446068242986[76] = 0;
   out_3701654446068242986[77] = 0;
   out_3701654446068242986[78] = 0;
   out_3701654446068242986[79] = 0;
   out_3701654446068242986[80] = 1;
}
void h_25(double *state, double *unused, double *out_6301433132971002834) {
   out_6301433132971002834[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7433832135825478518) {
   out_7433832135825478518[0] = 0;
   out_7433832135825478518[1] = 0;
   out_7433832135825478518[2] = 0;
   out_7433832135825478518[3] = 0;
   out_7433832135825478518[4] = 0;
   out_7433832135825478518[5] = 0;
   out_7433832135825478518[6] = 1;
   out_7433832135825478518[7] = 0;
   out_7433832135825478518[8] = 0;
}
void h_24(double *state, double *unused, double *out_3213610399440082051) {
   out_3213610399440082051[0] = state[4];
   out_3213610399440082051[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4437340131292554997) {
   out_4437340131292554997[0] = 0;
   out_4437340131292554997[1] = 0;
   out_4437340131292554997[2] = 0;
   out_4437340131292554997[3] = 0;
   out_4437340131292554997[4] = 1;
   out_4437340131292554997[5] = 0;
   out_4437340131292554997[6] = 0;
   out_4437340131292554997[7] = 0;
   out_4437340131292554997[8] = 0;
   out_4437340131292554997[9] = 0;
   out_4437340131292554997[10] = 0;
   out_4437340131292554997[11] = 0;
   out_4437340131292554997[12] = 0;
   out_4437340131292554997[13] = 0;
   out_4437340131292554997[14] = 1;
   out_4437340131292554997[15] = 0;
   out_4437340131292554997[16] = 0;
   out_4437340131292554997[17] = 0;
}
void h_30(double *state, double *unused, double *out_2836284151712848348) {
   out_2836284151712848348[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4096221596392456343) {
   out_4096221596392456343[0] = 0;
   out_4096221596392456343[1] = 0;
   out_4096221596392456343[2] = 0;
   out_4096221596392456343[3] = 0;
   out_4096221596392456343[4] = 1;
   out_4096221596392456343[5] = 0;
   out_4096221596392456343[6] = 0;
   out_4096221596392456343[7] = 0;
   out_4096221596392456343[8] = 0;
}
void h_26(double *state, double *unused, double *out_3320528437082837162) {
   out_3320528437082837162[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3692328816951422294) {
   out_3692328816951422294[0] = 0;
   out_3692328816951422294[1] = 0;
   out_3692328816951422294[2] = 0;
   out_3692328816951422294[3] = 0;
   out_3692328816951422294[4] = 0;
   out_3692328816951422294[5] = 0;
   out_3692328816951422294[6] = 0;
   out_3692328816951422294[7] = 1;
   out_3692328816951422294[8] = 0;
}
void h_27(double *state, double *unused, double *out_7143637673077703137) {
   out_7143637673077703137[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6270984908192881254) {
   out_6270984908192881254[0] = 0;
   out_6270984908192881254[1] = 0;
   out_6270984908192881254[2] = 0;
   out_6270984908192881254[3] = 1;
   out_6270984908192881254[4] = 0;
   out_6270984908192881254[5] = 0;
   out_6270984908192881254[6] = 0;
   out_6270984908192881254[7] = 0;
   out_6270984908192881254[8] = 0;
}
void h_29(double *state, double *unused, double *out_7300824341428832282) {
   out_7300824341428832282[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3585990252078064159) {
   out_3585990252078064159[0] = 0;
   out_3585990252078064159[1] = 1;
   out_3585990252078064159[2] = 0;
   out_3585990252078064159[3] = 0;
   out_3585990252078064159[4] = 0;
   out_3585990252078064159[5] = 0;
   out_3585990252078064159[6] = 0;
   out_3585990252078064159[7] = 0;
   out_3585990252078064159[8] = 0;
}
void h_28(double *state, double *unused, double *out_8266320349187515794) {
   out_8266320349187515794[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8668389269147594733) {
   out_8668389269147594733[0] = 1;
   out_8668389269147594733[1] = 0;
   out_8668389269147594733[2] = 0;
   out_8668389269147594733[3] = 0;
   out_8668389269147594733[4] = 0;
   out_8668389269147594733[5] = 0;
   out_8668389269147594733[6] = 0;
   out_8668389269147594733[7] = 0;
   out_8668389269147594733[8] = 0;
}
void h_31(double *state, double *unused, double *out_5385182467266385755) {
   out_5385182467266385755[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7464478097702438946) {
   out_7464478097702438946[0] = 0;
   out_7464478097702438946[1] = 0;
   out_7464478097702438946[2] = 0;
   out_7464478097702438946[3] = 0;
   out_7464478097702438946[4] = 0;
   out_7464478097702438946[5] = 0;
   out_7464478097702438946[6] = 0;
   out_7464478097702438946[7] = 0;
   out_7464478097702438946[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_706472028070702947) {
  err_fun(nom_x, delta_x, out_706472028070702947);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4384673494611099764) {
  inv_err_fun(nom_x, true_x, out_4384673494611099764);
}
void car_H_mod_fun(double *state, double *out_399698239415973843) {
  H_mod_fun(state, out_399698239415973843);
}
void car_f_fun(double *state, double dt, double *out_764333901801294821) {
  f_fun(state,  dt, out_764333901801294821);
}
void car_F_fun(double *state, double dt, double *out_3701654446068242986) {
  F_fun(state,  dt, out_3701654446068242986);
}
void car_h_25(double *state, double *unused, double *out_6301433132971002834) {
  h_25(state, unused, out_6301433132971002834);
}
void car_H_25(double *state, double *unused, double *out_7433832135825478518) {
  H_25(state, unused, out_7433832135825478518);
}
void car_h_24(double *state, double *unused, double *out_3213610399440082051) {
  h_24(state, unused, out_3213610399440082051);
}
void car_H_24(double *state, double *unused, double *out_4437340131292554997) {
  H_24(state, unused, out_4437340131292554997);
}
void car_h_30(double *state, double *unused, double *out_2836284151712848348) {
  h_30(state, unused, out_2836284151712848348);
}
void car_H_30(double *state, double *unused, double *out_4096221596392456343) {
  H_30(state, unused, out_4096221596392456343);
}
void car_h_26(double *state, double *unused, double *out_3320528437082837162) {
  h_26(state, unused, out_3320528437082837162);
}
void car_H_26(double *state, double *unused, double *out_3692328816951422294) {
  H_26(state, unused, out_3692328816951422294);
}
void car_h_27(double *state, double *unused, double *out_7143637673077703137) {
  h_27(state, unused, out_7143637673077703137);
}
void car_H_27(double *state, double *unused, double *out_6270984908192881254) {
  H_27(state, unused, out_6270984908192881254);
}
void car_h_29(double *state, double *unused, double *out_7300824341428832282) {
  h_29(state, unused, out_7300824341428832282);
}
void car_H_29(double *state, double *unused, double *out_3585990252078064159) {
  H_29(state, unused, out_3585990252078064159);
}
void car_h_28(double *state, double *unused, double *out_8266320349187515794) {
  h_28(state, unused, out_8266320349187515794);
}
void car_H_28(double *state, double *unused, double *out_8668389269147594733) {
  H_28(state, unused, out_8668389269147594733);
}
void car_h_31(double *state, double *unused, double *out_5385182467266385755) {
  h_31(state, unused, out_5385182467266385755);
}
void car_H_31(double *state, double *unused, double *out_7464478097702438946) {
  H_31(state, unused, out_7464478097702438946);
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
