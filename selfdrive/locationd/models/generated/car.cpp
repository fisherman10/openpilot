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
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8009260081015432478) {
   out_8009260081015432478[0] = delta_x[0] + nom_x[0];
   out_8009260081015432478[1] = delta_x[1] + nom_x[1];
   out_8009260081015432478[2] = delta_x[2] + nom_x[2];
   out_8009260081015432478[3] = delta_x[3] + nom_x[3];
   out_8009260081015432478[4] = delta_x[4] + nom_x[4];
   out_8009260081015432478[5] = delta_x[5] + nom_x[5];
   out_8009260081015432478[6] = delta_x[6] + nom_x[6];
   out_8009260081015432478[7] = delta_x[7] + nom_x[7];
   out_8009260081015432478[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_652454977147307792) {
   out_652454977147307792[0] = -nom_x[0] + true_x[0];
   out_652454977147307792[1] = -nom_x[1] + true_x[1];
   out_652454977147307792[2] = -nom_x[2] + true_x[2];
   out_652454977147307792[3] = -nom_x[3] + true_x[3];
   out_652454977147307792[4] = -nom_x[4] + true_x[4];
   out_652454977147307792[5] = -nom_x[5] + true_x[5];
   out_652454977147307792[6] = -nom_x[6] + true_x[6];
   out_652454977147307792[7] = -nom_x[7] + true_x[7];
   out_652454977147307792[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1090209382812354337) {
   out_1090209382812354337[0] = 1.0;
   out_1090209382812354337[1] = 0.0;
   out_1090209382812354337[2] = 0.0;
   out_1090209382812354337[3] = 0.0;
   out_1090209382812354337[4] = 0.0;
   out_1090209382812354337[5] = 0.0;
   out_1090209382812354337[6] = 0.0;
   out_1090209382812354337[7] = 0.0;
   out_1090209382812354337[8] = 0.0;
   out_1090209382812354337[9] = 0.0;
   out_1090209382812354337[10] = 1.0;
   out_1090209382812354337[11] = 0.0;
   out_1090209382812354337[12] = 0.0;
   out_1090209382812354337[13] = 0.0;
   out_1090209382812354337[14] = 0.0;
   out_1090209382812354337[15] = 0.0;
   out_1090209382812354337[16] = 0.0;
   out_1090209382812354337[17] = 0.0;
   out_1090209382812354337[18] = 0.0;
   out_1090209382812354337[19] = 0.0;
   out_1090209382812354337[20] = 1.0;
   out_1090209382812354337[21] = 0.0;
   out_1090209382812354337[22] = 0.0;
   out_1090209382812354337[23] = 0.0;
   out_1090209382812354337[24] = 0.0;
   out_1090209382812354337[25] = 0.0;
   out_1090209382812354337[26] = 0.0;
   out_1090209382812354337[27] = 0.0;
   out_1090209382812354337[28] = 0.0;
   out_1090209382812354337[29] = 0.0;
   out_1090209382812354337[30] = 1.0;
   out_1090209382812354337[31] = 0.0;
   out_1090209382812354337[32] = 0.0;
   out_1090209382812354337[33] = 0.0;
   out_1090209382812354337[34] = 0.0;
   out_1090209382812354337[35] = 0.0;
   out_1090209382812354337[36] = 0.0;
   out_1090209382812354337[37] = 0.0;
   out_1090209382812354337[38] = 0.0;
   out_1090209382812354337[39] = 0.0;
   out_1090209382812354337[40] = 1.0;
   out_1090209382812354337[41] = 0.0;
   out_1090209382812354337[42] = 0.0;
   out_1090209382812354337[43] = 0.0;
   out_1090209382812354337[44] = 0.0;
   out_1090209382812354337[45] = 0.0;
   out_1090209382812354337[46] = 0.0;
   out_1090209382812354337[47] = 0.0;
   out_1090209382812354337[48] = 0.0;
   out_1090209382812354337[49] = 0.0;
   out_1090209382812354337[50] = 1.0;
   out_1090209382812354337[51] = 0.0;
   out_1090209382812354337[52] = 0.0;
   out_1090209382812354337[53] = 0.0;
   out_1090209382812354337[54] = 0.0;
   out_1090209382812354337[55] = 0.0;
   out_1090209382812354337[56] = 0.0;
   out_1090209382812354337[57] = 0.0;
   out_1090209382812354337[58] = 0.0;
   out_1090209382812354337[59] = 0.0;
   out_1090209382812354337[60] = 1.0;
   out_1090209382812354337[61] = 0.0;
   out_1090209382812354337[62] = 0.0;
   out_1090209382812354337[63] = 0.0;
   out_1090209382812354337[64] = 0.0;
   out_1090209382812354337[65] = 0.0;
   out_1090209382812354337[66] = 0.0;
   out_1090209382812354337[67] = 0.0;
   out_1090209382812354337[68] = 0.0;
   out_1090209382812354337[69] = 0.0;
   out_1090209382812354337[70] = 1.0;
   out_1090209382812354337[71] = 0.0;
   out_1090209382812354337[72] = 0.0;
   out_1090209382812354337[73] = 0.0;
   out_1090209382812354337[74] = 0.0;
   out_1090209382812354337[75] = 0.0;
   out_1090209382812354337[76] = 0.0;
   out_1090209382812354337[77] = 0.0;
   out_1090209382812354337[78] = 0.0;
   out_1090209382812354337[79] = 0.0;
   out_1090209382812354337[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2878359103814338252) {
   out_2878359103814338252[0] = state[0];
   out_2878359103814338252[1] = state[1];
   out_2878359103814338252[2] = state[2];
   out_2878359103814338252[3] = state[3];
   out_2878359103814338252[4] = state[4];
   out_2878359103814338252[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2878359103814338252[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2878359103814338252[7] = state[7];
   out_2878359103814338252[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7685646572396440351) {
   out_7685646572396440351[0] = 1;
   out_7685646572396440351[1] = 0;
   out_7685646572396440351[2] = 0;
   out_7685646572396440351[3] = 0;
   out_7685646572396440351[4] = 0;
   out_7685646572396440351[5] = 0;
   out_7685646572396440351[6] = 0;
   out_7685646572396440351[7] = 0;
   out_7685646572396440351[8] = 0;
   out_7685646572396440351[9] = 0;
   out_7685646572396440351[10] = 1;
   out_7685646572396440351[11] = 0;
   out_7685646572396440351[12] = 0;
   out_7685646572396440351[13] = 0;
   out_7685646572396440351[14] = 0;
   out_7685646572396440351[15] = 0;
   out_7685646572396440351[16] = 0;
   out_7685646572396440351[17] = 0;
   out_7685646572396440351[18] = 0;
   out_7685646572396440351[19] = 0;
   out_7685646572396440351[20] = 1;
   out_7685646572396440351[21] = 0;
   out_7685646572396440351[22] = 0;
   out_7685646572396440351[23] = 0;
   out_7685646572396440351[24] = 0;
   out_7685646572396440351[25] = 0;
   out_7685646572396440351[26] = 0;
   out_7685646572396440351[27] = 0;
   out_7685646572396440351[28] = 0;
   out_7685646572396440351[29] = 0;
   out_7685646572396440351[30] = 1;
   out_7685646572396440351[31] = 0;
   out_7685646572396440351[32] = 0;
   out_7685646572396440351[33] = 0;
   out_7685646572396440351[34] = 0;
   out_7685646572396440351[35] = 0;
   out_7685646572396440351[36] = 0;
   out_7685646572396440351[37] = 0;
   out_7685646572396440351[38] = 0;
   out_7685646572396440351[39] = 0;
   out_7685646572396440351[40] = 1;
   out_7685646572396440351[41] = 0;
   out_7685646572396440351[42] = 0;
   out_7685646572396440351[43] = 0;
   out_7685646572396440351[44] = 0;
   out_7685646572396440351[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7685646572396440351[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7685646572396440351[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7685646572396440351[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7685646572396440351[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7685646572396440351[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7685646572396440351[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7685646572396440351[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7685646572396440351[53] = -9.8000000000000007*dt;
   out_7685646572396440351[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7685646572396440351[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7685646572396440351[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7685646572396440351[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7685646572396440351[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7685646572396440351[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7685646572396440351[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7685646572396440351[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7685646572396440351[62] = 0;
   out_7685646572396440351[63] = 0;
   out_7685646572396440351[64] = 0;
   out_7685646572396440351[65] = 0;
   out_7685646572396440351[66] = 0;
   out_7685646572396440351[67] = 0;
   out_7685646572396440351[68] = 0;
   out_7685646572396440351[69] = 0;
   out_7685646572396440351[70] = 1;
   out_7685646572396440351[71] = 0;
   out_7685646572396440351[72] = 0;
   out_7685646572396440351[73] = 0;
   out_7685646572396440351[74] = 0;
   out_7685646572396440351[75] = 0;
   out_7685646572396440351[76] = 0;
   out_7685646572396440351[77] = 0;
   out_7685646572396440351[78] = 0;
   out_7685646572396440351[79] = 0;
   out_7685646572396440351[80] = 1;
}
void h_25(double *state, double *unused, double *out_3524389857431468744) {
   out_3524389857431468744[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8102978843099882900) {
   out_8102978843099882900[0] = 0;
   out_8102978843099882900[1] = 0;
   out_8102978843099882900[2] = 0;
   out_8102978843099882900[3] = 0;
   out_8102978843099882900[4] = 0;
   out_8102978843099882900[5] = 0;
   out_8102978843099882900[6] = 1;
   out_8102978843099882900[7] = 0;
   out_8102978843099882900[8] = 0;
}
void h_24(double *state, double *unused, double *out_7474005308437218817) {
   out_7474005308437218817[0] = state[4];
   out_7474005308437218817[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1125086342969312325) {
   out_1125086342969312325[0] = 0;
   out_1125086342969312325[1] = 0;
   out_1125086342969312325[2] = 0;
   out_1125086342969312325[3] = 0;
   out_1125086342969312325[4] = 1;
   out_1125086342969312325[5] = 0;
   out_1125086342969312325[6] = 0;
   out_1125086342969312325[7] = 0;
   out_1125086342969312325[8] = 0;
   out_1125086342969312325[9] = 0;
   out_1125086342969312325[10] = 0;
   out_1125086342969312325[11] = 0;
   out_1125086342969312325[12] = 0;
   out_1125086342969312325[13] = 0;
   out_1125086342969312325[14] = 1;
   out_1125086342969312325[15] = 0;
   out_1125086342969312325[16] = 0;
   out_1125086342969312325[17] = 0;
}
void h_30(double *state, double *unused, double *out_3681576525782597889) {
   out_3681576525782597889[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5816068900482060518) {
   out_5816068900482060518[0] = 0;
   out_5816068900482060518[1] = 0;
   out_5816068900482060518[2] = 0;
   out_5816068900482060518[3] = 0;
   out_5816068900482060518[4] = 1;
   out_5816068900482060518[5] = 0;
   out_5816068900482060518[6] = 0;
   out_5816068900482060518[7] = 0;
   out_5816068900482060518[8] = 0;
}
void h_26(double *state, double *unused, double *out_9095028842829394371) {
   out_9095028842829394371[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6602261911735612492) {
   out_6602261911735612492[0] = 0;
   out_6602261911735612492[1] = 0;
   out_6602261911735612492[2] = 0;
   out_6602261911735612492[3] = 0;
   out_6602261911735612492[4] = 0;
   out_6602261911735612492[5] = 0;
   out_6602261911735612492[6] = 0;
   out_6602261911735612492[7] = 1;
   out_6602261911735612492[8] = 0;
}
void h_27(double *state, double *unused, double *out_6561175908875801429) {
   out_6561175908875801429[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8039662971666003735) {
   out_8039662971666003735[0] = 0;
   out_8039662971666003735[1] = 0;
   out_8039662971666003735[2] = 0;
   out_8039662971666003735[3] = 1;
   out_8039662971666003735[4] = 0;
   out_8039662971666003735[5] = 0;
   out_8039662971666003735[6] = 0;
   out_8039662971666003735[7] = 0;
   out_8039662971666003735[8] = 0;
}
void h_29(double *state, double *unused, double *out_5114732938483399251) {
   out_5114732938483399251[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6326300244796452702) {
   out_6326300244796452702[0] = 0;
   out_6326300244796452702[1] = 1;
   out_6326300244796452702[2] = 0;
   out_6326300244796452702[3] = 0;
   out_6326300244796452702[4] = 0;
   out_6326300244796452702[5] = 0;
   out_6326300244796452702[6] = 0;
   out_6326300244796452702[7] = 0;
   out_6326300244796452702[8] = 0;
}
void h_28(double *state, double *unused, double *out_1559502641214955784) {
   out_1559502641214955784[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1243901227726922128) {
   out_1243901227726922128[0] = 1;
   out_1243901227726922128[1] = 0;
   out_1243901227726922128[2] = 0;
   out_1243901227726922128[3] = 0;
   out_1243901227726922128[4] = 0;
   out_1243901227726922128[5] = 0;
   out_1243901227726922128[6] = 0;
   out_1243901227726922128[7] = 0;
   out_1243901227726922128[8] = 0;
}
void h_31(double *state, double *unused, double *out_3799583919715974633) {
   out_3799583919715974633[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8072332881222922472) {
   out_8072332881222922472[0] = 0;
   out_8072332881222922472[1] = 0;
   out_8072332881222922472[2] = 0;
   out_8072332881222922472[3] = 0;
   out_8072332881222922472[4] = 0;
   out_8072332881222922472[5] = 0;
   out_8072332881222922472[6] = 0;
   out_8072332881222922472[7] = 0;
   out_8072332881222922472[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8009260081015432478) {
  err_fun(nom_x, delta_x, out_8009260081015432478);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_652454977147307792) {
  inv_err_fun(nom_x, true_x, out_652454977147307792);
}
void car_H_mod_fun(double *state, double *out_1090209382812354337) {
  H_mod_fun(state, out_1090209382812354337);
}
void car_f_fun(double *state, double dt, double *out_2878359103814338252) {
  f_fun(state,  dt, out_2878359103814338252);
}
void car_F_fun(double *state, double dt, double *out_7685646572396440351) {
  F_fun(state,  dt, out_7685646572396440351);
}
void car_h_25(double *state, double *unused, double *out_3524389857431468744) {
  h_25(state, unused, out_3524389857431468744);
}
void car_H_25(double *state, double *unused, double *out_8102978843099882900) {
  H_25(state, unused, out_8102978843099882900);
}
void car_h_24(double *state, double *unused, double *out_7474005308437218817) {
  h_24(state, unused, out_7474005308437218817);
}
void car_H_24(double *state, double *unused, double *out_1125086342969312325) {
  H_24(state, unused, out_1125086342969312325);
}
void car_h_30(double *state, double *unused, double *out_3681576525782597889) {
  h_30(state, unused, out_3681576525782597889);
}
void car_H_30(double *state, double *unused, double *out_5816068900482060518) {
  H_30(state, unused, out_5816068900482060518);
}
void car_h_26(double *state, double *unused, double *out_9095028842829394371) {
  h_26(state, unused, out_9095028842829394371);
}
void car_H_26(double *state, double *unused, double *out_6602261911735612492) {
  H_26(state, unused, out_6602261911735612492);
}
void car_h_27(double *state, double *unused, double *out_6561175908875801429) {
  h_27(state, unused, out_6561175908875801429);
}
void car_H_27(double *state, double *unused, double *out_8039662971666003735) {
  H_27(state, unused, out_8039662971666003735);
}
void car_h_29(double *state, double *unused, double *out_5114732938483399251) {
  h_29(state, unused, out_5114732938483399251);
}
void car_H_29(double *state, double *unused, double *out_6326300244796452702) {
  H_29(state, unused, out_6326300244796452702);
}
void car_h_28(double *state, double *unused, double *out_1559502641214955784) {
  h_28(state, unused, out_1559502641214955784);
}
void car_H_28(double *state, double *unused, double *out_1243901227726922128) {
  H_28(state, unused, out_1243901227726922128);
}
void car_h_31(double *state, double *unused, double *out_3799583919715974633) {
  h_31(state, unused, out_3799583919715974633);
}
void car_H_31(double *state, double *unused, double *out_8072332881222922472) {
  H_31(state, unused, out_8072332881222922472);
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

ekf_lib_init(car)
