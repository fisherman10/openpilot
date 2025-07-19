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
void err_fun(double *nom_x, double *delta_x, double *out_897924418877243906) {
   out_897924418877243906[0] = delta_x[0] + nom_x[0];
   out_897924418877243906[1] = delta_x[1] + nom_x[1];
   out_897924418877243906[2] = delta_x[2] + nom_x[2];
   out_897924418877243906[3] = delta_x[3] + nom_x[3];
   out_897924418877243906[4] = delta_x[4] + nom_x[4];
   out_897924418877243906[5] = delta_x[5] + nom_x[5];
   out_897924418877243906[6] = delta_x[6] + nom_x[6];
   out_897924418877243906[7] = delta_x[7] + nom_x[7];
   out_897924418877243906[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3636397663220992824) {
   out_3636397663220992824[0] = -nom_x[0] + true_x[0];
   out_3636397663220992824[1] = -nom_x[1] + true_x[1];
   out_3636397663220992824[2] = -nom_x[2] + true_x[2];
   out_3636397663220992824[3] = -nom_x[3] + true_x[3];
   out_3636397663220992824[4] = -nom_x[4] + true_x[4];
   out_3636397663220992824[5] = -nom_x[5] + true_x[5];
   out_3636397663220992824[6] = -nom_x[6] + true_x[6];
   out_3636397663220992824[7] = -nom_x[7] + true_x[7];
   out_3636397663220992824[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4379404279019457682) {
   out_4379404279019457682[0] = 1.0;
   out_4379404279019457682[1] = 0.0;
   out_4379404279019457682[2] = 0.0;
   out_4379404279019457682[3] = 0.0;
   out_4379404279019457682[4] = 0.0;
   out_4379404279019457682[5] = 0.0;
   out_4379404279019457682[6] = 0.0;
   out_4379404279019457682[7] = 0.0;
   out_4379404279019457682[8] = 0.0;
   out_4379404279019457682[9] = 0.0;
   out_4379404279019457682[10] = 1.0;
   out_4379404279019457682[11] = 0.0;
   out_4379404279019457682[12] = 0.0;
   out_4379404279019457682[13] = 0.0;
   out_4379404279019457682[14] = 0.0;
   out_4379404279019457682[15] = 0.0;
   out_4379404279019457682[16] = 0.0;
   out_4379404279019457682[17] = 0.0;
   out_4379404279019457682[18] = 0.0;
   out_4379404279019457682[19] = 0.0;
   out_4379404279019457682[20] = 1.0;
   out_4379404279019457682[21] = 0.0;
   out_4379404279019457682[22] = 0.0;
   out_4379404279019457682[23] = 0.0;
   out_4379404279019457682[24] = 0.0;
   out_4379404279019457682[25] = 0.0;
   out_4379404279019457682[26] = 0.0;
   out_4379404279019457682[27] = 0.0;
   out_4379404279019457682[28] = 0.0;
   out_4379404279019457682[29] = 0.0;
   out_4379404279019457682[30] = 1.0;
   out_4379404279019457682[31] = 0.0;
   out_4379404279019457682[32] = 0.0;
   out_4379404279019457682[33] = 0.0;
   out_4379404279019457682[34] = 0.0;
   out_4379404279019457682[35] = 0.0;
   out_4379404279019457682[36] = 0.0;
   out_4379404279019457682[37] = 0.0;
   out_4379404279019457682[38] = 0.0;
   out_4379404279019457682[39] = 0.0;
   out_4379404279019457682[40] = 1.0;
   out_4379404279019457682[41] = 0.0;
   out_4379404279019457682[42] = 0.0;
   out_4379404279019457682[43] = 0.0;
   out_4379404279019457682[44] = 0.0;
   out_4379404279019457682[45] = 0.0;
   out_4379404279019457682[46] = 0.0;
   out_4379404279019457682[47] = 0.0;
   out_4379404279019457682[48] = 0.0;
   out_4379404279019457682[49] = 0.0;
   out_4379404279019457682[50] = 1.0;
   out_4379404279019457682[51] = 0.0;
   out_4379404279019457682[52] = 0.0;
   out_4379404279019457682[53] = 0.0;
   out_4379404279019457682[54] = 0.0;
   out_4379404279019457682[55] = 0.0;
   out_4379404279019457682[56] = 0.0;
   out_4379404279019457682[57] = 0.0;
   out_4379404279019457682[58] = 0.0;
   out_4379404279019457682[59] = 0.0;
   out_4379404279019457682[60] = 1.0;
   out_4379404279019457682[61] = 0.0;
   out_4379404279019457682[62] = 0.0;
   out_4379404279019457682[63] = 0.0;
   out_4379404279019457682[64] = 0.0;
   out_4379404279019457682[65] = 0.0;
   out_4379404279019457682[66] = 0.0;
   out_4379404279019457682[67] = 0.0;
   out_4379404279019457682[68] = 0.0;
   out_4379404279019457682[69] = 0.0;
   out_4379404279019457682[70] = 1.0;
   out_4379404279019457682[71] = 0.0;
   out_4379404279019457682[72] = 0.0;
   out_4379404279019457682[73] = 0.0;
   out_4379404279019457682[74] = 0.0;
   out_4379404279019457682[75] = 0.0;
   out_4379404279019457682[76] = 0.0;
   out_4379404279019457682[77] = 0.0;
   out_4379404279019457682[78] = 0.0;
   out_4379404279019457682[79] = 0.0;
   out_4379404279019457682[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5495334738527013555) {
   out_5495334738527013555[0] = state[0];
   out_5495334738527013555[1] = state[1];
   out_5495334738527013555[2] = state[2];
   out_5495334738527013555[3] = state[3];
   out_5495334738527013555[4] = state[4];
   out_5495334738527013555[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5495334738527013555[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5495334738527013555[7] = state[7];
   out_5495334738527013555[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6163853603347526612) {
   out_6163853603347526612[0] = 1;
   out_6163853603347526612[1] = 0;
   out_6163853603347526612[2] = 0;
   out_6163853603347526612[3] = 0;
   out_6163853603347526612[4] = 0;
   out_6163853603347526612[5] = 0;
   out_6163853603347526612[6] = 0;
   out_6163853603347526612[7] = 0;
   out_6163853603347526612[8] = 0;
   out_6163853603347526612[9] = 0;
   out_6163853603347526612[10] = 1;
   out_6163853603347526612[11] = 0;
   out_6163853603347526612[12] = 0;
   out_6163853603347526612[13] = 0;
   out_6163853603347526612[14] = 0;
   out_6163853603347526612[15] = 0;
   out_6163853603347526612[16] = 0;
   out_6163853603347526612[17] = 0;
   out_6163853603347526612[18] = 0;
   out_6163853603347526612[19] = 0;
   out_6163853603347526612[20] = 1;
   out_6163853603347526612[21] = 0;
   out_6163853603347526612[22] = 0;
   out_6163853603347526612[23] = 0;
   out_6163853603347526612[24] = 0;
   out_6163853603347526612[25] = 0;
   out_6163853603347526612[26] = 0;
   out_6163853603347526612[27] = 0;
   out_6163853603347526612[28] = 0;
   out_6163853603347526612[29] = 0;
   out_6163853603347526612[30] = 1;
   out_6163853603347526612[31] = 0;
   out_6163853603347526612[32] = 0;
   out_6163853603347526612[33] = 0;
   out_6163853603347526612[34] = 0;
   out_6163853603347526612[35] = 0;
   out_6163853603347526612[36] = 0;
   out_6163853603347526612[37] = 0;
   out_6163853603347526612[38] = 0;
   out_6163853603347526612[39] = 0;
   out_6163853603347526612[40] = 1;
   out_6163853603347526612[41] = 0;
   out_6163853603347526612[42] = 0;
   out_6163853603347526612[43] = 0;
   out_6163853603347526612[44] = 0;
   out_6163853603347526612[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6163853603347526612[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6163853603347526612[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6163853603347526612[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6163853603347526612[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6163853603347526612[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6163853603347526612[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6163853603347526612[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6163853603347526612[53] = -9.8000000000000007*dt;
   out_6163853603347526612[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6163853603347526612[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6163853603347526612[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6163853603347526612[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6163853603347526612[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6163853603347526612[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6163853603347526612[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6163853603347526612[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6163853603347526612[62] = 0;
   out_6163853603347526612[63] = 0;
   out_6163853603347526612[64] = 0;
   out_6163853603347526612[65] = 0;
   out_6163853603347526612[66] = 0;
   out_6163853603347526612[67] = 0;
   out_6163853603347526612[68] = 0;
   out_6163853603347526612[69] = 0;
   out_6163853603347526612[70] = 1;
   out_6163853603347526612[71] = 0;
   out_6163853603347526612[72] = 0;
   out_6163853603347526612[73] = 0;
   out_6163853603347526612[74] = 0;
   out_6163853603347526612[75] = 0;
   out_6163853603347526612[76] = 0;
   out_6163853603347526612[77] = 0;
   out_6163853603347526612[78] = 0;
   out_6163853603347526612[79] = 0;
   out_6163853603347526612[80] = 1;
}
void h_25(double *state, double *unused, double *out_8322548960560647008) {
   out_8322548960560647008[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2117618680870572269) {
   out_2117618680870572269[0] = 0;
   out_2117618680870572269[1] = 0;
   out_2117618680870572269[2] = 0;
   out_2117618680870572269[3] = 0;
   out_2117618680870572269[4] = 0;
   out_2117618680870572269[5] = 0;
   out_2117618680870572269[6] = 1;
   out_2117618680870572269[7] = 0;
   out_2117618680870572269[8] = 0;
}
void h_24(double *state, double *unused, double *out_3847341423091520543) {
   out_3847341423091520543[0] = state[4];
   out_3847341423091520543[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2588076162913910993) {
   out_2588076162913910993[0] = 0;
   out_2588076162913910993[1] = 0;
   out_2588076162913910993[2] = 0;
   out_2588076162913910993[3] = 0;
   out_2588076162913910993[4] = 1;
   out_2588076162913910993[5] = 0;
   out_2588076162913910993[6] = 0;
   out_2588076162913910993[7] = 0;
   out_2588076162913910993[8] = 0;
   out_2588076162913910993[9] = 0;
   out_2588076162913910993[10] = 0;
   out_2588076162913910993[11] = 0;
   out_2588076162913910993[12] = 0;
   out_2588076162913910993[13] = 0;
   out_2588076162913910993[14] = 1;
   out_2588076162913910993[15] = 0;
   out_2588076162913910993[16] = 0;
   out_2588076162913910993[17] = 0;
}
void h_30(double *state, double *unused, double *out_1627534431885164616) {
   out_1627534431885164616[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2246957628013812339) {
   out_2246957628013812339[0] = 0;
   out_2246957628013812339[1] = 0;
   out_2246957628013812339[2] = 0;
   out_2246957628013812339[3] = 0;
   out_2246957628013812339[4] = 1;
   out_2246957628013812339[5] = 0;
   out_2246957628013812339[6] = 0;
   out_2246957628013812339[7] = 0;
   out_2246957628013812339[8] = 0;
}
void h_26(double *state, double *unused, double *out_5043807721534234632) {
   out_5043807721534234632[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5859121999744628493) {
   out_5859121999744628493[0] = 0;
   out_5859121999744628493[1] = 0;
   out_5859121999744628493[2] = 0;
   out_5859121999744628493[3] = 0;
   out_5859121999744628493[4] = 0;
   out_5859121999744628493[5] = 0;
   out_5859121999744628493[6] = 0;
   out_5859121999744628493[7] = 1;
   out_5859121999744628493[8] = 0;
}
void h_27(double *state, double *unused, double *out_3177658897444999471) {
   out_3177658897444999471[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4421720939814237250) {
   out_4421720939814237250[0] = 0;
   out_4421720939814237250[1] = 0;
   out_4421720939814237250[2] = 0;
   out_4421720939814237250[3] = 1;
   out_4421720939814237250[4] = 0;
   out_4421720939814237250[5] = 0;
   out_4421720939814237250[6] = 0;
   out_4421720939814237250[7] = 0;
   out_4421720939814237250[8] = 0;
}
void h_29(double *state, double *unused, double *out_4771507085528684530) {
   out_4771507085528684530[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6135083666683788283) {
   out_6135083666683788283[0] = 0;
   out_6135083666683788283[1] = 1;
   out_6135083666683788283[2] = 0;
   out_6135083666683788283[3] = 0;
   out_6135083666683788283[4] = 0;
   out_6135083666683788283[5] = 0;
   out_6135083666683788283[6] = 0;
   out_6135083666683788283[7] = 0;
   out_6135083666683788283[8] = 0;
}
void h_28(double *state, double *unused, double *out_6143899208802909488) {
   out_6143899208802909488[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7229261389956232759) {
   out_7229261389956232759[0] = 1;
   out_7229261389956232759[1] = 0;
   out_7229261389956232759[2] = 0;
   out_7229261389956232759[3] = 0;
   out_7229261389956232759[4] = 0;
   out_7229261389956232759[5] = 0;
   out_7229261389956232759[6] = 0;
   out_7229261389956232759[7] = 0;
   out_7229261389956232759[8] = 0;
}
void h_31(double *state, double *unused, double *out_8597743022845152897) {
   out_8597743022845152897[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2086972718993611841) {
   out_2086972718993611841[0] = 0;
   out_2086972718993611841[1] = 0;
   out_2086972718993611841[2] = 0;
   out_2086972718993611841[3] = 0;
   out_2086972718993611841[4] = 0;
   out_2086972718993611841[5] = 0;
   out_2086972718993611841[6] = 0;
   out_2086972718993611841[7] = 0;
   out_2086972718993611841[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_897924418877243906) {
  err_fun(nom_x, delta_x, out_897924418877243906);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3636397663220992824) {
  inv_err_fun(nom_x, true_x, out_3636397663220992824);
}
void car_H_mod_fun(double *state, double *out_4379404279019457682) {
  H_mod_fun(state, out_4379404279019457682);
}
void car_f_fun(double *state, double dt, double *out_5495334738527013555) {
  f_fun(state,  dt, out_5495334738527013555);
}
void car_F_fun(double *state, double dt, double *out_6163853603347526612) {
  F_fun(state,  dt, out_6163853603347526612);
}
void car_h_25(double *state, double *unused, double *out_8322548960560647008) {
  h_25(state, unused, out_8322548960560647008);
}
void car_H_25(double *state, double *unused, double *out_2117618680870572269) {
  H_25(state, unused, out_2117618680870572269);
}
void car_h_24(double *state, double *unused, double *out_3847341423091520543) {
  h_24(state, unused, out_3847341423091520543);
}
void car_H_24(double *state, double *unused, double *out_2588076162913910993) {
  H_24(state, unused, out_2588076162913910993);
}
void car_h_30(double *state, double *unused, double *out_1627534431885164616) {
  h_30(state, unused, out_1627534431885164616);
}
void car_H_30(double *state, double *unused, double *out_2246957628013812339) {
  H_30(state, unused, out_2246957628013812339);
}
void car_h_26(double *state, double *unused, double *out_5043807721534234632) {
  h_26(state, unused, out_5043807721534234632);
}
void car_H_26(double *state, double *unused, double *out_5859121999744628493) {
  H_26(state, unused, out_5859121999744628493);
}
void car_h_27(double *state, double *unused, double *out_3177658897444999471) {
  h_27(state, unused, out_3177658897444999471);
}
void car_H_27(double *state, double *unused, double *out_4421720939814237250) {
  H_27(state, unused, out_4421720939814237250);
}
void car_h_29(double *state, double *unused, double *out_4771507085528684530) {
  h_29(state, unused, out_4771507085528684530);
}
void car_H_29(double *state, double *unused, double *out_6135083666683788283) {
  H_29(state, unused, out_6135083666683788283);
}
void car_h_28(double *state, double *unused, double *out_6143899208802909488) {
  h_28(state, unused, out_6143899208802909488);
}
void car_H_28(double *state, double *unused, double *out_7229261389956232759) {
  H_28(state, unused, out_7229261389956232759);
}
void car_h_31(double *state, double *unused, double *out_8597743022845152897) {
  h_31(state, unused, out_8597743022845152897);
}
void car_H_31(double *state, double *unused, double *out_2086972718993611841) {
  H_31(state, unused, out_2086972718993611841);
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
