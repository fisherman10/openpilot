#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_897924418877243906);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3636397663220992824);
void car_H_mod_fun(double *state, double *out_4379404279019457682);
void car_f_fun(double *state, double dt, double *out_5495334738527013555);
void car_F_fun(double *state, double dt, double *out_6163853603347526612);
void car_h_25(double *state, double *unused, double *out_8322548960560647008);
void car_H_25(double *state, double *unused, double *out_2117618680870572269);
void car_h_24(double *state, double *unused, double *out_3847341423091520543);
void car_H_24(double *state, double *unused, double *out_2588076162913910993);
void car_h_30(double *state, double *unused, double *out_1627534431885164616);
void car_H_30(double *state, double *unused, double *out_2246957628013812339);
void car_h_26(double *state, double *unused, double *out_5043807721534234632);
void car_H_26(double *state, double *unused, double *out_5859121999744628493);
void car_h_27(double *state, double *unused, double *out_3177658897444999471);
void car_H_27(double *state, double *unused, double *out_4421720939814237250);
void car_h_29(double *state, double *unused, double *out_4771507085528684530);
void car_H_29(double *state, double *unused, double *out_6135083666683788283);
void car_h_28(double *state, double *unused, double *out_6143899208802909488);
void car_H_28(double *state, double *unused, double *out_7229261389956232759);
void car_h_31(double *state, double *unused, double *out_8597743022845152897);
void car_H_31(double *state, double *unused, double *out_2086972718993611841);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}