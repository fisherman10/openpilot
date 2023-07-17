#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_2829633900128880649);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7762599449520589818);
void car_H_mod_fun(double *state, double *out_1808600300430226605);
void car_f_fun(double *state, double dt, double *out_6453172057652161082);
void car_F_fun(double *state, double dt, double *out_2576361439132911649);
void car_h_25(double *state, double *unused, double *out_2629926194421007623);
void car_H_25(double *state, double *unused, double *out_5004037656767691018);
void car_h_24(double *state, double *unused, double *out_1503054954568587628);
void car_H_24(double *state, double *unused, double *out_1571534149823827083);
void car_h_30(double *state, double *unused, double *out_8132025166865176366);
void car_H_30(double *state, double *unused, double *out_1912652684723925737);
void car_h_26(double *state, double *unused, double *out_7710313586035677599);
void car_H_26(double *state, double *unused, double *out_8745540975641747242);
void car_h_27(double *state, double *unused, double *out_2371747073239838022);
void car_H_27(double *state, double *unused, double *out_262110627076499174);
void car_h_29(double *state, double *unused, double *out_4399088153110512914);
void car_H_29(double *state, double *unused, double *out_1975473353946050207);
void car_h_28(double *state, double *unused, double *out_1204003272163696285);
void car_H_28(double *state, double *unused, double *out_7057872371015580781);
void car_h_31(double *state, double *unused, double *out_2472739526069878478);
void car_H_31(double *state, double *unused, double *out_4973391694890730590);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}