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
void car_err_fun(double *nom_x, double *delta_x, double *out_706472028070702947);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4384673494611099764);
void car_H_mod_fun(double *state, double *out_399698239415973843);
void car_f_fun(double *state, double dt, double *out_764333901801294821);
void car_F_fun(double *state, double dt, double *out_3701654446068242986);
void car_h_25(double *state, double *unused, double *out_6301433132971002834);
void car_H_25(double *state, double *unused, double *out_7433832135825478518);
void car_h_24(double *state, double *unused, double *out_3213610399440082051);
void car_H_24(double *state, double *unused, double *out_4437340131292554997);
void car_h_30(double *state, double *unused, double *out_2836284151712848348);
void car_H_30(double *state, double *unused, double *out_4096221596392456343);
void car_h_26(double *state, double *unused, double *out_3320528437082837162);
void car_H_26(double *state, double *unused, double *out_3692328816951422294);
void car_h_27(double *state, double *unused, double *out_7143637673077703137);
void car_H_27(double *state, double *unused, double *out_6270984908192881254);
void car_h_29(double *state, double *unused, double *out_7300824341428832282);
void car_H_29(double *state, double *unused, double *out_3585990252078064159);
void car_h_28(double *state, double *unused, double *out_8266320349187515794);
void car_H_28(double *state, double *unused, double *out_8668389269147594733);
void car_h_31(double *state, double *unused, double *out_5385182467266385755);
void car_H_31(double *state, double *unused, double *out_7464478097702438946);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}