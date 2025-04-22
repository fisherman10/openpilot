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
void car_err_fun(double *nom_x, double *delta_x, double *out_7570101262000998390);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8378965514508472172);
void car_H_mod_fun(double *state, double *out_2284935391344495239);
void car_f_fun(double *state, double dt, double *out_1508310104116332664);
void car_F_fun(double *state, double dt, double *out_8054310567907954818);
void car_h_25(double *state, double *unused, double *out_3075084501222235420);
void car_H_25(double *state, double *unused, double *out_8710286078482917812);
void car_h_24(double *state, double *unused, double *out_1935496555183779272);
void car_H_24(double *state, double *unused, double *out_1267611485494201492);
void car_h_30(double *state, double *unused, double *out_4525715893851118587);
void car_H_30(double *state, double *unused, double *out_4182589748355309614);
void car_h_26(double *state, double *unused, double *out_1713163790581265204);
void car_H_26(double *state, double *unused, double *out_4968782759608861588);
void car_h_27(double *state, double *unused, double *out_8368896868694250417);
void car_H_27(double *state, double *unused, double *out_2007826436554884703);
void car_h_29(double *state, double *unused, double *out_475957373809727058);
void car_H_29(double *state, double *unused, double *out_4692821092669701798);
void car_h_28(double *state, double *unused, double *out_613072303312590348);
void car_H_28(double *state, double *unused, double *out_6656451364235028049);
void car_h_31(double *state, double *unused, double *out_3092029538748898883);
void car_H_31(double *state, double *unused, double *out_4342574657375510112);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}