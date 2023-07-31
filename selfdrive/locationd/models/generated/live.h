#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_7593522537215282130);
void live_err_fun(double *nom_x, double *delta_x, double *out_6827781401052364011);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4028348475919853116);
void live_H_mod_fun(double *state, double *out_5325771464325271103);
void live_f_fun(double *state, double dt, double *out_4926465751480700905);
void live_F_fun(double *state, double dt, double *out_683613686407685824);
void live_h_4(double *state, double *unused, double *out_712825342107268946);
void live_H_4(double *state, double *unused, double *out_963657620946598155);
void live_h_9(double *state, double *unused, double *out_3647840826127792566);
void live_H_9(double *state, double *unused, double *out_6323561314317849315);
void live_h_10(double *state, double *unused, double *out_723709260435323646);
void live_H_10(double *state, double *unused, double *out_1735766027073105648);
void live_h_12(double *state, double *unused, double *out_8905093625972493565);
void live_H_12(double *state, double *unused, double *out_4055798787085363640);
void live_h_31(double *state, double *unused, double *out_4201244250452045053);
void live_H_31(double *state, double *unused, double *out_6801361819410377349);
void live_h_32(double *state, double *unused, double *out_4795647535063909201);
void live_H_32(double *state, double *unused, double *out_1051508625677158713);
void live_h_13(double *state, double *unused, double *out_7339748548439160345);
void live_H_13(double *state, double *unused, double *out_1378785699147390946);
void live_h_14(double *state, double *unused, double *out_3647840826127792566);
void live_H_14(double *state, double *unused, double *out_6323561314317849315);
void live_h_33(double *state, double *unused, double *out_4183883782374507509);
void live_H_33(double *state, double *unused, double *out_5553561441064866825);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}