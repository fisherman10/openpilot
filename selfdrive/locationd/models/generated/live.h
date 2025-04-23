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
void live_H(double *in_vec, double *out_3397704107241963718);
void live_err_fun(double *nom_x, double *delta_x, double *out_6247838002175635449);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7968072719679139883);
void live_H_mod_fun(double *state, double *out_5364641416467943769);
void live_f_fun(double *state, double dt, double *out_1124575111298849253);
void live_F_fun(double *state, double dt, double *out_4805229676219982100);
void live_h_4(double *state, double *unused, double *out_2824070433864028268);
void live_H_4(double *state, double *unused, double *out_8232538440005487589);
void live_h_9(double *state, double *unused, double *out_1980255960950734359);
void live_H_9(double *state, double *unused, double *out_7991348793375896944);
void live_h_10(double *state, double *unused, double *out_5959958634893850991);
void live_H_10(double *state, double *unused, double *out_3854266958234967749);
void live_h_12(double *state, double *unused, double *out_6125305952234532630);
void live_H_12(double *state, double *unused, double *out_7611439414957893922);
void live_h_31(double *state, double *unused, double *out_2375958955441805744);
void live_H_31(double *state, double *unused, double *out_4865876382632880213);
void live_h_32(double *state, double *unused, double *out_478625314351653848);
void live_H_32(double *state, double *unused, double *out_4963811070420394205);
void live_h_13(double *state, double *unused, double *out_6014197381686384807);
void live_H_13(double *state, double *unused, double *out_3900519711919345409);
void live_h_14(double *state, double *unused, double *out_1980255960950734359);
void live_H_14(double *state, double *unused, double *out_7991348793375896944);
void live_h_33(double *state, double *unused, double *out_2599002633094445422);
void live_H_33(double *state, double *unused, double *out_1715319377994022609);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}