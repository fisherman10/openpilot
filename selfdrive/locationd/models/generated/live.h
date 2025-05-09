#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_6822288655831934941);
void live_err_fun(double *nom_x, double *delta_x, double *out_4019390996317282630);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4575567528401428033);
void live_H_mod_fun(double *state, double *out_7352548495556649022);
void live_f_fun(double *state, double dt, double *out_2349477927366927039);
void live_F_fun(double *state, double dt, double *out_2364351584771819535);
void live_h_4(double *state, double *unused, double *out_8849287721919723923);
void live_H_4(double *state, double *unused, double *out_1859554171675354118);
void live_h_9(double *state, double *unused, double *out_7920137680748752076);
void live_H_9(double *state, double *unused, double *out_1618364525045763473);
void live_h_10(double *state, double *unused, double *out_706786687963870247);
void live_H_10(double *state, double *unused, double *out_2228941529350231786);
void live_h_12(double *state, double *unused, double *out_7948390464994904187);
void live_H_12(double *state, double *unused, double *out_1238455146627760451);
void live_h_35(double *state, double *unused, double *out_9054749556203381432);
void live_H_35(double *state, double *unused, double *out_1507107885697253258);
void live_h_32(double *state, double *unused, double *out_5221312914566152130);
void live_H_32(double *state, double *unused, double *out_5407756617996863098);
void live_h_13(double *state, double *unused, double *out_3261382629348032471);
void live_H_13(double *state, double *unused, double *out_5457892714386861278);
void live_h_14(double *state, double *unused, double *out_7920137680748752076);
void live_H_14(double *state, double *unused, double *out_1618364525045763473);
void live_h_33(double *state, double *unused, double *out_9179565178789600805);
void live_H_33(double *state, double *unused, double *out_4657664890336110862);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}