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
void live_H(double *in_vec, double *out_1975042085507842433);
void live_err_fun(double *nom_x, double *delta_x, double *out_6316537518300744749);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5437110354054996620);
void live_H_mod_fun(double *state, double *out_6104230166974042109);
void live_f_fun(double *state, double dt, double *out_6857365862403648822);
void live_F_fun(double *state, double dt, double *out_5197654036786227137);
void live_h_4(double *state, double *unused, double *out_8949618456756746518);
void live_H_4(double *state, double *unused, double *out_59142098895015889);
void live_h_9(double *state, double *unused, double *out_8154532371085123043);
void live_H_9(double *state, double *unused, double *out_2948003651175095231);
void live_h_10(double *state, double *unused, double *out_2408295147902325293);
void live_H_10(double *state, double *unused, double *out_2111320417356642961);
void live_h_12(double *state, double *unused, double *out_8832198300946419974);
void live_H_12(double *state, double *unused, double *out_7726270412577466381);
void live_h_31(double *state, double *unused, double *out_7166568566371386790);
void live_H_31(double *state, double *unused, double *out_3425804156267623265);
void live_h_32(double *state, double *unused, double *out_6656861460542203515);
void live_H_32(double *state, double *unused, double *out_5191252464388653933);
void live_h_13(double *state, double *unused, double *out_2708158829041384267);
void live_H_13(double *state, double *unused, double *out_7835345973033672311);
void live_h_14(double *state, double *unused, double *out_8154532371085123043);
void live_H_14(double *state, double *unused, double *out_2948003651175095231);
void live_h_33(double *state, double *unused, double *out_5234659048996153164);
void live_H_33(double *state, double *unused, double *out_4824353624168213922);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}