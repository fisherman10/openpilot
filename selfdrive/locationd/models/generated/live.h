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
void live_H(double *in_vec, double *out_1180236145992489736);
void live_err_fun(double *nom_x, double *delta_x, double *out_6715227640650976900);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1548982549716130313);
void live_H_mod_fun(double *state, double *out_2860172969077957858);
void live_f_fun(double *state, double dt, double *out_7509353369532479333);
void live_F_fun(double *state, double dt, double *out_4886194137270510489);
void live_h_4(double *state, double *unused, double *out_4637897841970702621);
void live_H_4(double *state, double *unused, double *out_5805323984074307015);
void live_h_9(double *state, double *unused, double *out_8591118695218469571);
void live_H_9(double *state, double *unused, double *out_5354201154370797131);
void live_h_10(double *state, double *unused, double *out_7242359529701963629);
void live_H_10(double *state, double *unused, double *out_4137885373585509426);
void live_h_12(double *state, double *unused, double *out_8526913763654168672);
void live_H_12(double *state, double *unused, double *out_575934392968425981);
void live_h_31(double *state, double *unused, double *out_5203034508709014668);
void live_H_31(double *state, double *unused, double *out_2228728743627780400);
void live_h_32(double *state, double *unused, double *out_6866430390157578802);
void live_H_32(double *state, double *unused, double *out_6285504618598459895);
void live_h_13(double *state, double *unused, double *out_4190634259273921891);
void live_H_13(double *state, double *unused, double *out_5258487694199186349);
void live_h_14(double *state, double *unused, double *out_8591118695218469571);
void live_H_14(double *state, double *unused, double *out_5354201154370797131);
void live_h_33(double *state, double *unused, double *out_211279920056200028);
void live_H_33(double *state, double *unused, double *out_921828261011077204);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}