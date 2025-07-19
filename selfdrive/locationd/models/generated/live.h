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
void live_H(double *in_vec, double *out_9139619358220741889);
void live_err_fun(double *nom_x, double *delta_x, double *out_8028881765390267423);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1190762036757190897);
void live_H_mod_fun(double *state, double *out_5477359023855089867);
void live_f_fun(double *state, double dt, double *out_2101094440922555394);
void live_F_fun(double *state, double dt, double *out_8294613652205942994);
void live_h_4(double *state, double *unused, double *out_6471440480384820624);
void live_H_4(double *state, double *unused, double *out_92139954894711518);
void live_h_9(double *state, double *unused, double *out_58924091347204254);
void live_H_9(double *state, double *unused, double *out_7195078980369735952);
void live_h_10(double *state, double *unused, double *out_450912211379179165);
void live_H_10(double *state, double *unused, double *out_7446966426181119485);
void live_h_12(double *state, double *unused, double *out_3285972173859264459);
void live_H_12(double *state, double *unused, double *out_4927316453137250277);
void live_h_35(double *state, double *unused, double *out_8104768064320442641);
void live_H_35(double *state, double *unused, double *out_7672879485462263986);
void live_h_32(double *state, double *unused, double *out_8422499417943187138);
void live_H_32(double *state, double *unused, double *out_3405686887418636327);
void live_h_13(double *state, double *unused, double *out_4067265527449708732);
void live_H_13(double *state, double *unused, double *out_5738060543198070931);
void live_h_14(double *state, double *unused, double *out_58924091347204254);
void live_H_14(double *state, double *unused, double *out_7195078980369735952);
void live_h_33(double *state, double *unused, double *out_6671869651883879035);
void live_H_33(double *state, double *unused, double *out_6425079107116753462);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}