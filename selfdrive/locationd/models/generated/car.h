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
void car_err_fun(double *nom_x, double *delta_x, double *out_5515148942190465097);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_891077304524521050);
void car_H_mod_fun(double *state, double *out_3776379998635721899);
void car_f_fun(double *state, double dt, double *out_561451910231373417);
void car_F_fun(double *state, double dt, double *out_9175892949450483581);
void car_h_25(double *state, double *unused, double *out_2733309602735812484);
void car_H_25(double *state, double *unused, double *out_3753800710342315827);
void car_h_24(double *state, double *unused, double *out_4404590517770641861);
void car_H_24(double *state, double *unused, double *out_5474264475726879398);
void car_h_30(double *state, double *unused, double *out_5906567873982729400);
void car_H_30(double *state, double *unused, double *out_3883139657485555897);
void car_h_26(double *state, double *unused, double *out_7538502932065976571);
void car_H_26(double *state, double *unused, double *out_7495304029216372051);
void car_h_27(double *state, double *unused, double *out_6009157415483269073);
void car_H_27(double *state, double *unused, double *out_6057902969285980808);
void car_h_29(double *state, double *unused, double *out_6284351477767774962);
void car_H_29(double *state, double *unused, double *out_7771265696155531841);
void car_h_28(double *state, double *unused, double *out_4318709580747588113);
void car_H_28(double *state, double *unused, double *out_5593079360484489201);
void car_h_31(double *state, double *unused, double *out_8942599244623388196);
void car_H_31(double *state, double *unused, double *out_3723154748465355399);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}