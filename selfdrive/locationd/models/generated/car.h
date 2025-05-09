#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8009260081015432478);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_652454977147307792);
void car_H_mod_fun(double *state, double *out_1090209382812354337);
void car_f_fun(double *state, double dt, double *out_2878359103814338252);
void car_F_fun(double *state, double dt, double *out_7685646572396440351);
void car_h_25(double *state, double *unused, double *out_3524389857431468744);
void car_H_25(double *state, double *unused, double *out_8102978843099882900);
void car_h_24(double *state, double *unused, double *out_7474005308437218817);
void car_H_24(double *state, double *unused, double *out_1125086342969312325);
void car_h_30(double *state, double *unused, double *out_3681576525782597889);
void car_H_30(double *state, double *unused, double *out_5816068900482060518);
void car_h_26(double *state, double *unused, double *out_9095028842829394371);
void car_H_26(double *state, double *unused, double *out_6602261911735612492);
void car_h_27(double *state, double *unused, double *out_6561175908875801429);
void car_H_27(double *state, double *unused, double *out_8039662971666003735);
void car_h_29(double *state, double *unused, double *out_5114732938483399251);
void car_H_29(double *state, double *unused, double *out_6326300244796452702);
void car_h_28(double *state, double *unused, double *out_1559502641214955784);
void car_H_28(double *state, double *unused, double *out_1243901227726922128);
void car_h_31(double *state, double *unused, double *out_3799583919715974633);
void car_H_31(double *state, double *unused, double *out_8072332881222922472);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}