#pragma once

void init_slam();

#include "slam_matchers.h" // result_t

int slam_input_from_tss(tof_slam_set_t* tss, result_t* corr_out);

int input_tof_slam_set_for_gyrocal(tof_slam_set_t* tss, int state);

void gyroslam_process_before();
double gyroslam_process_after();
void gyroslam_empty();
