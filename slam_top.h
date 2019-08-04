#pragma once

void init_slam();

#include "slam_matchers.h" // result_t

int slam_input_from_tss(tof_slam_set_t* tss, result_t* corr_out);

