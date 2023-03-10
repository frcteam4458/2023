#pragma once

#ifndef CONSTANTS_H
#define CONSTANTS_H

#define CAN_DRIVETRAIN 1


#include <units/time.h>
#include <units/voltage.h>
#include <units/length.h>
#include <units/angle.h>

const auto FL = 1;
const auto FR = 2;
const auto BL = 3;
const auto BR = 4;
const auto LIFT = 5;
const auto EXTENSION = 6;
const auto GRIPPER = 7;

const int LEFT_ENCODER[] = {0, 1};
const int RIGHT_ENCODER[] = {2, 3};

const int GYRO = 0;

// motor constants

const auto kS = 2.207_V;
const auto kV = 0.62873_V * 1_s / 1_m;
const auto kA = 0.81252_V * 1_s * 1_s / 1_m;

const auto akS = 2.27_V;
const auto akV = 1.2917_V * 1_s / 1_m;
const auto akA = 0.025244_V * 1_s * 1_s / 1_m;

const auto lkP = 1.2463;
const auto rkP = 1.4399;
const auto akP = (lkP + rkP) / 2;

const auto ramseteB = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
const auto ramseteZeta = 0.7 * 1_rad;

const auto trackWidth = 0.7366_m;

const double CUBE_AMPERAGE = 1.0;
const double CONE_AMPERAGE = 1.0;

#endif