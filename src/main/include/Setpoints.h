#pragma once

#ifndef SETPOINTS_H
#define SETPOINTS_H

const double DEADZONE = 0.05;
const double GRIPPER_POWER = 0.5;

// PEG
// Pivot
// Extension
// Gripper

const double FLOOR_INTAKE[] = {-13, -3.5, -7.5};
const double READY[] = {-17, 1, 0};
const double SINGLE_STATION[] = {-45, -4, -7.5};
const double DOUBLE_STATION[] = {-50, -13.5, -7.5};
const double MID_POSITION[] = {-45, -17};
const double HIGH_POSITION[] = {-46, -30};

#endif