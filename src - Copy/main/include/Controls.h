#pragma once

#ifndef CONTROLS_H
#define CONTROLS_H

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>

frc::XboxController controller{0};
frc::Joystick pivotExtensionStick{1};

frc2::JoystickButton button1{&controller, 1};
frc2::JoystickButton button2{&controller, 2};
frc2::JoystickButton button3{&controller, 3};
frc2::JoystickButton button4{&controller, 4};
frc2::JoystickButton rightTrigger{&controller, 8};

frc2::JoystickButton leftBumper{&controller, 5};
frc2::JoystickButton rightBumper{&controller, 6};

frc2::JoystickButton toggleSoftLimits{&pivotExtensionStick, 1};

#endif