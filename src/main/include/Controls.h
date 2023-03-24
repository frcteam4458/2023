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
frc2::JoystickButton stopgrip{&controller, 1};
frc2::JoystickButton single{&controller, 2};
frc2::JoystickButton doubleb{&controller, 3};
frc2::JoystickButton ready{&controller, 4};

frc2::JoystickButton leftBumper{&controller, 5};
frc2::JoystickButton rightBumper{&controller, 6};

frc2::JoystickButton toggleSoftLimits{&pivotExtensionStick, 2};

frc2::JoystickButton joystickTrigger{&pivotExtensionStick, 1};

frc2::JoystickButton purple{&pivotExtensionStick, 4};
frc2::JoystickButton team{&pivotExtensionStick, 3};
frc2::JoystickButton orange{&pivotExtensionStick, 5};

frc2::JoystickButton readyStick{&pivotExtensionStick, 6};
frc2::JoystickButton intake{&pivotExtensionStick, 7};

frc2::JoystickButton singlestick{&pivotExtensionStick, 11};
frc2::JoystickButton slidestick{&pivotExtensionStick, 10};


#endif