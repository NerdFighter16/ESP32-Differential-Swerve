#pragma once
#include <Bluepad32.h>

extern ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void Gamepad_setup();
void Gamepad_update();
ControllerPtr Gamepad_getConnected();
