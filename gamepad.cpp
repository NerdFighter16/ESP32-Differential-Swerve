#include "gamepad.h"

ControllerPtr myControllers[BP32_MAX_GAMEPADS] = {nullptr};

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
        if (!myControllers[i]) {
            myControllers[i] = ctl;
            Serial.printf("Connected controller at index %d\n", i);
            return;
        }
    }
    Serial.println("Controller connected, but no free slots");
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            Serial.printf("Disconnected controller at index %d\n", i);
            return;
        }
    }
    Serial.println("Disconnected controller not found");
}

void Gamepad_setup() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
}

void Gamepad_update() {
    BP32.update();
}

ControllerPtr Gamepad_getConnected() {
    for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
        if (myControllers[i] && myControllers[i]->isConnected()) {
            return myControllers[i];
        }
    }
    return nullptr;
}
