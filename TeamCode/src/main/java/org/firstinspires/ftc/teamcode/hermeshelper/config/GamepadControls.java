package org.firstinspires.ftc.teamcode.hermeshelper.config;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class GamepadControls {
    // Deadzone values for left and right sticks
    private final double L_STICK_DEADZONE = 0.1;
    private final double R_STICK_DEADZONE = 0.1;

    private Map<String, String> gamepadOneControlDefinitions = new HashMap<>();
    private Map<String, String> gamepadTwoControlDefinitions = new HashMap<>();

    // Make sure to initialize for both gamepads
    public void InitControls(Gamepad gamepad) {
        if(gamepad.id == 1) {
            // TODO: Initialize gamepad1 controls here
            // Example: addControl("resetIMU", "A", gamepad);
        } else if(gamepad.id == 2) {
            // TODO: Initialize gamepad2 controls here
            // Example: addControl("resetIMU", "A", gamepad);
        }
    }

    public void addControl(String name, String control, Gamepad gamepad) {
        if(gamepad.id == 1) {
            gamepadOneControlDefinitions.put(name, control);
        } else if(gamepad.id == 2) {
            gamepadTwoControlDefinitions.put(name, control);
        }
    }

    public boolean getControl(String name, Gamepad gamepad) {
        boolean value = false;

        if(gamepad.id == 1) {
            value = getBooleanControlValue(Objects.requireNonNull(gamepadOneControlDefinitions.get(name)), gamepad);
        } else if(gamepad.id == 2) {
            value = getBooleanControlValue(Objects.requireNonNull(gamepadTwoControlDefinitions.get(name)), gamepad);
        }

        return value;
    }

    public double getDoubleControl(String name, Gamepad gamepad) {
        double value = 0.0;

        if(gamepad.id == 1) {
            value = getDoubleControlValue(Objects.requireNonNull(gamepadOneControlDefinitions.get(name)), gamepad);
        } else if(gamepad.id == 2) {
            value = getDoubleControlValue(Objects.requireNonNull(gamepadTwoControlDefinitions.get(name)), gamepad);
        }

        return value;
    }

    public boolean getBooleanControlValue(String control, Gamepad gamepad) {
        switch (control) {
            case "A":
                return gamepad.a;
            case "B":
                return gamepad.b;
            case "X":
                return gamepad.x;
            case "Y":
                return gamepad.y;
            case "LEFT_BUMPER":
                return gamepad.left_bumper;
            case "RIGHT_BUMPER":
                return gamepad.right_bumper;
            default:
                return false;
        }
    }

    public double getDoubleControlValue(String control, Gamepad gamepad) {
        switch (control) {
            case "LEFT_TRIGGER":
                return gamepad.left_trigger;
            case "RIGHT_TRIGGER":
                return gamepad.right_trigger;
            case "LEFT_STICK_X":
                return gamepad.left_stick_x;
            case "LEFT_STICK_Y":
                return gamepad.left_stick_y;
            case "RIGHT_STICK_X":
                return gamepad.right_stick_x;
            case "RIGHT_STICK_Y":
                return gamepad.right_stick_y;
            default:
                return 0.0;
        }
    }
}
