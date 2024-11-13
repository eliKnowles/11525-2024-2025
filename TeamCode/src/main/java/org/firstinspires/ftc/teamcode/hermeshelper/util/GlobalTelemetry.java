package org.firstinspires.ftc.teamcode.hermeshelper.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GlobalTelemetry {

    // Singleton instance of telemetry
    private static Telemetry telemetryInstance;

    // Private constructor to prevent instantiation
    private GlobalTelemetry() {}

    // TODO: call GlobalTelemetry.init(telemetry) in TeleOp
    public static void init(Telemetry telemetry) {
        if (telemetryInstance == null) {
            telemetryInstance = telemetry;
        }
    }

    // Accessor for telemetry
    public static Telemetry get() {
        return telemetryInstance;
    }
}