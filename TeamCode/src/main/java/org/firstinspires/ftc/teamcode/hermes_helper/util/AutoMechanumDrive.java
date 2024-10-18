package org.firstinspires.ftc.teamcode.hermes_helper.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoMechanumDrive {
    private final MechanumDrive mechanumDrive;

    public AutoMechanumDrive(String fLName, String fRName, String bLName, String bRName, String imuName, Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        DcMotorV2 fL = new DcMotorV2(fLName, hardwareMap);
        DcMotorV2 fR = new DcMotorV2(fRName, hardwareMap);
        DcMotorV2 bL = new DcMotorV2(bLName, hardwareMap);
        DcMotorV2 bR = new DcMotorV2(bRName, hardwareMap);
        IMUV2 imu = new IMUV2(imuName, hardwareMap, null, null);
        this.mechanumDrive = new MechanumDrive(fL, fR, bL, bR, imu, gamepad1, gamepad2, telemetry);
    }

    public MechanumDrive getRawMechanumDrive() {
        return mechanumDrive;
    }

    public void drive(double x, double y, double rx){
        mechanumDrive.drive(x, y, rx);
    }

    public void drive(double x, double y){
        mechanumDrive.drive(x, y);
    }

    public void drive(){
        mechanumDrive.drive();
    }

    public void fieldCentricDrive(double x, double y, double rx) {
        mechanumDrive.fieldCentricDrive(x, y, rx);
    }

    public void fieldCentricDrive() {
        mechanumDrive.fieldCentricDrive();
    }
}