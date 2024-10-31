package org.firstinspires.ftc.teamcode.hermeshelper.util.mechanum_drive;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hermeshelper.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.IMUV2;

public class MechanumDrive {
    private final DcMotorV2 fL;
    private final DcMotorV2 fR;
    private final DcMotorV2 bL;
    private final DcMotorV2 bR;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final IMUV2 imu;

    public MechanumDrive(DcMotorV2 fL, DcMotorV2 fR, DcMotorV2 bL, DcMotorV2 bR, IMUV2 imu, Gamepad gamepad1, Gamepad gamepad2) {
        this.fL = fL;
        this.fR = fR;
        this.bL = bL;
        this.bR = bR;
        this.imu = imu;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public DcMotorV2 getFL() {
        return fL;
    }

    public DcMotorV2 getFR() {
        return fR;
    }

    public DcMotorV2 getBL() {
        return bL;
    }

    public DcMotorV2 getBR() {
        return bR;
    }

    public void drive(double x, double y, double rx){
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        fL.setPower((y + x + rx) / denominator);
        bL.setPower((y - x + rx) / denominator);
        fR.setPower((y - x - rx) / denominator);
        bR.setPower((y + x - rx) / denominator);
    }

    public void drive(double x, double y){
        fL.setPower(y + x);
        fR.setPower(y - x);
        bL.setPower(-y - x);
        bR.setPower(-y + x);
    }

    public void drive(){
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        fL.setPower((y + x + rx) / denominator);
        bL.setPower((y - x + rx) / denominator);
        fR.setPower((y - x - rx) / denominator);
        bR.setPower((y + x - rx) / denominator);
    }

    public void fieldCentricDrive(double x, double y, double rx) {
        double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        GlobalTelemetry.get().addData("bot heading: ", botHeading);
        GlobalTelemetry.get().update();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        fL.setPower((rotY + rotX + rx) / denominator);
        bL.setPower((rotY - rotX + rx) / denominator);
        fR.setPower((rotY - rotX - rx) / denominator);
        bR.setPower((rotY + rotX - rx) / denominator);
    }

    public void fieldCentricDrive() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        fL.setPower((rotY + rotX + rx) / denominator);
        bL.setPower((rotY - rotX + rx) / denominator);
        fR.setPower((rotY - rotX - rx) / denominator);
        bR.setPower((rotY + rotX - rx) / denominator);
    }
}