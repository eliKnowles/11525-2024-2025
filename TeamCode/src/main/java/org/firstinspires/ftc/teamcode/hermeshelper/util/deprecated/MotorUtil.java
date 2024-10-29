package org.firstinspires.ftc.teamcode.hermeshelper.util.deprecated;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Deprecated
public class MotorUtil {
    private final DcMotorEx motor;

    public MotorUtil(DcMotorEx motor) {
        this.motor = motor;
    }

    public void runToPosition(int targetPosition, double power) {
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    public void runToPosition(int targetPosition) {
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1.0);
    }

    public void setPower(double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }

    public void stop() {
        motor.setPower(0.0);
    }

    public void stopAndReset() {
        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
