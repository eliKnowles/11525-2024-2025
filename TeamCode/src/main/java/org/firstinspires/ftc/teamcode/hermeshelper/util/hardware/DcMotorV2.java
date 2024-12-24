package org.firstinspires.ftc.teamcode.hermeshelper.util.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DcMotorV2 implements DcMotorEx{
    public DcMotorEx motor = null;
    public final HardwareMap hardwareMap;
    public double integral, lastError, kP, kI, kD, kF = 0;

    public DcMotorV2(String motorName, @NonNull HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
    }

    public void runToPosition(int targetPosition, double power) {
        motor.setTargetPosition(targetPosition);
        motor.setMode(RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    public void runToPosition(int targetPosition) {
        motor.setTargetPosition(targetPosition);
        motor.setMode(RunMode.RUN_TO_POSITION);
        motor.setPower(1.0);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void stop() {
        motor.setPower(0.0);
    }

    public void stopAndReset() {
        motor.setPower(0.0);
        motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setBreak() {
        motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    }

    public void disableBreak() {
        motor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
    }

    public DcMotorEx getRawMotor() {
        return motor;
    }

    public void setMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setDirection(Direction direction) {
        motor.setDirection(direction);
    }

    public void setPowerWithoutPosition(double power) {
        motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }

        public void setPIDFCoefficients(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }

    public void setPositionWithPIDF(double targetPosition, double currentPosition) {
        setPower(computePIDFOutput(targetPosition, currentPosition));
    }
    
    public void runToPositionWithPIDF(double targetPosition) {
        double currentPosition = motor.getCurrentPosition();
        double output = computePIDFOutput(targetPosition, currentPosition);
        motor.setPower(output);
    }
    
    private double computePIDFOutput(double targetPosition, double currentPosition) {
        double error = targetPosition - currentPosition;
        integral += error;
        double derivative = error - lastError;
        lastError = error;
        
        return kP * error + kI * integral + kD * derivative + kF * targetPosition;
    }

    // Normal motor methods

    @Override
    public double getPower() {
        return motor.getPower();
    }

    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    @Deprecated
    public void setPowerFloat() {
        motor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return motor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        motor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return motor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode runMode) {
        motor.setMode(runMode);
    }

    @Override
    public RunMode getMode() {
        return motor.getMode();
    }

    @Override
    public void setMotorEnable() {
        motor.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        motor.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return motor.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        motor.setVelocity(angularRate);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        motor.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return motor.getVelocity(unit);
    }

    @Deprecated
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        motor.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        motor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        motor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        motor.setPositionPIDFCoefficients(p);
    }

    @Deprecated
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return motor.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return motor.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        motor.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return motor.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motor.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        motor.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }

    @Override
    public Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        motor.close();
    }
}
