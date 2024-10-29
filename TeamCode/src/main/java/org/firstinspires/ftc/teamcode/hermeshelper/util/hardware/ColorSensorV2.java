package org.firstinspires.ftc.teamcode.hermeshelper.util.hardware;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorV2 implements ColorRangeSensor {

    public ColorRangeSensor sensor = null;
    public final HardwareMap hardwareMap;

    public ColorSensorV2(String name, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        sensor = hardwareMap.get(ColorRangeSensor.class, name);
    }

    // Normal color sensor stuff

    @Override
    public int red() {
        return sensor.red();
    }

    @Override
    public int green() {
        return sensor.green();
    }

    @Override
    public int blue() {
        return sensor.blue();
    }

    @Override
    public int alpha() {
        return sensor.alpha();
    }

    @Override
    public int argb() {
        return sensor.argb();
    }

    @Override
    public double getLightDetected() {
        return sensor.getLightDetected();
    }

    @Override
    public double getRawLightDetected() {
        return sensor.getRawLightDetected();
    }

    @Override
    public double getRawLightDetectedMax() {
        return sensor.getRawLightDetectedMax();
    }

    @Override
    public void enableLed(boolean enable) {
        sensor.enableLed(enable);
    }

    @Override
    public String status() {
        return sensor.status();
    }

    @Override
    public void setI2cAddress(I2cAddr newAddress) {
        sensor.setI2cAddress(newAddress);
    }

    @Override
    public I2cAddr getI2cAddress() {
        return sensor.getI2cAddress();
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        return sensor.getDistance(unit);
    }

    @Override
    public NormalizedRGBA getNormalizedColors() {
        return sensor.getNormalizedColors();
    }

    @Override
    public float getGain() {
        return sensor.getGain();
    }

    @Override
    public void setGain(float newGain) {
        sensor.setGain(newGain);
    }

    @Override
    public Manufacturer getManufacturer() {
        return sensor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return sensor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return sensor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return sensor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        sensor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        sensor.close();
    }
}
