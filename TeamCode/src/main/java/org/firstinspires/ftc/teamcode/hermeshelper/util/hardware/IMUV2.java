package org.firstinspires.ftc.teamcode.hermeshelper.util.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hermeshelper.config.HermesConfig;

public class IMUV2 implements IMU {
    private final IMU imu;

    public IMUV2(String name, HardwareMap hardwareMap) {
        this.imu = hardwareMap.get(IMU.class, name);

        Parameters parameters;

        parameters = new Parameters(new RevHubOrientationOnRobot(
                HermesConfig.IMU.LOGO_FACING_DIRECTION, HermesConfig.IMU.USB_FACING_DIRECTION));

        imu.initialize(parameters);

        imu.resetYaw();
    }

    public IMU getRawIMU() {
        return imu;
    }

    // IMU methods

    @Override
    public boolean initialize(Parameters parameters) {
        return imu.initialize(parameters);
    }

    @Override
    public void resetYaw() {
        imu.resetYaw();
    }

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return imu.getRobotYawPitchRollAngles();
    }

    @Override
    public Orientation getRobotOrientation(AxesReference reference, AxesOrder order, AngleUnit angleUnit) {
        return imu.getRobotOrientation(reference, order, angleUnit);
    }

    @Override
    public Quaternion getRobotOrientationAsQuaternion() {
        return imu.getRobotOrientationAsQuaternion();
    }

    @Override
    public AngularVelocity getRobotAngularVelocity(AngleUnit angleUnit) {
        return imu.getRobotAngularVelocity(angleUnit);
    }

    @Override
    public Manufacturer getManufacturer() {
        return imu.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return imu.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return imu.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return imu.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        imu.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        imu.close();
    }
}
