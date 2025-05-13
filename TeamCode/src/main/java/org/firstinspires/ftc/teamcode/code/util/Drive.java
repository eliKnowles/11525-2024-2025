package org.firstinspires.ftc.teamcode.code.util;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hermeshelper.pp.constants.FConstants;
import org.firstinspires.ftc.teamcode.hermeshelper.pp.constants.LConstants;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();

    public static Follower follower;

    public static DcMotorEx fl;
    public static DcMotorEx fr;
    public static DcMotorEx bl;
    public static DcMotorEx br;

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    private Telemetry telemetry;

    public static void drive(double x, double y, double z) {
        follower.setTeleOpMovementVectors(x, y, z, false);
        follower.update();
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach { }

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        this.telemetry = opMode.getOpMode().telemetry;
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(FeatureRegistrar.getActiveOpMode().hardwareMap);

        follower.setStartingPose(new Pose(0, 0, 0));
        setDefaultCommand(drive(Mercurial.gamepad1()));

        HardwareMap hardwareMap = opMode.getOpMode().hardwareMap;
        fl = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        bl = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        fr = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        br = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
    }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        follower.startTeleopDrive();
    }

    public static Lambda drive(BoundGamepad gamepad){
        return new Lambda("drive")
                .addRequirements(INSTANCE)
                .setExecute(() -> drive(
                        -gamepad.leftStickY().state(),
                        gamepad.leftStickX().state(),
                        gamepad.rightStickX().state()
                ))
                .setFinish(() -> false);
    }
}
