package org.firstinspires.ftc.teamcode.code;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

@Config
public class HSlide implements Subsystem {
    public static final HSlide INSTANCE = new HSlide();

    private static DcMotorEx hSlideMotorOne;
    private static DcMotorEx hSlideMotorTwo;
    public static DcMotorEx encoder;
    private static int targetPosition = 0;

    public static double kP = 0.04;
    public static double kI = 0.0;
    public static double kD = 0.005;
    public static double kF = 0.0;
    public static int tolerance = 5;

    private static double lastError = 0;
    private static double integral = 0;

    private HSlide() {}

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @MustBeDocumented
    @Inherited
    public @interface Attach {}

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));
    @NonNull
    @Override public Dependency<?> getDependency() { return dependency; }
    @Override public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hw = opMode.getOpMode().hardwareMap;

        hSlideMotorOne = new DcMotorV2("h_slide", hw);

        hSlideMotorOne.setDirection(DcMotorV2.Direction.REVERSE);

        hSlideMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hSlideMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        goTo(0);

        encoder = hSlideMotorOne;

        setDefaultCommand(update());
    }

    public static void setTarget(int ticks) {
        targetPosition = ticks;
        integral = 0;
        lastError = 0;
    }

    public static boolean atTarget() {
        return Math.abs(targetPosition - encoder.getCurrentPosition()) < tolerance;
    }

    public static void resetEncoders() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void pidfUpdate() {
        double current = encoder.getCurrentPosition();
        double error = targetPosition - current;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double output = kP * error + kI * integral + kD * derivative + kF * targetPosition;
        output = Math.max(-1, Math.min(1, output));

        if (Math.abs(output) < 0.05) output = 0;


        hSlideMotorOne.setPower(output);
    }

    @NonNull
    public static Lambda update() {
        return new Lambda("hslide-update")
                .addRequirements(INSTANCE)
                .setExecute(HSlide::pidfUpdate)
                .setInterruptible(() -> true)
                .setFinish(() -> false);
    }

    @NonNull
    public static Lambda goTo(int target) {
        return new Lambda("hslide-set")
                .setExecute(() -> setTarget(target))
                .setFinish(HSlide::atTarget);
    }

    @NonNull
    public static Lambda retract() {
        return new Lambda("hslide-retract")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    hSlideMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    hSlideMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    hSlideMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    hSlideMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                });
    }
}
