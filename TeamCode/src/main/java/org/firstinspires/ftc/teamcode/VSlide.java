package org.firstinspires.ftc.teamcode;


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
public class VSlide implements Subsystem {
    public static final VSlide INSTANCE = new VSlide();

    private static DcMotorEx vSlideMotorOne;
    private static DcMotorEx vSlideMotorTwo;
    public static DcMotorEx encoder;
    private static int targetPosition = 0;

    // PIDF variables
    public static double kP = 0.08;
    public static double kI = 0.0;
    public static double kD = 0.005;
    public static double kF = 0.0;
    public static int tolerance = 10;

    private static double lastError = 0;
    private static double integral = 0;

    private VSlide() {}

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

        vSlideMotorTwo = new DcMotorV2("v_slide_two", hw);
        vSlideMotorOne = new DcMotorV2("v_slide_one", hw);


        vSlideMotorOne.setDirection(DcMotorV2.Direction.REVERSE);
        vSlideMotorTwo.setDirection(DcMotorV2.Direction.FORWARD);

        vSlideMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlideMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlideMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vSlideMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encoder = vSlideMotorOne; // Use whichever motor has the accurate encoder

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

        vSlideMotorOne.setPower(output);
        vSlideMotorTwo.setPower(output);
    }

    @NonNull
    public static Lambda update() {
        return new Lambda("vslide-update")
                .addRequirements(INSTANCE)
                .setExecute(VSlide::pidfUpdate)
                .setInterruptible(() -> true)
                .setFinish(() -> false);
    }

    @NonNull
    public static Lambda goTo(int target) {
        return new Lambda("vslide-set")
                .setExecute(() -> setTarget(target))
                .setFinish(VSlide::atTarget);
    }

    @NonNull
    public static Lambda retract() {
        return new Lambda("vslide-retract")
                .addRequirements(INSTANCE)
                .setInit(() -> {
                    vSlideMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    vSlideMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    vSlideMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    vSlideMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                });
    }
}
