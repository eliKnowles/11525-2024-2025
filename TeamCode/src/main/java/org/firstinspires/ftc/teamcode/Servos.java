package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.concurrent.atomic.AtomicLong;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class Servos implements Subsystem {
    public static final Servos INSTANCE = new Servos();

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Mercurial.Attach.class));

    private static ServoV2 intakePivotServoOne;
    private static ServoV2 intakePivotServoTwo;
    private static ServoV2 intakeClawServo;
    private static ServoV2 outtakeClawServo;
    private static ServoV2 outtakePivotServo;
    private static ServoV2 intakeWristServo;
    private static ServoV2 intakeWristServoTwo;

    public Servos() {}

    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    public static Lambda setZero(){
        return new Lambda("set-zero")
                .addRequirements(INSTANCE)
                .setExecute(
                        () -> {
                            intakePivotServoOne.setPosition(0);
                            intakePivotServoTwo.setPosition(0);
                            intakeClawServo.setPosition(0);
                            outtakeClawServo.setPosition(0);
                            outtakePivotServo.setPosition(0);
                            intakeWristServo.setPosition(0);
                            intakeWristServoTwo.setPosition(0);
                        }
                );
    }

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
        HardwareMap hardwareMap = opMode.getOpMode().hardwareMap;

        intakePivotServoOne = new ServoV2("intake_pivot_one", hardwareMap);
        intakePivotServoTwo = new ServoV2("intake_pivot_two", hardwareMap);
        intakeClawServo = new ServoV2("intake_claw", hardwareMap);
        outtakeClawServo = new ServoV2("outtake_claw", hardwareMap);
        outtakePivotServo = new ServoV2("outtake_pivot_one", hardwareMap);
        intakeWristServo = new ServoV2("intake_wrist", hardwareMap);
        intakeWristServoTwo = new ServoV2("intake_wrist_two", hardwareMap);
    }
}
