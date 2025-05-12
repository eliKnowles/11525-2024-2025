package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

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
    private static ServoV2 outtakePivotServoTwo;
    private static ServoV2 intakeClawServo;
    private static ServoV2 outtakeClawServo;
    private static ServoV2 outtakePivotServo;
    private static ServoV2 intakeWristServo;
    private static ServoV2 intakeWristServoTwo;
    private static ServoV2 outtakeLinkageServo;
    private static ServoV2 outtakeWristServo;


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
                          //  intakeClawServo.setPosition(0);
                          //  outtakeClawServo.setPosition(.2);
                            //outtakePivotServo.setPosition(.5);
                           // outtakePivotServoTwo.setPosition(.5);
                            //intakeWristServo.setPosition(0);
                           // intakeWristServoTwo.setPosition(0);
                            outtakeLinkageServo.setPosition(.1);
                           // outtakeWristServo.setPosition(.2);
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
        outtakePivotServoTwo = new ServoV2("outtake_pivot_two", hardwareMap);
        intakeClawServo = new ServoV2("intake_claw", hardwareMap);
        outtakeClawServo = new ServoV2("outtake_claw", hardwareMap);
        outtakePivotServo = new ServoV2("outtake_pivot_one", hardwareMap);
        intakeWristServo = new ServoV2("intake_wrist", hardwareMap);
        intakeWristServoTwo = new ServoV2("intake_wrist_two", hardwareMap);
        outtakeLinkageServo = new ServoV2("outtake_linkage", hardwareMap);
        outtakeWristServo = new ServoV2("outtake_wrist", hardwareMap);
        outtakePivotServoTwo.setDirection(ServoV2.Direction.REVERSE);
        outtakeLinkageServo.setDirection(ServoV2.Direction.REVERSE);
        outtakeLinkageServo.setPosition(0.05);
        outtakePivotServo.setPosition(.5);
        outtakePivotServoTwo.setPosition(.5);



       /* intakePivotServoOne.setPosition(1);
        //  intakeClawServo.setPosition(0);
          outtakeClawServo.setPosition(0);
        outtakePivotServo.setPosition(.85);
         outtakePivotServoTwo.setPosition(.85);
        //intakeWristServo.setPosition(0);
        // intakeWristServoTwo.setPosition(0);
        outtakeLinkageServo.setPosition(.38);
       // neutral pos outtakeLinkageServo.setPosition(.085);
        outtakeWristServo.setPosition(0); */
    }
}
