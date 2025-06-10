package org.firstinspires.ftc.teamcode.code.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.code.subsystem.Outtake.clawStates;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.subsystems.Subsystem;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();

    public static DigitalChannel pin0;
    public static DigitalChannel pin1;


    private static CRServo hangServo, hangServo2;
    private static ServoV2 intakePivotServoOne;
    private static ServoV2 intakeClawServo, intakeWristServo, intakeWristServoTwo;
    private static ServoV2 outtakeClawServo, outtakePivotServo;

    private Intake() {}

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @Inherited
    public @interface Attach {}

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));
    @NonNull @Override public Dependency<?> getDependency() { return dependency; }
    @Override public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }

    static int wristPos = 0; // -1 = left, 0 = center, 1 = right

    public enum ClawPosition {
        OPEN(.32), CLOSED(.85);
        public final double pos;
        ClawPosition(double pos) { this.pos = pos; }
    }


    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hw = opMode.getOpMode().hardwareMap;

        pin0 = hw.get(DigitalChannel.class, "digital0");
        pin1 = hw.get(DigitalChannel.class, "digital1");
        pin0.setMode(DigitalChannel.Mode.INPUT);
        pin1.setMode(DigitalChannel.Mode.INPUT);

        hangServo = hw.get(CRServo.class, "hang_servo");
        hangServo2 = hw.get(CRServo.class, "hang_servo_2");




        intakePivotServoOne = new ServoV2("intake_pivot_servo", hw);
        intakeClawServo = new ServoV2("intake_claw", hw);
        intakeWristServo = new ServoV2("intake_wrist", hw);
        intakeWristServoTwo = new ServoV2("intake_wrist_two", hw);
        intakeClawOpen();
        intakeSpecimen();
    }

    public static Parallel runTransfer() {
        return new Parallel(
                new Lambda("Set Wrist to 0").addRequirements(INSTANCE).setExecute(() -> intakeWristServo.setPosition(.43f)),
                new Lambda("Set Wrist2 to .5").addRequirements(INSTANCE).setExecute(() -> intakeWristServoTwo.setPosition(.5f)),


                new Lambda("Set Pivot to 0.55").addRequirements(INSTANCE).setExecute(() -> intakePivotServoOne.setPosition(0.86f))

        );
    }
    public static Parallel wrist_auto() {
        return new Parallel(
                new Lambda("Set Wrist to 0").addRequirements(INSTANCE).setExecute(() -> intakeWristServoTwo.setPosition(.8f))
        );
    }


    public static Parallel hang_1() {
        return new Parallel(
                new Lambda("Set Wrist to 0").addRequirements(INSTANCE).setExecute(() -> hangServo.setPower(1))
        );
    }
    public static Parallel hang_2() {
        return new Parallel(
                new Lambda("Set Wrist to 0").addRequirements(INSTANCE).setExecute(() -> hangServo2.setPower(-1))
        );
    }


    public static Parallel limelightSearch() {
        return new Parallel(
                new Lambda("Set Wrist to 0").addRequirements(INSTANCE).setExecute(() -> intakeWristServo.setPosition(.96f)),
                new Lambda("Set Wrist to 0").addRequirements(INSTANCE).setExecute(() -> intakePivotServoOne.setPosition(.22f))
        );
    }

    public static void setWristPosition() {
        double targetPos = 0.5; // default center
        if (wristPos == -1) targetPos = 0.05;
        if (wristPos == 1) targetPos = 0.95;
        intakeWristServoTwo.setPosition(targetPos);
    }

    public static void adjustWrist(int delta) {
        wristPos = Range.clip(wristPos + delta, -1, 1);
        setWristPosition();
    }

    public static void resetWrist() {
        wristPos = 0;
        setWristPosition();
    }
    public static boolean hasSample() {
        if (pin1.getState() || pin0.getState() && pin1.getState() == false)  {
            return true;
        }
        else {
            return false;
        }
    }


    public static Sequential intakeClawOpen() {
        return new Sequential(
               new Lambda("intake claw open").addRequirements(INSTANCE).setExecute(() -> intakeClawServo.setPosition(ClawPosition.OPEN.pos))

        );
    }

    public static Sequential runRetract() {
        return new Sequential();
    }

    public static Sequential runExtend() {
        return new Sequential(
                new Lambda("Set Wrist2 to 0.5").addRequirements(INSTANCE).setExecute(() -> intakeWristServoTwo.setPosition(0.5f)),
                new Lambda("Set Pivot to 0.07").addRequirements(INSTANCE).setExecute(() -> intakePivotServoOne.setPosition(0.1f)),
                new Lambda("Set Wrist to 0.96").addRequirements(INSTANCE).setExecute(() -> intakeWristServo.setPosition(0.98f))

        );
    }

    public static Sequential intakeSpecimen() {
        return new Sequential(
                new Lambda("Set Wrist2 to 0.5").addRequirements(INSTANCE).setExecute(() -> intakeWristServoTwo.setPosition(0.5f)),
                new Lambda("Set Pivot to 0.07").addRequirements(INSTANCE).setExecute(() -> intakePivotServoOne.setPosition(0.07f)),
                new Lambda("Set Wrist to 0.3").addRequirements(INSTANCE).setExecute(() -> intakeWristServo.setPosition(0.45f)),
                new Lambda("Set Intake Claw to 0.4").addRequirements(INSTANCE).setExecute(() -> intakeClawServo.setPosition(0.4f))
        );
    }


    public static Sequential intakeGrab() {
        return new Sequential(
                new Lambda("Set Pivot to 0.02").addRequirements(INSTANCE).setExecute(() -> intakePivotServoOne.setPosition(0.00f))
                        .setFinish(() -> true),
                new Lambda("Set Intake Claw to 0.92").addRequirements(INSTANCE).setExecute(() -> intakeClawServo.setPosition(ClawPosition.CLOSED.pos))
                        .setFinish(() -> true)
               // new Lambda("Set Pivot to 0.3").addRequirements(INSTANCE).setExecute(() -> intakePivotServoOne.setPosition(0.3f))
        );
    }


    public static Sequential idle() {
        return new Sequential(
                new Lambda("Set Pivot to 0.5").addRequirements(INSTANCE).setExecute(() -> intakePivotServoOne.setPosition(0.5f)),
                new Lambda("Set Wrist2 to 0.5").addRequirements(INSTANCE).setExecute(() -> intakeWristServoTwo.setPosition(0.5f)),
                new Lambda("Set Wrist to 0.7").addRequirements(INSTANCE).setExecute(() -> intakeWristServo.setPosition(0.7f)),
                new Lambda("Set Outtake Pivot to 0.76").addRequirements(INSTANCE).setExecute(() -> outtakePivotServo.setPosition(0.76f))
        );
    }
}
