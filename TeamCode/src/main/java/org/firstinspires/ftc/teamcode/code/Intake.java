
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.mercurial.subsystems.Subsystem;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();

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

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hw = opMode.getOpMode().hardwareMap;

        intakePivotServoOne = new ServoV2("intake_pivot_servo", hw);
        intakeClawServo = new ServoV2("intake_claw", hw);
        intakeWristServo = new ServoV2("intake_wrist", hw);
        intakeWristServoTwo = new ServoV2("intake_wrist_two", hw);

    }

    public static Sequential runTransfer() {
        return new Sequential(
                new Lambda("Set Wrist to 0").addRequirements(INSTANCE).setExecute(() -> intakeWristServo.setPosition(0f)),
                new Lambda("Set Pivot to 0.55").addRequirements(INSTANCE).setExecute(() -> intakePivotServoOne.setPosition(0.55f)), new Wait(0.1),
                new Lambda("Set Wrist2 to 0.5").addRequirements(INSTANCE).setExecute(() -> intakeWristServoTwo.setPosition(0.5f)),
                new Lambda("Set Outtake Claw to 0.78").addRequirements(INSTANCE).setExecute(() -> outtakeClawServo.setPosition(0.78f)), new Wait(0.5),
                new Lambda("Set Intake Claw to 0.4").addRequirements(INSTANCE).setExecute(() -> intakeClawServo.setPosition(0.4f)), new Wait(0.1),
                new Lambda("Set Wrist to 0.35").addRequirements(INSTANCE).setExecute(() -> intakeWristServo.setPosition(0.35f)), new Wait(0.1),
                new Lambda("Set Outtake Pivot to 0.3").addRequirements(INSTANCE).setExecute(() -> outtakePivotServo.setPosition(0.3f))
        );
    }

    public static Sequential runRetract() {
        return new Sequential();
    }

    public static Sequential runExtend() {
        return new Sequential(
                new Lambda("Set Wrist2 to 0.5").addRequirements(INSTANCE).setExecute(() -> intakeWristServoTwo.setPosition(0.5f)),
                new Lambda("Set Outtake Pivot to 0.79").addRequirements(INSTANCE).setExecute(() -> outtakePivotServo.setPosition(0.79f)),
                new Lambda("Set Outtake Claw to 0.98").addRequirements(INSTANCE).setExecute(() -> outtakeClawServo.setPosition(0.98f)),
                new Lambda("Set Pivot to 0.07").addRequirements(INSTANCE).setExecute(() -> intakePivotServoOne.setPosition(0.07f)),
                new Lambda("Set Wrist to 0.96").addRequirements(INSTANCE).setExecute(() -> intakeWristServo.setPosition(0.96f)),
                new Lambda("Set Intake Claw to 0.4").addRequirements(INSTANCE).setExecute(() -> intakeClawServo.setPosition(0.4f)), new Wait(0.6)
        );
    }

    public static Sequential intakeNeutral() {
        return new Sequential(
                new Lambda("Set Wrist2 to 0.5").addRequirements(INSTANCE).setExecute(() -> intakeWristServoTwo.setPosition(0.5f)),
                new Lambda("Set Outtake Pivot to 0.79").addRequirements(INSTANCE).setExecute(() -> outtakePivotServo.setPosition(0.79f)),
                new Lambda("Set Outtake Claw to 0.98").addRequirements(INSTANCE).setExecute(() -> outtakeClawServo.setPosition(0.98f)),
                new Lambda("Set Pivot to 0.07").addRequirements(INSTANCE).setExecute(() -> intakePivotServoOne.setPosition(0.07f)),
                new Lambda("Set Wrist to 0.96").addRequirements(INSTANCE).setExecute(() -> intakeWristServo.setPosition(0.96f)),
                new Lambda("Set Intake Claw to 0.4").addRequirements(INSTANCE).setExecute(() -> intakeClawServo.setPosition(0.4f))
        );
    }


    public static Sequential intakeGrab() {
        return new Sequential(
                new Lambda("Set Pivot to 0.02").addRequirements(INSTANCE).setExecute(() -> intakePivotServoOne.setPosition(0.02f)),
                new Lambda("Set Intake Claw to 0.92").addRequirements(INSTANCE).setExecute(() -> intakeClawServo.setPosition(0.92f)), new Wait(0.1),
                new Lambda("Set Pivot to 0.3").addRequirements(INSTANCE).setExecute(() -> intakePivotServoOne.setPosition(0.3f)), new Wait(0.3)
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
