package org.firstinspires.ftc.teamcode.code.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.StateMachine;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.mercurial.subsystems.Subsystem;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Config
public class Outtake implements Subsystem {
    public static final Outtake INSTANCE = new Outtake();

    private static ServoV2 outtakePivotOne;
    private static ServoV2 outtakePivotTwo;
    private static ServoV2 outtakeLinkage;
    private static ServoV2 outtakeClaw;
    private static ServoV2 outtakeWrist;
    private static Servo LED;
    private static Wrapper opModeInstance;

    private static boolean specMode = false;

    public static final StateMachine<OuttakeStates> clawStates = new StateMachine<>(OuttakeStates.RETRACTED_SAMPLE)
            .withState(OuttakeStates.EXTENDED_SPEC, (ref, name) -> new Lambda("noop")
                    .setFinish(() -> true))
            .withState(OuttakeStates.SPECIMEN_WALL, (ref, name) -> new Lambda("noop")
                    .setFinish(() -> true))
            .withState(OuttakeStates.EXTENDED_SAMPLE, (ref, name) ->new Lambda("noop")
                    .setFinish(() -> true))
            .withState(OuttakeStates.RETRACTED_SAMPLE, (ref, name) -> new Lambda("noop")
                    .setFinish(() -> true))
            .withState(OuttakeStates.TRANSFER_SAMPLE, (ref, name) -> new Lambda("noop")
                    .setFinish(() -> true))
            .withState(OuttakeStates.EXTENDED_INTAKE, (ref, name) -> new Lambda("noop")
                    .setFinish(() -> true));


    public static StateMachine<OuttakeStates> getClawStates() {
        return clawStates;
    }

    public enum OuttakeStates {
        RETRACTED_SAMPLE,
        EXTENDED_SAMPLE,
        SPECIMEN_WALL,
        EXTENDED_SPEC,
        RETRACTED_SPEC,
        TRANSFER_SAMPLE,
        EXTENDED_INTAKE
    }

    public OuttakeStates getState() {
        return clawStates.getState();
    }

    public static Lambda toggleMode() {
        return new Lambda("toggle mode")
                .setExecute(() -> specMode = !specMode)
                .setFinish(() -> true);
    }
    public static boolean isSpecMode() {
        return specMode;
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @Inherited
    public @interface Attach {}

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    private Outtake() {}

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        opModeInstance = opMode;
        HardwareMap hw = opMode.getOpMode().hardwareMap;

        outtakePivotOne = new ServoV2("outtake_pivot_one", hw);
        outtakePivotTwo = new ServoV2("outtake_pivot_two", hw);
        outtakeLinkage = new ServoV2("outtake_linkage", hw);
        outtakeClaw = new ServoV2("outtake_claw", hw);
        outtakeWrist = new ServoV2("outtake_wrist", hw);

        LED = hw.get(Servo.class, "led");

        outtakePivotOne.setDirection(ServoV2.Direction.FORWARD);
        outtakePivotTwo.setDirection(ServoV2.Direction.REVERSE);
        outtakeLinkage.setDirection(ServoV2.Direction.REVERSE);

                setLED(1.0); //white
               setOuttakeClaw(outtakeClawPosition.CLOSED.pos);
                setOuttakeWrist(1);
                setLinkage(0.05);
                setPivot(0.5);
    }

    private static void setLED(double pos) {
        LED.setPosition(pos);
    }

    private static void setPivot(double pos) {
        outtakePivotOne.setPosition(pos);
        outtakePivotTwo.setPosition(pos);
    }

    private static void setLinkage(double pos) {
        outtakeLinkage.setPosition(pos);
    }

    private static void setClaw(double pos) {
        outtakeClaw.setPosition(pos);
    }

    private static void setOuttakeWrist(double pos) {
        outtakeWrist.setPosition(pos);
    }

    public static void setOuttakeClaw(double pos) {
        outtakeClaw.setPosition(pos);
    }


    public enum WristPosition {
        SPECIMEN(1), MID(0.5), SAMPLE(0),SAMPLESCORE(0.2);
        public final double pos;
        WristPosition(double pos) { this.pos = pos; }
    }

    public enum outtakeClawPosition {
        OPEN(.05), CLOSED(0.55);
        public final double pos;
        outtakeClawPosition(double pos) { this.pos = pos; }
    }

    public static Sequential extendArmSample() {
        return new Sequential(
                new Lambda("LED change").addRequirements(INSTANCE)
                        .setExecute(() -> setLED(0.279))
                        .setFinish(() -> true), // red
                new Lambda("pivot to 0.67").addRequirements(INSTANCE)
                        .setExecute(() -> setPivot(0.67))
                        .setFinish(() -> true),
                new Lambda("wrist to sample").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeWrist(WristPosition.SAMPLESCORE.pos))
                        .setFinish(() -> true),
                new Lambda("linkage to 0.15").addRequirements(INSTANCE)
                        .setExecute(() -> setLinkage(0.15))
                        .setFinish(() -> true),
                new Lambda("linkage to 0.36").addRequirements(INSTANCE)
                        .setExecute(() -> setLinkage(0.36))
                        .setFinish(() -> true),
                new Lambda("pivot to 0.8").addRequirements(INSTANCE)
                        .setExecute(() -> setPivot(0.82))
                        .setFinish(() -> true)

                );
    }


    public static Sequential outtakeClawClose() {
        return new Sequential(

                new Lambda("close claw").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeClaw((outtakeClawPosition.CLOSED.pos)))
                        .setFinish(() -> true)

        );
    }
    public static Sequential outtakeClawOpen() {
        return new Sequential(
                new Lambda("close claw").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeClaw((outtakeClawPosition.OPEN.pos)))
                        .setFinish(()-> true)
        );
    }


    public static Sequential retractArmSample() {
        return new Sequential(
                new Lambda("LED change").addRequirements(INSTANCE)
                        .setExecute(() -> setLED(0.5)), //green
                new Lambda("wrist for passthrough").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeWrist(WristPosition.SPECIMEN.pos)),
                new Lambda("linkage to 0.00").addRequirements(INSTANCE)
                        .setExecute(() -> setLinkage(0.04)),
                new Lambda("pivot to 0.5").addRequirements(INSTANCE)
                        .setExecute(() -> setPivot(0.60)),
                new Lambda("claw open").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeClaw(outtakeClawPosition.OPEN.pos)),
                new Lambda("wrist SPECIMEN").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeWrist(0.73))
        );
    }

    public static Sequential retractFromBasket() {
        return new Sequential(

                new Lambda("LED change").addRequirements(INSTANCE)
                        .setExecute(() -> setLED(0.5)), //green
                new Lambda("open    claw" ).addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeClaw(outtakeClawPosition.OPEN.pos)),
                new Lambda("wrist for passthrough").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeWrist(WristPosition.SPECIMEN.pos)),
                new Lambda("pivot to 0.5").addRequirements(INSTANCE)
                        .setExecute(() -> setPivot(0.70)),
                new Lambda("linkage to 0.00").addRequirements(INSTANCE)
                        .setExecute(() -> setLinkage(0.04))
        );
    }
    public static Parallel retractFromChamber() {
        return new Parallel(

                new Lambda("open claw" ).addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeClaw(outtakeClawPosition.OPEN.pos)),
                new Lambda("open claw" ).addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeClaw(WristPosition.SAMPLE.pos)),
                new Lambda("linkage to 0.00").addRequirements(INSTANCE)
                        .setExecute(() -> setLinkage(0.04)),
                new Lambda("pivot to 0.5").addRequirements(INSTANCE)
                        .setExecute(() -> setPivot(0.65))
                        .setFinish(() -> true)


        );
    }

    public static Sequential grabSpecimen() {
        return new Sequential(
                new Lambda("LED change").addRequirements(INSTANCE)
                        .setExecute(() -> setLED(0.611)) //blue
                        .setFinish(() -> true),
                new Lambda("wrist to SPECIMEN").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeClaw(outtakeClawPosition.OPEN.pos))
                        .setFinish(() -> true),
                new Lambda("linkage to 0.00").addRequirements(INSTANCE)
                        .setExecute(() -> setLinkage(0.03))
                        .setFinish(() -> true),
                new Lambda("wrist to SPECIMEN").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeWrist(WristPosition.SPECIMEN.pos))
                        .setFinish(() -> true),
                new Lambda("pivot to 0.02").addRequirements(INSTANCE)
                        .setExecute(() -> setPivot(0.03))
                        .setFinish(() -> true),
                new Lambda("claw OPEN").addRequirements(INSTANCE)
                        .setExecute(() -> setClaw(outtakeClawPosition.OPEN.pos))
                        .setFinish(() -> true),
                new Wait(.4),
                new Lambda("wrist to .7").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeWrist(0.73))
                        .setFinish(() -> true)

        );
    }

    public static Parallel scoreSpecimenAuto() {
        return new Parallel(
                new Lambda("LED change").addRequirements(INSTANCE)
                        .setExecute(() -> setLED(0.722))
                        .setFinish(() -> true),// violet
                new Lambda("pivot to 0.5").addRequirements(INSTANCE)
                        .setExecute(() -> setPivot(0.52))
                        .setFinish(() -> true),
                new Lambda("linkage to 0.32").addRequirements(INSTANCE)
                        .setExecute(() -> setLinkage(0.37))
                        .setFinish(() -> true),
                new Lambda("wrist to .7").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeWrist(0.8))
                        .setFinish(() -> true),
                new Lambda("mark state EXTENDED_SPEC").addRequirements(INSTANCE)
                        .setExecute(() -> clawStates.setState(OuttakeStates.EXTENDED_SPEC))
                        .setFinish(() -> true)
        );
    }

    public static Sequential scoreSpecimen() {
        return new Sequential(

                new Lambda("LED change").addRequirements(INSTANCE)
                        .setExecute(() -> setLED(0.722)), // violet
                new Lambda("claw CLOSED").addRequirements(INSTANCE)
                        .setExecute(() -> setClaw(outtakeClawPosition.CLOSED.pos)),
                new Lambda("wrist SPECIMEN").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeWrist(WristPosition.SPECIMEN.pos)),
                new Wait(.2),
                new Lambda("pivot to 0.5").addRequirements(INSTANCE)
                        .setExecute(() -> setPivot(0.52)),

                new Wait(.2),
                new Lambda("linkage to 0.32").addRequirements(INSTANCE)
                        .setExecute(() -> setLinkage(0.37)),
                new Lambda(" wrist to score").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeWrist(0.8))

        );
    }
    public static Sequential transferSample() {
        return new Sequential(
                new Lambda("LED change").addRequirements(INSTANCE)
                        .setExecute(() -> setLED(1.0)), //white
                new Lambda("claw open").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeClaw(outtakeClawPosition.OPEN.pos)),

                new Lambda("wrist SPECIMEN").addRequirements(INSTANCE)
                        .setExecute(() -> setOuttakeWrist(0.75)),
                new Lambda("linkage to 0.00").addRequirements(INSTANCE)
                        .setExecute(() -> setLinkage(0.02)),
                new Wait(0.3),
                new Lambda("pivot to 0.5").addRequirements(INSTANCE)
                        .setExecute(() -> setPivot(0.59))

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
}
