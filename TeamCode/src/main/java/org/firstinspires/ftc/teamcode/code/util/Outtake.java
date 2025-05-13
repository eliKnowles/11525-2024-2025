package org.firstinspires.ftc.teamcode.code.util;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.StateMachine;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.mercurial.subsystems.Subsystem;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

public class Outtake implements Subsystem {
    public static final Outtake INSTANCE = new Outtake();

    private static ServoV2 outtakePivotOne;
    private static ServoV2 outtakePivotTwo;
    private static ServoV2 outtakeLinkage;
    private static ServoV2 outtakeClaw;
    private static ServoV2 outtakeWrist;

    private static final StateMachine<OuttakeStates> clawStates = new StateMachine<>(OuttakeStates.RETRACTED)
            .withState(OuttakeStates.EXTENDED_SPEC, (stateRef, name) -> specExtend())
            .withState(OuttakeStates.SPECIMEN_WALL, (stateRef, name) -> specRetract())
            .withState(OuttakeStates.EXTENDED, (stateRef, name) -> sampleExtend())
            .withState(OuttakeStates.RETRACTED, (stateRef, name) -> sampleRetract());

    public enum OuttakeStates {
        RETRACTED,
        EXTENDED,
        SPECIMEN_WALL,
        EXTENDED_SPEC,
        RETRACTED_SPEC
    }

    public OuttakeStates getState() {
        return clawStates.getState();
    }

    public static Lambda sampleExtend() {
        return new Lambda("extend-sample")
                .addRequirements(INSTANCE)
                .setInterruptible(false)
                .setInit(Outtake::extendArmSample);
    }

    public static Lambda sampleRetract() {
        return new Lambda("retract-sample")
                .addRequirements(INSTANCE)
                .setInterruptible(false)
                .setInit(Outtake::retractArmSample);
    }

    public static Lambda specExtend() {
        return new Lambda("extend-spec")
                .addRequirements(INSTANCE)
                .setInterruptible(false)
                .setInit(Outtake::scoreSpecimen);
    }

    public static Lambda specRetract() {
        return new Lambda("retract-spec")
                .addRequirements(INSTANCE)
                .setInterruptible(false)
                .setInit(Outtake::grabSpecimen);
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @Inherited
    public @interface Attach {}

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    private Outtake() {}

    @Override
    public void preUserInitHook(@NonNull Wrapper opMode) {
        HardwareMap hw = opMode.getOpMode().hardwareMap;

        outtakePivotOne = new ServoV2("outtake_pivot_one", hw);
        outtakePivotTwo = new ServoV2("outtake_pivot_two", hw);
        outtakeLinkage = new ServoV2("outtake_linkage", hw);
        outtakeClaw = new ServoV2("outtake_claw", hw);
        outtakeWrist = new ServoV2("outtake_wrist", hw);


        outtakePivotTwo.setDirection(ServoV2.Direction.REVERSE);
        outtakeLinkage.setDirection(ServoV2.Direction.REVERSE);

        outtakePivotOne.setPosition(0.5);
        outtakePivotTwo.setPosition(0.5);
        outtakeLinkage.setPosition(0.1);
        outtakeWrist.setPosition(WristPosition.MID.pos);

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

    private static void setOuttakeClaw(double pos) {
        outtakeClaw.setPosition(pos);
    }


    public enum WristPosition {
        SPECIMEN(1), MID(0.5), SAMPLE(0);
        public final double pos;
        WristPosition(double pos) { this.pos = pos; }
    }

    public enum ClawPosition {
        OPEN(0.4), CLOSED(0.7);
        public final double pos;
        ClawPosition(double pos) { this.pos = pos; }
    }

    public static void extendArmSample() {
        setPivot(0.67);
        setOuttakeWrist(WristPosition.SAMPLE.pos);
        setLinkage(0.15);
        new Wait(0.1);
        setLinkage(0.36);
        setPivot(0.8);
        clawStates.setState(OuttakeStates.EXTENDED);
    }

    public static void retractArmSample() {
        setLinkage(0.05);
        setPivot(0.5);
        clawStates.setState(OuttakeStates.RETRACTED);
    }

    public static void grabSpecimen() {
        setOuttakeWrist(WristPosition.SPECIMEN.pos);
        setOuttakeClaw(ClawPosition.CLOSED.pos);
        setLinkage(0.00);
        new Wait(0.6);
        setPivot(0.02);
        new Wait(0.6);
        setClaw(ClawPosition.OPEN.pos);
        setOuttakeWrist(.7);
        clawStates.setState(OuttakeStates.SPECIMEN_WALL);
    }

    public static void scoreSpecimen() {
        setClaw(ClawPosition.CLOSED.pos);
        setOuttakeWrist(WristPosition.SPECIMEN.pos);
        setLinkage(0.00);
        new Wait(0.2);
        setPivot(0.5);
        new Wait(0.3);
        setLinkage(0.32);
        clawStates.setState(OuttakeStates.EXTENDED_SPEC);
    }

//    public static Sequential extendArmSample() {
//        return new Sequential(
//                new Lambda("pivot to 0.67")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setPivot(0.67)),
//                new Lambda("pivot to 0.67")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setOuttakeWrist(WristPosition.SAMPLE.pos)),
//                new Lambda("linkage to 0.15")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setLinkage(0.15)),
//                new Wait(0.1),
//                new Lambda("linkage to 0.36")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setLinkage(0.36)),
//                new Lambda("pivot to 0.8")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setPivot(0.8)),
//                new Lambda("mark state EXTENDED")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setState(OuttakeStates.EXTENDED))
//        );
//    }


//    public static Sequential retractArmSample() {
//        return new Sequential(
//                new Lambda("linkage to 0.05")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setLinkage(0.05)),
//                new Wait(0.1),
//                new Lambda("pivot to 0.5")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setPivot(0.5)),
//                new Lambda("mark state RETRACTED")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setState(OuttakeStates.RETRACTED))
//        );
//    }

//    public static Sequential grabSpecimen() {
//        return new Sequential(
//                new Lambda("pivot to 0.0")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setOuttakeWrist(WristPosition.SPECIMEN.pos)),
//                new Lambda("pivot to 0.0")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setOuttakeClaw(.4)),
//                new Lambda("linkage to 0.00")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setLinkage(0.00)),
//                new Wait(0.6),
//                new Lambda("pivot to 0.0")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setPivot(0.02)),
//                new Wait(0.6),
//
//                new Lambda("pivot to 0.0")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setClaw(.3)),
//
//                new Lambda("pivot to 0.0")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setOuttakeWrist(.7)),
//                new Lambda("mark state SPECIMEN_WALL")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setState(OuttakeStates.SPECIMEN_WALL))
//        );
//    }

//    public static Sequential scoreSpecimen() {
//        return new Sequential(
//                new Lambda("claw to closed")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setClaw(ClawPosition.OPEN.pos)),
//                new Lambda("claw to closed")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setOuttakeWrist(WristPosition.SPECIMEN.pos)),
//
//                new Lambda("linkage to 0.00")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setLinkage(0.00)),
//                new Wait(0.2),
//                new Lambda("pivot to 0.5")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setPivot(0.5)),
//                new Wait(0.3),
//                new Lambda("linkage to 0.32")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setLinkage(0.32)),
//                new Lambda("mark state RETRACTED_SPEC")
//                        .addRequirements(INSTANCE)
//                        .setExecute(() -> setState(OuttakeStates.RETRACTED_SPEC))
//        );
//    }

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
