package org.firstinspires.ftc.teamcode.hermeshelper.util.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hermeshelper.util.VSlide;

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

public class Outtake implements Subsystem {
    public static final Outtake INSTANCE = new Outtake();

    private ServoV2 outtakePivotOne;
    private ServoV2 outtakePivotTwo;
    private ServoV2 outtakeLinkage;
    private ServoV2 outtakeClaw;
    private ServoV2 outtakeWrist;


    public enum OuttakeState {
        RETRACTED,
        EXTENDED,
        SPECIMEN_WALL,
        EXTENDED_SPEC,
        RETRACTED_SPEC
    }

    private OuttakeState currentState = OuttakeState.RETRACTED;

    public OuttakeState getState() {
        return currentState;
    }

    public void setState(OuttakeState newState) {
        this.currentState = newState;
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
    }

    private void setPivot(double pos) {
        outtakePivotOne.setPosition(pos);
        outtakePivotTwo.setPosition(pos);
    }

    private void setLinkage(double pos) {
        outtakeLinkage.setPosition(pos);
    }

    public enum WristPosition {
        DOWN(0.0), MID(0.5), UP(1.0);
        public final double pos;
        WristPosition(double pos) { this.pos = pos; }
    }

    public enum ClawPosition {
        OPEN(0.2), CLOSED(0);
        public final double pos;
        ClawPosition(double pos) { this.pos = pos; }
    }



    public static Sequential extendArmSample() {
        return new Sequential(
                new Lambda("pivot to 0.65")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setPivot(0.67)),
                new Lambda("linkage to .1")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setLinkage(0.15)),

                new Wait(0.1),

                new Lambda("linkage to 0.3")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setLinkage(0.36)),

                new Lambda("pivot to 0.8")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setPivot(0.8)),

                new Lambda("mark state EXTENDED")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setState(OuttakeState.EXTENDED))
        );
    }

    public static Sequential retractArmSample() {
        return new Sequential(
                new Lambda("linkage to 0.1")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setLinkage(0.05)),

                new Wait(0.1),

                new Lambda("pivot to 0.5")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setPivot(0.5)),

                new Lambda("mark state RETRACTED")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setState(OuttakeState.RETRACTED))
        );
    }

    public static Sequential grabSpecimen() {
        return new Sequential(
                new Lambda("linkage to 0")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setLinkage(0.00)),
                new Lambda("linkage to 0")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.),

                new Wait(0.2),

                new Lambda("pivot to 0.1")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setPivot(0)),

                new Lambda("mark state RETRACTED")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setState(OuttakeState.SPECIMEN_WALL))
        );
    }

    public static Sequential scoreSpecimen() {
        return new Sequential(
                new Lambda("claw to closed")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setLinkage(0.00)),

                new Lambda("linkage to 0")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setLinkage(0.00)),

                new Wait(0.2),

                new Lambda("pivot to 0.1")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setPivot(.5)),
                new Wait(.3),
                new Lambda("pivot to 0.1")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setLinkage(.32)),

                new Lambda("mark state RETRACTED")
                        .addRequirements(INSTANCE)
                        .setExecute(() -> INSTANCE.setState(OuttakeState.RETRACTED_SPEC))
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
