package org.firstinspires.ftc.teamcode.code;




import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.code.subsystem.HSlide;
import org.firstinspires.ftc.teamcode.code.subsystem.Intake;
import org.firstinspires.ftc.teamcode.code.subsystem.VSlide;
import org.firstinspires.ftc.teamcode.code.subsystem.Drive;
import org.firstinspires.ftc.teamcode.code.subsystem.Outtake;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@TeleOp
@VSlide.Attach
@Outtake.Attach
@Intake.Attach
@HSlide.Attach
@Mercurial.Attach
@Drive.Attach
public class MainTeleOp extends OpMode {

    @Override
    public void init() {
        HSlide.INSTANCE.setDefaultCommand(HSlide.update());
        VSlide.INSTANCE.setDefaultCommand(VSlide.update());

        Mercurial.gamepad1().triangle().onTrue(
                new Lambda("Score Position")
                        .setExecute(() -> {
                            if (Outtake.isSpecMode() && Outtake.getClawStates().getState() == Outtake.OuttakeStates.SPECIMEN_WALL) {
                                new Sequential(Outtake.outtakeClawClose(),
                                        new Wait(.2),
                                        new Parallel(Outtake.scoreSpecimen(),
                                                VSlide.goTo(20000))
                                ).schedule();
                            } else if (!Outtake.isSpecMode() && Outtake.getClawStates().getState() == Outtake.OuttakeStates.TRANSFER_SAMPLE) {
                                new Parallel(
                                        Outtake.extendArmSample(),
                                        VSlide.goTo(26000)
                                ).schedule();
                            }
                        })
                        .setFinish(() -> true)
        );

        // RETRACT button
        Mercurial.gamepad1().square().onTrue(
                new Lambda("Grab Position")
                        .setInterruptible(true)
                        .setExecute(() -> {
                            if (Outtake.isSpecMode() && Outtake.getClawStates().getState() == Outtake.OuttakeStates.RETRACTED_SPEC ||Outtake.getClawStates().getState() == Outtake.OuttakeStates.RETRACTED_SAMPLE || Outtake.getClawStates().getState() == Outtake.OuttakeStates.EXTENDED_SPEC ) {
                                new Sequential(
                                        Outtake.outtakeClawOpen(),
                                        new Wait(.3),
                                        new Parallel(
                                                Outtake.grabSpecimen(),
                                                VSlide.goTo(0, 0.6)
                                        )
                                ).schedule();
                            } else if (Outtake.getClawStates().getState() == Outtake.OuttakeStates.EXTENDED_SAMPLE) {
                                new Sequential(
                                        Outtake.retractFromBasket(),
                                        new Wait(.5),
                                        VSlide.goTo(0, 0.6),
                                        Outtake.retractArmSample()
                                ).schedule();
                            }
                        })
                        .setFinish(() -> true)
        );

        Mercurial.gamepad1().b().onTrue(
                new Lambda("extend intake")
                        .setExecute(() -> {
                            if (Outtake.getClawStates().getState() == Outtake.OuttakeStates.RETRACTED_SAMPLE || Outtake.getClawStates().getState() ==  Outtake.OuttakeStates.RETRACTED_SAMPLE) {
                                new Parallel(HSlide.goTo(13500),
                                        Intake.runExtend()
                                ).schedule();
                            }
                        })
                        .setFinish(() -> true)
        );


        Mercurial.gamepad1().a().onTrue(
                new Lambda("Transfer")
                        .setExecute(() -> {
                            if (Outtake.getClawStates().getState() == Outtake.OuttakeStates.RETRACTED_SAMPLE) {
                                        new Sequential (
                                                Intake.intakeGrab(),
                                                Intake.runTransfer(),
                                                HSlide.goTo(0),
                                                Outtake.outtakeClawClose(),
                                                Intake.intakeClawOpen(),
                                                Intake.intakeSpecimen()
                                ).schedule();
                            }
                        })
                        .setFinish(() -> true)
        );
        // LEFT BUMPER: move wrist left
        Mercurial.gamepad1().leftBumper().onTrue(
                new Lambda("Wrist Left")
                        .setExecute(() -> Intake.adjustWrist(-1))
                        .setFinish(() -> true)
        );

        Mercurial.gamepad1().rightBumper().onTrue(
                new Lambda("Wrist Right")
                        .setExecute(() -> Intake.adjustWrist(1))
                        .setFinish(() -> true)
        );


        // TOGGLE button
        Mercurial.gamepad1().share().onTrue(
                Outtake.toggleMode()
        );
    }

    @Override
    public void loop() {
        Drive.follower.update();


        telemetry.addData("VSlide Position", VSlide.getPosition());
        telemetry.addData("Extendo Position", HSlide.getPosition());

        telemetry.addData("spec mode:", Outtake.isSpecMode());

        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
        telemetry.update();
        if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
            Intake.resetWrist();
        }
    }
}
