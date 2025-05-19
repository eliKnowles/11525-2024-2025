package org.firstinspires.ftc.teamcode.code;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VSlide;
import org.firstinspires.ftc.teamcode.code.util.Drive;
import org.firstinspires.ftc.teamcode.code.util.Outtake;

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
                                new Parallel(

                                        Outtake.scoreSpecimen(),
                                        VSlide.goTo(490)
                                ).schedule();
                            } else if (!Outtake.isSpecMode() && Outtake.getClawStates().getState() == Outtake.OuttakeStates.TRANSFER_SAMPLE) {
                                new Parallel(
                                        Outtake.extendArmSample(),
                                        VSlide.goTo(630)
                                ).schedule();
                            }
                        })
                        .setFinish(() -> true)
        );

        // RETRACT button
        Mercurial.gamepad1().square().onTrue(
                new Lambda("Grab Position")
                        .setExecute(() -> {
                            if (Outtake.isSpecMode()) {
                                new Parallel(
                                        Intake.intakeSpecimen(),
                                        HSlide.goTo(0),
                                        new Wait(.2),
                                        Outtake.grabSpecimen(),
                                        new Wait(.4),
                                        VSlide.goTo(0, 0.2)
                                ).schedule();
                            } else  {
                                new Sequential(
                                        Outtake.retractArmSample(),
                                        new Wait(.1),
                                        VSlide.goTo(0, 0.2)
                                ).schedule();
                            }
                        })
                        .setFinish(() -> true)
        );

        Mercurial.gamepad1().b().onTrue(
                new Lambda("extend intake")
                        .setExecute(() -> {
                            if (Outtake.getClawStates().getState() == Outtake.OuttakeStates.RETRACTED_SAMPLE || Outtake.getClawStates().getState() ==  Outtake.OuttakeStates.RETRACTED_SAMPLE) {
                                new Parallel(HSlide.goTo(480),
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
                                        new Parallel (
                                                Outtake.transferSample(),
                                                Intake.intakeGrab(),
                                                Intake.runTransfer(),
                                                new Wait(.1),
                                                HSlide.goTo(0),
                                                new Wait(.1),
                                                Outtake.outtakeClawClose(),
                                                new Wait(.05),
                                                Intake.intakeClawOpen(),
                                                new Wait(.1),
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
        telemetry.addData("VSlide Position", VSlide.INSTANCE.encoder.getCurrentPosition());
        telemetry.addData("spec mode:", Outtake.isSpecMode());

        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
        telemetry.update();
        if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
            Intake.resetWrist();
        }
    }
}
