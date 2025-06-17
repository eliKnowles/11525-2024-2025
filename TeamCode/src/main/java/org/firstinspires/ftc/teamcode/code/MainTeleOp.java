package org.firstinspires.ftc.teamcode.code;

import static org.firstinspires.ftc.teamcode.code.subsystem.Drive.follower;
import static org.firstinspires.ftc.teamcode.code.subsystem.HSlide.INSTANCE;
import static org.firstinspires.ftc.teamcode.code.subsystem.Intake.pin0;
import static org.firstinspires.ftc.teamcode.code.subsystem.Intake.pin1;
import static org.firstinspires.ftc.teamcode.code.subsystem.Outtake.clawStates;
import static org.firstinspires.ftc.teamcode.code.subsystem.Outtake.isSpecMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.code.limelight.Limelight;
import org.firstinspires.ftc.teamcode.code.limelight.LimelightCV;
import org.firstinspires.ftc.teamcode.code.subsystem.Drive;
import org.firstinspires.ftc.teamcode.code.subsystem.HSlide;
import org.firstinspires.ftc.teamcode.code.subsystem.Intake;
import org.firstinspires.ftc.teamcode.code.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.code.subsystem.VSlide;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.IfElse;
import dev.frozenmilk.mercurial.commands.util.Wait;

@TeleOp
@VSlide.Attach
@Outtake.Attach
@Intake.Attach
@HSlide.Attach
@Mercurial.Attach
@Drive.Attach
public class MainTeleOp extends OpMode {

    private Limelight limelight;
    private Limelight.SampleState buffer;

    private LimelightCV llCV;

    private boolean stopHangSceduled = false;
    private boolean limelightScheduled, grabScheduled = false;

    @Override
    public void init() {
        this.buffer = new Limelight.SampleState();
        this.limelight = new Limelight(hardwareMap);

        this.llCV = new LimelightCV(hardwareMap, follower);

        clawStates.setState(isSpecMode() ? Outtake.OuttakeStates.RETRACTED_SPEC : Outtake.OuttakeStates.RETRACTED_SAMPLE);

        INSTANCE.setDefaultCommand(HSlide.update());
        VSlide.INSTANCE.setDefaultCommand(VSlide.update());

        Mercurial.gamepad1().triangle().onTrue(
                new Lambda("Score Position")
                        .setExecute(() -> {
                            if (Outtake.isSpecMode() && Outtake.getClawStates().getState() == Outtake.OuttakeStates.SPECIMEN_WALL) {

                                new Sequential(
                                        Outtake.outtakeClawClose(),
                                        new Wait(.2),
                                        new Parallel(
                                                new Lambda("Set slide target").setExecute(() -> VSlide.setTarget(19800, 1)).setFinish(() -> true),
                                                Outtake.scoreSpecimen()
                                        ),
                                        Drive.nerfDrive(),
                                        new Lambda("mark state EXTENDED_SPEC")
                                                .setExecute(() -> clawStates.setState(Outtake.OuttakeStates.EXTENDED_SPEC))
                                                .setFinish(() -> true)
                                        ).schedule();
                            }
                            else if (!Outtake.isSpecMode() && Outtake.getClawStates().getState() == Outtake.OuttakeStates.TRANSFER_SAMPLE) {
                                new Parallel(
                                        new Lambda("mark state SPECIMEN_WALL")
                                                .setExecute(() -> clawStates.setState(Outtake.OuttakeStates.EXTENDED_SAMPLE))
                                                .setFinish(() -> true),
                                        Intake.intakeSpecimen(),
                                        Outtake.extendArmSample(),
                                        VSlide.goTo(27500),
                                        Drive.nerfDrive()

                                ).schedule();
                            }
                        })
                        .setFinish(() -> true)
        );

        // RETRACT button
        Mercurial.gamepad1().square().onTrue(
                new Lambda("Grab Position")
                        .setExecute(() -> {
                            if (isSpecMode() &&
                                    (Outtake.getClawStates().getState() == Outtake.OuttakeStates.RETRACTED_SPEC ||Outtake.getClawStates().getState() == Outtake.OuttakeStates.RETRACTED_SAMPLE ||
                                            Outtake.getClawStates().getState() == Outtake.OuttakeStates.EXTENDED_SPEC)) {
                                {
                                    new Sequential(
                                            Intake.intakeSpecimen(),
                                            Outtake.grabSpecimen(),
                                            Drive.nerfDrive(),
                                            VSlide.goTo(0, 0.6),
                                                    new Lambda("mark state SPECIMEN_WALL")
                                                            .setExecute(() -> clawStates.setState(Outtake.OuttakeStates.SPECIMEN_WALL))
                                                            .setFinish(() -> true)
                                    ).schedule();
                                }
                            } else if (Outtake.getClawStates().getState() == Outtake.OuttakeStates.EXTENDED_SAMPLE || !isSpecMode() && (Outtake.getClawStates().getState() == Outtake.OuttakeStates.SPECIMEN_WALL)) { // retract if its at the basket position
                                new Sequential(
                                        Outtake.retractFromBasket(),
                                        Drive.normalDrive(),
                                        new Wait(.4),
                                        new Lambda("mark state SPECIMEN_WALL")
                                                .setExecute(() -> clawStates.setState(Outtake.OuttakeStates.RETRACTED_SAMPLE))
                                                .setFinish(() -> true),
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
                            if (Outtake.getClawStates().getState() == Outtake.OuttakeStates.RETRACTED_SAMPLE && !isSpecMode()) { // not spec mode and outtake is in sample pos
                                new Parallel(
                                        Drive.nerfDrive(),
                                        HSlide.goTo(13500),
                                        Intake.runExtend(),
                                        new Lambda("Set READY_FOR_TRANSFER")
                                                .setExecute(() -> clawStates.setState(Outtake.OuttakeStates.EXTENDED_INTAKE))
                                                .setFinish(() -> true)
                                ).schedule();
                            }
                        })
                        .setFinish(() -> true)
        );

        Mercurial.gamepad1().a().onTrue(
                new Sequential(
                        Intake.intakeGrab(),
                        new IfElse(
                                Intake::hasSample, // condition: sample detected
                                // TRUE: run full transfer
                                new Parallel(
                                        Outtake.retractArmSample(),
                                        Outtake.outtakeClawOpen(),
                                        VSlide.goTo(0),
                                        Drive.normalDrive(),
                                        new Sequential(
                                                Intake.chamberTransfer(),
                                                HSlide.goTo(0),
                                                Intake.runTransfer(),
                                                new Wait(.4),
                                                Outtake.outtakeClawClose(),
                                                Intake.intakeClawOpen(),
                                                //  Intake.intakeSpecimen(),
                                                new Lambda("Transfer done")
                                                        .setExecute(() -> clawStates.setState(Outtake.OuttakeStates.TRANSFER_SAMPLE))
                                                        .setFinish(() -> true)
                                        )
                                ),
                                // FALSE: re-extend intake
                                new Parallel(
                                        Intake.intakeClawOpen(),
                                        Intake.runExtend(),
                                        HSlide.goTo(13500),
                                        new Lambda("Re-extend intake")
                                                .setExecute(() -> clawStates.setState(Outtake.OuttakeStates.EXTENDED_INTAKE))
                                                .setFinish(() -> true)
                                )
                        )
                )
        );

        Mercurial.gamepad1().options().onTrue(
                new Lambda("Wrist Left")
                        .setExecute(Outtake::resetExtendo)
                        .setFinish(() -> true)
        );

        Mercurial.gamepad1().dpadLeft().onTrue(
                new Parallel(
                        HSlide.goTo(0),
                        Intake.intakeSpecimen(),
                        new Lambda("mark state SPECIMEN_WALL")
                                .setExecute(() -> clawStates.setState(Outtake.OuttakeStates.RETRACTED_SAMPLE))
                                .setFinish(() -> true)
                )
        );

        Mercurial.gamepad2().leftBumper().onTrue(
                Intake.hang_1()
        );

        Mercurial.gamepad2().rightBumper().onTrue(
                Intake.hang_2()
        );

        Mercurial.gamepad2().leftBumper().and(() -> gamepad2.right_bumper).onTrue(
                new Parallel(
                        Intake.hang_1(),
                        Intake.hang_2()
                )
        );

        Mercurial.gamepad2().triangle().whileTrue(
                new Parallel(
                        Intake.intakeSpecimen(),
                        Outtake.grabSpecimen(),
                        Intake.hangDeploy()
                )
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

        //HSlides
        Mercurial.gamepad2().dpadDown().onTrue(
                HSlide.changeOffset(-200)
        );

        Mercurial.gamepad2().dpadUp().onTrue(
                HSlide.zeroEncoder()
        );

      //  Mercurial.gamepad1().dpadUp().onTrue(
        //        Drive.followPathChain(llCV.align())
//                new Parallel(
//                        Intake.limelightSearch(),
////                        new SearchForever(follower).raceWith(
//                                new ScanForSample(limelight, buffer, follower, false)
////                        )
//                )
    //    );
    }


    @Override
    public void loop() {
        follower.update();
        telemetry.addData("Claw State", clawStates.getState());
        telemetry.addData("VSlide Position", VSlide.getPosition());
        telemetry.addData("Extendo Position", HSlide.getPosition());

        telemetry.addData("speed:", Drive.getSpeed());

        telemetry.addData("digital 0", pin0.getState());
        telemetry.addData("digital 1", pin1.getState());

        telemetry.addData("spec mode:", isSpecMode());
        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
        telemetry.update();
        if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
            Intake.resetWrist();
        }
        if (!gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.triangle && !stopHangSceduled) {
            Intake.stopHang();
            stopHangSceduled = true;
        } else {
            stopHangSceduled = false;
        }

        if (gamepad1.right_trigger > 0.75 && !limelightScheduled) {
            limelightScheduled = true;
            new Lambda("Score Position")
                    .setExecute(() -> {
                        if (Outtake.getClawStates().getState() == Outtake.OuttakeStates.TRANSFER_SAMPLE) {
                            new Sequential(
                                    Outtake.dropSample(),
                                    new Wait(.2),
                                    Outtake.outtakeClawOpen(),
                                    new Lambda("mark state EXTENDED_SPEC")
                                            .setExecute(() -> clawStates.setState(Outtake.OuttakeStates.RETRACTED_SAMPLE))
                                            .setFinish(() -> true)
                            ).schedule();
                        }
                    })
                    .setFinish(() -> true)
                    .schedule();
        } else if (gamepad1.right_trigger < 0.1) {
            limelightScheduled = false;
        }

        if (gamepad1.left_trigger > 0.75 && !grabScheduled) {
            grabScheduled = true;
            new Sequential(
                    Intake.intakeGrab(),
                    new IfElse(
                            Intake::hasSample, // condition: sample detected
                            // TRUE: run full transfer
                            new Parallel(
                                    Outtake.retractArmSample(),
                                    Outtake.outtakeClawOpen(),
                                    Drive.normalDrive(),
                                    new Sequential(
                                            Intake.runTransfer(),
                                            HSlide.goTo(0),
                                            Outtake.outtakeClawClose(),
                                            Intake.intakeClawOpen(),
                                            //  Intake.intakeSpecimen(),
                                            new Lambda("Transfer done")
                                                    .setExecute(() -> clawStates.setState(Outtake.OuttakeStates.TRANSFER_SAMPLE))
                                                    .setFinish(() -> true)
                                    )
                            ),
                            // FALSE: re-extend intake
                            new Parallel(
                                    Intake.intakeClawOpen(),
                                    Intake.runExtend(),
                                    HSlide.goTo(13500),
                                    new Lambda("Re-extend intake")
                                            .setExecute(() -> clawStates.setState(Outtake.OuttakeStates.EXTENDED_INTAKE))
                                            .setFinish(() -> true)
                            )
                    )
            ).schedule();
        } else if (gamepad1.left_trigger < 0.1) {
            grabScheduled = false;
        }
    }
}
