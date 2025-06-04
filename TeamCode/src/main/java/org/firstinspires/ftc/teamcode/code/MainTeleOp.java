package org.firstinspires.ftc.teamcode.code;




import static org.firstinspires.ftc.teamcode.code.subsystem.Drive.follower;
import static org.firstinspires.ftc.teamcode.code.subsystem.HSlide.INSTANCE;
import static org.firstinspires.ftc.teamcode.code.subsystem.Intake.limelightSearch;
import static org.firstinspires.ftc.teamcode.code.subsystem.Intake.pin0;
import static org.firstinspires.ftc.teamcode.code.subsystem.Intake.pin1;
import static org.firstinspires.ftc.teamcode.code.subsystem.Outtake.clawStates;
import static org.firstinspires.ftc.teamcode.code.subsystem.Outtake.isSpecMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.code.limelight.Limelight;
import org.firstinspires.ftc.teamcode.code.limelight.ScanForSample;
import org.firstinspires.ftc.teamcode.code.limelight.SearchForever;
import org.firstinspires.ftc.teamcode.code.subsystem.HSlide;
import org.firstinspires.ftc.teamcode.code.subsystem.Intake;
import org.firstinspires.ftc.teamcode.code.subsystem.VSlide;
import org.firstinspires.ftc.teamcode.code.subsystem.Drive;
import org.firstinspires.ftc.teamcode.code.subsystem.Outtake;


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

    @Override
    public void init() {
        this.buffer = new Limelight.SampleState();
        this.limelight = new Limelight(hardwareMap);

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
                                                new Lambda("Set slide target").setExecute(() -> VSlide.setTarget(19200, 1)).setFinish(() -> true),
                                                Outtake.scoreSpecimen()
                                        ),
                                        //Drive.nerfDrive(),
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
                                        Outtake.extendArmSample(),
                                        VSlide.goTo(26000)
                                     //   Drive.nerfDrive()

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
                                            Outtake.grabSpecimen(),
                                           // Drive.normalDrive(),
                                            VSlide.goTo(0, 0.6),
                                                    new Lambda("mark state SPECIMEN_WALL")
                                                            .setExecute(() -> clawStates.setState(Outtake.OuttakeStates.SPECIMEN_WALL))
                                                            .setFinish(() -> true)
                                    ).schedule();
                                }
                            } else if (Outtake.getClawStates().getState() == Outtake.OuttakeStates.EXTENDED_SAMPLE || !isSpecMode() && (Outtake.getClawStates().getState() == Outtake.OuttakeStates.SPECIMEN_WALL)) { // retract if its at the basket position
                                new Sequential(
                                        Outtake.retractFromBasket(),
                                      //  Drive.normalDrive(),
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
                                new Parallel(HSlide.goTo(13500),
                                        Intake.runExtend(),
                                     //   Drive.nerfDrive(),
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
                               // Drive.normalDrive(),
                                new Sequential(
                                        Intake.runTransfer(),
                                        HSlide.goTo(0),
                                        Outtake.outtakeClawClose(),
                                        Intake.intakeClawOpen(),
                                        Intake.intakeSpecimen(),
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

        Mercurial.gamepad1().dpadUp().onTrue(
                new Sequential(
                        Intake.limelightSearch(),
//                        new SearchForever(follower).raceWith(
                                new ScanForSample(limelight, buffer, telemetry, follower, false)
//                        )
                )

        );
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
    }
}
