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
                                new Parallel(
                                        new Wait(.5),
                                        Outtake.retractArmSample(),
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
                                        new Sequential (
                                                Intake.intakeGrab(),
                                                Outtake.transferSample(),
                                                Intake.runTransfer(),
                                                HSlide.goTo(0),
                                                new Wait(.1),
                                                Outtake.outtakeClawClose(),
                                                new Wait(.05),
                                                Intake.intakeClawOpen(),
                                            Intake.intakeSpecimen()
                                ).schedule();
                            }
                        })
                        .setFinish(() -> true)
        );



        // TOGGLE button
        Mercurial.gamepad1().share().onTrue(
                Outtake.toggleMode()
        );
    }

//                new IfElse(
//                        () -> Outtake.INSTANCE.getState() == Outtake.OuttakeState.RETRACTED_SAMPLE,
//                        Outtake.extendArmSample(),
//                        new Lambda("noop")
//                                .addRequirements(Outtake.INSTANCE)
//                                .setExecute(() -> {})
//                )



//                new IfElse(
//                        () -> Outtake.INSTANCE.getState() == Outtake.OuttakeState.EXTENDED || Outtake.INSTANCE.getState()== Outtake.OuttakeState.RETRACTED_SPEC,
//                        Outtake.retractArmSample(),
//                        new Lambda("noop")
//                                .addRequirements(Outtake.INSTANCE)
//                                .setExecute(() -> {})
//                )



//                new IfElse(
//                        () -> Outtake.INSTANCE.getState() == Outtake.OuttakeState.RETRACTED_SAMPLE || Outtake.INSTANCE.getState()== Outtake.OuttakeState.RETRACTED_SPEC,
//                        Outtake.grabSpecimen(),
//                        new Lambda("noop")
//                                .addRequirements(Outtake.INSTANCE)
//                                .setExecute(() -> {})
//                )



//                new IfElse(
//                        () -> Outtake.INSTANCE.getState() == Outtake.OuttakeState.SPECIMEN_WALL,
//                        Outtake.scoreSpecimen(),
//                        new Lambda("noop")
//                                .addRequirements(Outtake.INSTANCE)
//                                .setExecute(() -> {})
//                )


    @Override
    public void loop() {
        telemetry.addData("VSlide Position", VSlide.INSTANCE.encoder.getCurrentPosition());
        telemetry.addData("spec mode:", Outtake.isSpecMode());

        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
        telemetry.update();
    }
}
