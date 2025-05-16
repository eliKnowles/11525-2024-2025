package org.firstinspires.ftc.teamcode.code;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VSlide;
import org.firstinspires.ftc.teamcode.code.util.Drive;
import org.firstinspires.ftc.teamcode.code.util.Intake;
import org.firstinspires.ftc.teamcode.code.util.Outtake;


import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
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

        Mercurial.gamepad1().y().onTrue(
                new Lambda("Dynamic Y Command")
                        .setExecute(() -> {
                            if (Outtake.isSpecMode()) {
                                new Parallel(
                                        Outtake.scoreSpecimen(),
                                        VSlide.goTo(490)
                                ).schedule();
                            } else {
                                new Parallel(
                                        Outtake.extendArmSample(),
                                        VSlide.goTo(630)
                                ).schedule();
                            }
                        })
                        .setFinish(() -> true)
        );

        // RETRACT button
        Mercurial.gamepad1().x().onTrue(
                new Lambda("Dynamic X Command")
                        .setExecute(() -> {
                            if (Outtake.isSpecMode()) {
                                new Parallel(
                                        Outtake.grabSpecimen(),
                                        new Wait(.4),
                                        VSlide.goTo(0, 0.2)
                                ).schedule();
                            } else {
                                new Parallel(
                                        Outtake.retractArmSample(),
                                        VSlide.goTo(0, 0.2)
                                ).schedule();
                            }
                        })
                        .setFinish(() -> true)
        );


        // TOGGLE button
        Mercurial.gamepad1().share().onTrue(
                Outtake.toggleMode()
        );
        Mercurial.gamepad1().circle().onTrue(
                Intake.intakeGrab()
        );
    }






//                new IfElse(
//                        () -> Outtake.INSTANCE.getState() == Outtake.OuttakeState.RETRACTED,
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
//                        () -> Outtake.INSTANCE.getState() == Outtake.OuttakeState.RETRACTED || Outtake.INSTANCE.getState()== Outtake.OuttakeState.RETRACTED_SPEC,
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
