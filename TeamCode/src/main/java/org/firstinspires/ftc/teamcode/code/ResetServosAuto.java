package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hermeshelper.util.VSlide;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.Outtake;



import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.util.IfElse;


@TeleOp
@VSlide.Attach
@Outtake.Attach
@Mercurial.Attach

public class ResetServosAuto extends OpMode {
    private final int EXTEND_POS = 800;
    private final int RETRACT_POS = 0;

    @Override
    public void init() {
        Mercurial.gamepad1().b().onTrue(VSlide.goTo(800));
        Mercurial.gamepad1().a().onTrue(VSlide.goTo(0));

        Mercurial.gamepad1().y().onTrue(
                new IfElse(
                        () -> Outtake.INSTANCE.getState() == Outtake.OuttakeState.RETRACTED,
                        Outtake.extendArmSample(),
                        new Lambda("noop")
                                .addRequirements(Outtake.INSTANCE)
                                .setExecute(() -> {})
                )
        );
        Mercurial.gamepad1().x().onTrue(
                new IfElse(
                        () -> Outtake.INSTANCE.getState() == Outtake.OuttakeState.EXTENDED || Outtake.INSTANCE.getState()== Outtake.OuttakeState.RETRACTED_SPEC,
                        Outtake.retractArmSample(),
                        new Lambda("noop")
                                .addRequirements(Outtake.INSTANCE)
                                .setExecute(() -> {})
                )
        );
        Mercurial.gamepad2().x().onTrue(
                new IfElse(
                        () -> Outtake.INSTANCE.getState() == Outtake.OuttakeState.RETRACTED || Outtake.INSTANCE.getState()== Outtake.OuttakeState.RETRACTED_SPEC,
                        Outtake.grabPosSpecimen(),
                        new Lambda("noop")
                                .addRequirements(Outtake.INSTANCE)
                                .setExecute(() -> {})
                )
        );
        Mercurial.gamepad2().y().onTrue(
                new IfElse(
                        () -> Outtake.INSTANCE.getState() == Outtake.OuttakeState.SPECIMEN_WALL,
                        Outtake.retractSpecimen(),
                        new Lambda("noop")
                                .addRequirements(Outtake.INSTANCE)
                                .setExecute(() -> {})
                )
        );
    }



    @Override
    public void loop() {
        telemetry.addData("VSlide Position", VSlide.INSTANCE.encoder.getCurrentPosition());
        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
        telemetry.update();
    }
}
