package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VSlide;
import org.firstinspires.ftc.teamcode.code.util.Drive;
import org.firstinspires.ftc.teamcode.code.util.Outtake;



import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.IfElse;
import dev.frozenmilk.mercurial.commands.util.Wait;

@TeleOp
@VSlide.Attach
@Outtake.Attach
@Mercurial.Attach
@Drive.Attach
public class MainTeleOp extends OpMode {

    @Override
    public void init() {

            Mercurial.gamepad1().y().onTrue(
                    Outtake.extend()
            );

            Mercurial.gamepad1().x().onTrue(
                    Outtake.retract()
            );

            Mercurial.gamepad1().share().onTrue(
                    Outtake.extend()
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
        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
        telemetry.update();
    }
}
