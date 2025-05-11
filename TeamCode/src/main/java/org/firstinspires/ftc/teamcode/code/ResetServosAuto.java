package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hermeshelper.util.VSlide;

import dev.frozenmilk.mercurial.Mercurial;

@TeleOp
@VSlide.Attach
@Mercurial.Attach
public class ResetServosAuto extends OpMode {
    private final int EXTEND_POS = 800;
    private final int RETRACT_POS = 0;

    @Override
    public void init() {
        // Default VSlide.update() already set in the subsystem via setDefaultCommand()
        // We just need to trigger position change
        Mercurial.gamepad1().a().onTrue(VSlide.goTo(EXTEND_POS));
        Mercurial.gamepad1().b().onTrue(VSlide.goTo(RETRACT_POS));
    }

    @Override
    public void loop() {
        telemetry.addData("VSlide Position", VSlide.INSTANCE.encoder.getCurrentPosition());
        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
        telemetry.update();
    }
}
