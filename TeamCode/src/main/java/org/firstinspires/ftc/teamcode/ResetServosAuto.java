package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Autonomous
@Mercurial.Attach
@Servos.Attach
public class ResetServosAuto extends OpMode {

    @Override
    public void init() {
        //nothing
    }

    @Override
    public void loop() {
        //nothing
    }

    @Override
    public void start() {
        new Sequential(
                new Parallel(
                    Servos.setZero()
                )
        ).schedule();
    }
}
