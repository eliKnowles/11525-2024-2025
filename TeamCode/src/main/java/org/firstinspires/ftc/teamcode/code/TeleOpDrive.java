package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.code.subsystem.Drive;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;

@Drive.Attach
public class TeleOpDrive extends OpMode {

    @Override
    public void init() {
        BoundGamepad gpOne = Mercurial.gamepad1();
        BoundGamepad gpTwo = Mercurial.gamepad2();
    }

    @Override
    public void loop() {

    }
}
