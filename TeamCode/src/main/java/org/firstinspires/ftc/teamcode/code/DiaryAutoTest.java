package org.firstinspires.ftc.teamcode.code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

//@Mercurial.Attach
@Autonomous
public class DiaryAutoTest extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
    }

    @Override
    public void start() {
        new Sequential(
                new Parallel( // similar to parallel action but with dairy

                ),

                new Parallel(
                        new Sequential( // stack them inside each other

                        )
                ),

                new Parallel(
                        new Sequential(
                                new Wait(0.2) // wait like this instead of thread.sleep()
                        )
                )
        ).schedule(); // run it
    }
}