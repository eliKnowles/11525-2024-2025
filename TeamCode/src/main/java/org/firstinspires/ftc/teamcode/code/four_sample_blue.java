package org.firstinspires.ftc.teamcode.code;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hermeshelper.datatypes.TransferState;
import org.firstinspires.ftc.teamcode.hermeshelper.util.Sequence;
 import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;
import org.firstinspires.ftc.teamcode.rr.PinpointDrive;

@Config
@Autonomous(name = "BLUE_SAMPLE_4", group = "Autonomous")
public class four_sample_blue extends LinearOpMode {

    public static double vSlideTarget = 0;

    public class outtake {
        private ServoV2 intakePivotServoOne;
        private ServoV2 intakePivotServoTwo;
        private ServoV2 intakeClawServo;
        private ServoV2 outtakeClawServo;
        private ServoV2 outtakePivotServo;
        private ServoV2 intakeWristServo;
        private ServoV2 intakeWristServoTwo;

        private DcMotorV2 hSlideMotor;
        private DcMotorV2 vSlideMotorOne;
        private DcMotorV2 vSlideMotorTwo;

        private Sequence sequence;

        private TransferState currentTransferState = TransferState.H_IDLE;

        public outtake(HardwareMap hardwareMap) {
            intakePivotServoOne = new ServoV2("intake_pivot_one", hardwareMap);
            intakePivotServoTwo = new ServoV2("intake_pivot_two", hardwareMap);

            intakeClawServo = new ServoV2("intake_claw", hardwareMap);
            outtakeClawServo = new ServoV2("outtake_claw", hardwareMap);

            outtakePivotServo = new ServoV2("outtake_pivot_one", hardwareMap);
            intakeWristServo = new ServoV2("intake_wrist", hardwareMap);
            intakeWristServoTwo = new ServoV2("intake_wrist_two", hardwareMap);

            hSlideMotor = new DcMotorV2("h_slide", hardwareMap);
            hSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            vSlideMotorOne = new DcMotorV2("v_slide_one", hardwareMap);
            vSlideMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
            vSlideMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vSlideMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            vSlideMotorTwo = new DcMotorV2("v_slide_two", hardwareMap);
            vSlideMotorTwo.setDirection(DcMotorSimple.Direction.FORWARD);
            vSlideMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vSlideMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            intakeWristServo.setDirection(Servo.Direction.REVERSE);
            outtakePivotServo.setDirection(Servo.Direction.REVERSE);

            sequence = new Sequence();

            sequence.create("intakeGrab")

                    .add(intakePivotServoOne, .02, 0)
                    .add(intakeClawServo, .92f, 300)
                    .add(intakePivotServoOne, .2f, 300)
                    .build();

            sequence.create("transfer")
                    .add(intakePivotServoOne, .02, 300)
                    .add(intakeClawServo, .92f, 0)
                    .add(intakePivotServoOne, .55f, 500)
                    .add(intakeWristServo, .16f, 0)
                    .add(intakeWristServoTwo, .5f, 0)
                    .add(hSlideMotor, 0f, 300)
                    .add(outtakeClawServo, 0.86f, 500)
                    .add(intakeClawServo, 0.4f, 100)
                    .add(outtakePivotServo, .45f, 0)
                    .build();

            sequence.create("intakeNeutral")
                    .add(hSlideMotor, 450f, 0)
                    .add(intakeWristServoTwo, .5f, 0)
                    .add(outtakePivotServo, .89f, 0)
                    .add(outtakeClawServo, .98f, 0)
                    .add(intakePivotServoOne, .07f, 0)
                    .add(intakeWristServo, .73f, 0)
                    .add(intakeClawServo, .4f, 0)
                    .build();

            sequence.create("intakeNeutralNoExtendo")
                    .add(intakeWristServoTwo, .5f, 0)
                    .add(outtakePivotServo, .89f, 0)
                    .add(outtakeClawServo, .98, 0)
                    .add(intakePivotServoOne, .07f, 0)
                    .add(intakeWristServo, .73f, 0)
                    .add(intakeClawServo, .4f, 0)
                    .build();

            sequence.create("intakeNeutralNoExtendoFourthSpec")
                    .add(intakeWristServoTwo, 1f, 0)
                    .add(outtakePivotServo, .89f, 0)
                    .add(outtakeClawServo, .98, 0)
                    .add(intakePivotServoOne, .07f, 0)
                    .add(intakeWristServo, .73f, 0)
                    .add(intakeClawServo, .4f, 0)
                    .build();


            sequence.create("Idle")
                    .add(intakePivotServoOne, .5f, 0)
                    .add(intakeWristServoTwo, .5f, 0)
                    .add(intakeWristServo, .7f, 0)
                    .add(outtakePivotServo, .76f, 0)
                    .build();

            intakePivotServoTwo.setDirection(Servo.Direction.REVERSE);
        }


        class slidesNeutral implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                outtakePivotServo.setPosition(.89);
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }// open servo

                intakeWristServo.setPosition(.73);
                vSlideTarget = 0;
                return false;
            }
        }

        public Action SlidesNeutral() {
            return new slidesNeutral();

        }










        class SampleScoring implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeClawServo.setPosition(.8);


                vSlideTarget = 870;
                outtakePivotServo.setPosition(.39);
                return false;
            }
        }


        class servoPivotNeutral implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakePivotServo.setPosition(.89);
                return false;
            }
        }

        public Action ServoPivotNeutral() {
            return new servoPivotNeutral();

        }

        public Action sampleScoring() {
            return new SampleScoring();

        }

        class outtakeClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeClawServo.setPosition(.98);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }// open servo

                return false;
            }
        }

        public Action OuttakeClawOpen() {
            return new outtakeClawOpen();

        }

        public class vSlidePIDF implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                vSlideMotorOne.setPIDFCoefficients(.01, 0, .02, 0);
                vSlideMotorOne.setPositionWithPIDF(vSlideTarget, vSlideMotorOne.getCurrentPosition());
                vSlideMotorTwo.setPower(vSlideMotorOne.getPower());
//                vSlideMotorTwo.setPower(vSlideMotorOne.getPower());

                telemetry.addData("Ticks", "VSlideMotorTwo: " + vSlideMotorTwo.getCurrentPosition());
                telemetry.addData("Ticks", "vSlideMotorOne: " + vSlideMotorOne.getCurrentPosition());
                telemetry.update();
                return true;
            }
        }

        public Action SlidePIDF() {
            return new vSlidePIDF();

        }



        public class outtakeClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeClawServo.setPosition(.8);

                return false;
            }
        }

        public Action OuttakeClawClose() {
            return new outtakeClawClose();
        }


        public class IntakeGrabPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vSlideTarget = 0;
                sequence.run("intakeNeutralNoExtendo");
                return false;
            }
        }
        public Action intakeGrabPosition() {
            return new IntakeGrabPosition();
        }

        public class IntakeGrab implements Action {
            private boolean started = false; // Ensures sequence starts only once

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    vSlideTarget = 0;
                    sequence.run("intakeGrab"); // Start the sequence
                    started = true;
                }

                sequence.update(); // Progress the sequence

                // Return true while the sequence is running
                if (sequence.isRunning()) {
                    return true; // Keep the action looping
                }

                return false; // Done when the sequence finishes
            }
        }
        public Action intakeGrab() {
            return new IntakeGrab();
        }

        public Action IntakeGrabPositionFourth() {
            return new IntakeGrabPositionFourth();
        }

        public class IntakeGrabPositionFourth implements Action {
            private boolean started = false; // Ensures sequence starts only once

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    vSlideTarget = 0;
                    sequence.run("intakeNeutralNoExtendoFourthSpec"); // Start the sequence
                    started = true;
                }

                sequence.update(); // Progress the sequence

                // Return true while the sequence is running
                if (sequence.isRunning()) {
                    return true; // Keep the action looping
                }

                return false; // Done when the sequence finishes
            }
        }
        public Action intakeGrabPositionFourth() {
            return new IntakeGrabPositionFourth();
        }




        class transfer implements Action {
            private boolean started = false; // Ensures sequence starts only once

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    vSlideTarget = 0;
                    sequence.run("transfer"); // Start the sequence
                    started = true;
                }

                sequence.update(); // Progress the sequence

                // Return true while the sequence is running
                if (sequence.isRunning()) {
                    return true; // Keep the action looping
                }

                return false; // Done when the sequence finishes
            }
        }
        public Action Transfer() {
            return new transfer();

        }


        class transferFourth implements Action {
            private boolean started = false; // Ensures sequence starts only once

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vSlideTarget = 0;
                sequence.run("transfer"); // Start the sequence
                started = true;
                sequence.update(); // Progress the sequence

                // Return true while the sequence is running
                return false; // Done when the sequence finishes
            }
        }
        public Action TransferFourth() {
            return new transferFourth();

        }

        public class Sleep implements Action {

            private int time;

            public Sleep(int timeMS) {
                time = timeMS;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                try {
                    Thread.sleep(time);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                return false;
            }
        }

        public Action Sleep(int timeMS) {
            return new Sleep(timeMS);
        }




        public class SequenceUpdater implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sequence.update();
                return true;
            }
        }

        public Action sequenceUpdater() {
            return new SequenceUpdater();
        }

        public class hSlideIdle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hSlideMotor.setTargetPosition(0);
                hSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hSlideMotor.setPower(.5);
                return false;
            }
        }

        public Action HSlideIdle() {
            return new hSlideIdle();
        }
    }


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(37, 65, Math.toRadians(270));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        outtake claw = new outtake(hardwareMap);


        // vision here that outputs position
//        int initialPosition = 1;

        TrajectoryActionBuilder score_first_sample_position = drive.actionBuilder(new Pose2d(37, 65, Math.toRadians(270)))
                .strafeTo(new Vector2d(37,50))
                .setReversed(true)
                .setTangent(Math.toRadians(45))
                .splineTo(new Vector2d(56.5, 56.5), Math.toRadians(0));
        TrajectoryActionBuilder grab_second_sample = drive.actionBuilder(new Pose2d(56.5, 56.5, Math.toRadians(315)))
                .waitSeconds(.3)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(49, 50), Math.toRadians(270))
                .strafeTo(new Vector2d(49,37 ));
        TrajectoryActionBuilder score_second_sample =  drive.actionBuilder(new Pose2d(49, 37, Math.toRadians(270)))
                .waitSeconds(.3)
                .strafeToLinearHeading(new Vector2d(56.5, 56.5), Math.toRadians(225));
        TrajectoryActionBuilder grab_third_sample = drive.actionBuilder(new Pose2d(56.5, 56.5, Math.toRadians(315)))
                .waitSeconds(.3)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(58, 50), Math.toRadians(270))
                .strafeTo(new Vector2d(60,37 ));
        TrajectoryActionBuilder score_third_sample =  drive.actionBuilder(new Pose2d(60, 37, Math.toRadians(270)))
                .waitSeconds(.3)
                .strafeToLinearHeading(new Vector2d(56.5, 56.5), Math.toRadians(225));
        TrajectoryActionBuilder grab_fourth_sample = drive.actionBuilder(new Pose2d(56.5, 56.5,Math.toRadians(225)))
                .waitSeconds(.3)
                .strafeToLinearHeading(new Vector2d(45, 26.5), Math.toRadians(0))
                .strafeTo(new Vector2d(59,26.75 ));
        TrajectoryActionBuilder score_fourth_sample_drive_away = drive.actionBuilder(new Pose2d(59, 26.75,Math.toRadians(0)))
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(44, 26.5), Math.toRadians(0));

        TrajectoryActionBuilder score_fourth_sample = drive.actionBuilder(new Pose2d(44, 26.5,Math.toRadians(-135)))
                .waitSeconds(.3)
                .strafeToLinearHeading(new Vector2d(56.5, 56.5), Math.toRadians(225));
        TrajectoryActionBuilder score_fourth_sample_drive_away2 = drive.actionBuilder(new Pose2d(56.5, 56.5,Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(50, 50), Math.toRadians(225));




        // actions that need to happen on init; for instance, a claw tightening.


//        }
        waitForStart();
//        int startPosition = initialPosition;
//        telemetry.addData("Starting Position", startPosition);
//        telemetry.update();
//        waitForStart(); mnm
//
//        if (isStopRequested()) return;
//
//        Action trajectoryActionChosen;
//        if (startPosition == 1) {
//            trajectoryActionChosen = tab1.build();
//        } else {
//            trajectoryActionChosen = tab2.build();
//        }
        Actions.runBlocking(
                new ParallelAction(
                        claw.SlidePIDF(),
                        claw.HSlideIdle(),
                        new SequentialAction(
                                claw.sampleScoring(),
                                score_first_sample_position.build(),
                                claw.OuttakeClawOpen(),
                                claw.ServoPivotNeutral(),
                                claw.Sleep(200),

                                claw.intakeGrabPosition(),
                                grab_second_sample.build(),
                                claw.Transfer(),
                                claw.Sleep(1000),
                                claw.sampleScoring(),
                                score_second_sample.build(),
                                claw.OuttakeClawOpen(),
                                claw.ServoPivotNeutral(),
                                claw.Sleep(200),

                                claw.intakeGrabPosition(),
                                grab_third_sample.build(),
                                claw.Transfer(),
                                claw.Sleep(800),
                                claw.sampleScoring(),
                                score_third_sample.build(),
                                claw.OuttakeClawOpen(),
                                claw.ServoPivotNeutral(),
                                claw.Sleep(200),
                                claw.intakeGrabPosition(),
                                grab_fourth_sample.build(),
                                claw.IntakeGrabPositionFourth(),
                                claw.intakeGrab(),
                                score_fourth_sample_drive_away.build(),
                                claw.Transfer(),
                                claw.sampleScoring(),
                                score_fourth_sample.build(),
                                claw.Sleep(300),
                                claw.OuttakeClawOpen(),
                                score_fourth_sample_drive_away2.build(),
                                claw.ServoPivotNeutral(),
                                claw.SlidesNeutral()


                        ),
                        claw.sequenceUpdater()
                        // Run the PID loop for the slide
                )
        );

        // TODO: add placing the thing


    }
}