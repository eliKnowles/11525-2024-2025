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
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
@Autonomous(name = "RED_SAMPLE_4", group = "Autonomous")
public class four_sample_red extends LinearOpMode {

    public static double vSlideTarget = 0;
    public static double hSlideTarget = 0;




    public class outtake {

        Limelight3A limelight;
        HardwareMap hardwareMap;


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

            Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

            intakeClawServo = new ServoV2("intake_claw", hardwareMap);
            outtakeClawServo = new ServoV2("outtake_claw", hardwareMap);

            outtakePivotServo = new ServoV2("outtake_pivot_one", hardwareMap);
            intakeWristServo = new ServoV2("intake_wrist", hardwareMap);
            intakeWristServoTwo = new ServoV2("intake_wrist_two", hardwareMap);

            hSlideMotor = new DcMotorV2("h_slide", hardwareMap);
            hSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
                    .add(intakeWristServo, .73,0)
                    .add(intakePivotServoOne, .02, 0)
                    .add(intakeClawServo, .92f, 300)
                    .add(intakePivotServoOne, .2f, 300)
                    .build();

            sequence.create("transfer")
                    .add(intakeWristServo, .7,0)

                    .add(intakePivotServoOne, .02, 200)

                    .add(intakeClawServo, .92f, 300)
                    .add(intakePivotServoOne, .55f, 300)
                    .add(intakeWristServo, .16f, 0)
                    .add(intakeWristServoTwo, .5f, 0)
                    .add(outtakeClawServo, 0.86f, 500)
                    .add(intakeClawServo, 0.4f, 100)
                    .add(outtakePivotServo, .45f, 0)
                    .build();

            sequence.create("intakeNeutral")
                    .add(hSlideMotor, 450f, 0)
                    .add(intakeWristServoTwo, .5f, 0)
                    .add(outtakePivotServo, .85f, 0)
                    .add(outtakeClawServo, .98f, 0)
                    .add(intakePivotServoOne, .07f, 0)
                    .add(intakeWristServo, .73f, 0)
                    .add(intakeClawServo, .4f, 0)
                    .build();

            sequence.create("intakeNeutralNoExtendo")
                    .add(intakeWristServoTwo, .5f, 0)
                    .add(outtakePivotServo, .85f, 0)
                    .add(outtakeClawServo, .98, 0)
                    .add(intakePivotServoOne, .1f, 0)
                    .add(intakeWristServo, .77, 0)
                    .add(intakeClawServo, .4f, 0)
                    .build();

            sequence.create("intakeNeutralNoExtendoFourthSpec")
                    .add(intakeWristServoTwo, 0f, 0)
                    .add(outtakePivotServo, .85f, 0)
                    .add(outtakeClawServo, .98, 0)
                    .add(intakePivotServoOne, .07f, 0)
                    .add(intakeWristServo, .77f, 0)
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
                intakeWristServo.setPosition(.77);
                vSlideTarget = 0;
                return false;
            }
        }

        public Action SlidesNeutral() {
            return new slidesNeutral();

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
        class limelightHover implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeClawServo.setPosition(.4);
                intakePivotServoOne.setPosition(.15);
                intakeWristServo.setPosition(.75);
                return false;
            }
        }

        public Action LimelightHover() {
            return new limelightHover();

        }







        class SampleScoring implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                outtakeClawServo.setPosition(.8);
                outtakePivotServo.setPosition(.34);
                vSlideTarget = 880;
                return false;
            }
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

                vSlideMotorOne.setPIDFCoefficients(.07, 0, .02, 0);
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
        public class hSlidePIDF implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                hSlideMotor.setPIDFCoefficients(.04, 0, .01, 0);
                hSlideMotor.setPositionWithPIDF(hSlideTarget, hSlideMotor.getCurrentPosition());

                return true;
            }
        }

        public Action hSlidePIDF() {
            return new hSlidePIDF();

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


        public class hSlidePID implements Action {

            private double hSlideTargetpid = 0;

            public hSlidePID(double HSlide) {
                hSlideTargetpid = HSlide;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                hSlideTarget =  hSlideTargetpid;
                return true;
            }
        }

        public Action HSlidePos(double HSlide) {
            return new hSlidePID(HSlide);
        }



        public class IntakeGrabPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vSlideTarget = 0;
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
                hSlideTarget = 0;
                return false;
            }
        }

        public Action HSlideIdle() {
            return new hSlideIdle();
        }
    }




    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(-37, -65, Math.toRadians(90));
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        outtake claw = new outtake(hardwareMap);

        Align align = new Align(limelight, drive);

        Action run_auton = drive.actionBuilder(initialPose)
                .afterTime(.01,new SequentialAction(claw.sampleScoring()))
                .strafeTo(new Vector2d(-37,-50))
                .setReversed(true)
                .setTangent(Math.toRadians(225))
                .splineTo(new Vector2d(-56.5, -56.5), Math.toRadians(180))
                .afterDisp(.01, claw.OuttakeClawOpen())
                .waitSeconds(.5)
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-49, -50), Math.toRadians(90))

                .afterTime(.01,new SequentialAction(claw.SlidesNeutral(),claw.intakeGrabPosition(), claw.LimelightHover()))// slides down, limelight goes to searching positionm
                .waitSeconds(.7)
                .strafeTo(new Vector2d(-49,-41))
                .afterTime(.01,new SequentialAction(align.CenterOverTarget(),claw.Transfer()))// limelight alignment, transfer
                .waitSeconds(3)
                .setReversed(true)
                .afterTime(.01,new SequentialAction(claw.sampleScoring()))
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-56.5, -56.5), Math.toRadians(45))
                .afterDisp(.01, claw.OuttakeClawOpen())
                .waitSeconds(.5)



                .strafeToLinearHeading(new Vector2d(-60, -50), Math.toRadians(90))

                .afterTime(.01,new SequentialAction(claw.SlidesNeutral(),claw.intakeGrabPosition(), claw.LimelightHover()))// slides down, limelight goes to searching positionm
                .waitSeconds(.7)
                .strafeToLinearHeading(new Vector2d(-60, -41), Math.toRadians(90))
                .afterTime(.01,new SequentialAction(align.CenterOverTarget(),claw.Transfer()))// limelight alignment, transfer
                .waitSeconds(3)

                .afterTime(.01,new SequentialAction(claw.sampleScoring()))
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-56.5, -56.5), Math.toRadians(45))
                .afterTime(.01, claw.OuttakeClawOpen())
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(90))
                .afterTime(.01, new SequentialAction(claw.SlidesNeutral(), claw.intakeGrabPositionFourth()))
                 .waitSeconds(.2)
                 .strafeToLinearHeading(new Vector2d(-59, -26.75), Math.toRadians(180))
                  .afterTime(.01, claw.intakeGrab())
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-44, -26.5), Math.toRadians(180))
                .afterTime(.01, new SequentialAction( claw.Transfer(), claw.sampleScoring()))
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-53, -53), Math.toRadians(45))
                .strafeTo(new Vector2d(-56.5,-56.5 ))
                .afterTime(.01, claw.OuttakeClawOpen())
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(45))
                .afterTime(.01, claw.SlidesNeutral())



              /* .waitSeconds(.2)
               .strafeToLinearHeading(new Vector2d(-44, -26.5), Math.toRadians(180))
               .afterTime(.01, claw.Transfer())*/



                .build();


                /*

                .strafeToLinearHeading(new Vector2d(-45, -26.5), Math.toRadians(180))
                .strafeTo(new Vector2d(-59,-26.75 ));
                drive.updatePoseEstimate();

        TrajectoryActionBuilder score_fourth_sample_drive_away =  drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-44, -26.5), Math.toRadians(180));
                drive.updatePoseEstimate();


        TrajectoryActionBuilder score_fourth_sample =  drive.actionBuilder(drive.pose)
                .waitSeconds(.3)
                .strafeToLinearHeading(new Vector2d(-53, -53), Math.toRadians(45))
                .strafeTo(new Vector2d(-56.5,-56.5 ));


        TrajectoryActionBuilder score_fourth_sample_drive_away2 =  drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(45));
                 drive.updatePoseEstimate(); */// actions that need to happen on init; for instance, a claw tightening.
//        }


                        /*
                        TrajectoryActionBuilder score_second_sample =   drive.actionBuilder(drive.pose)
                                .waitSeconds(.3)
                                .strafeToLinearHeading(new Vector2d(-57, -56.5), Math.toRadians(45));
                                drive.updatePoseEstimate();

                        TrajectoryActionBuilder grab_third_sample =  drive.actionBuilder(drive.pose)
                                .setReversed(false)
                                .strafeToLinearHeading(new Vector2d(-58, -50), Math.toRadians(90))
                                .strafeTo(new Vector2d(-60,-37 ));
                                drive.updatePoseEstimate();

                        TrajectoryActionBuilder score_third_sample =   drive.actionBuilder(drive.pose)
                                .waitSeconds(.3)
                                .strafeToLinearHeading(new Vector2d(-57, -56.5), Math.toRadians(45));
                                drive.updatePoseEstimate();

                        TrajectoryActionBuilder grab_fourth_sample =  drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(-45, -26.5), Math.toRadians(180))
                                .strafeTo(new Vector2d(-59,-26.75 ));
                                drive.updatePoseEstimate();

                        TrajectoryActionBuilder score_fourth_sample_drive_away =  drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(-44, -26.5), Math.toRadians(180));
                                drive.updatePoseEstimate();


                        TrajectoryActionBuilder score_fourth_sample =  drive.actionBuilder(drive.pose)
                                .waitSeconds(.3)
                                .strafeToLinearHeading(new Vector2d(-53, -53), Math.toRadians(45))
                                .strafeTo(new Vector2d(-56.5,-56.5 ));


                        TrajectoryActionBuilder score_fourth_sample_drive_away2 =  drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(45));
                                 drive.updatePoseEstimate(); */




        // actions that need to happen on init; for instance, a claw tightening.

//        }

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
        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        claw.SlidePIDF(),
                        claw.hSlidePIDF(),
                        claw.sequenceUpdater(),
                        new SequentialAction(run_auton)
                )

        );


        // Run the PID loop for the slide

        // TODO: add placing the thing


    }
}