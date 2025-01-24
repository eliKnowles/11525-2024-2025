package org.firstinspires.ftc.teamcode.code;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hermeshelper.datatypes.TransferState;
import org.firstinspires.ftc.teamcode.hermeshelper.datatypes.vExtensionMode;
import org.firstinspires.ftc.teamcode.hermeshelper.util.Sequence;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;
import org.firstinspires.ftc.teamcode.rr.PinpointDrive;

@Config
@Autonomous(name = "RED_SPECIMEN 4", group = "DONT_RUN")
public class four_spec_red extends LinearOpMode {

    public static double vSlideTarget = 0 ;

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

            outtakePivotServo.setDirection(Servo.Direction.REVERSE);

            sequence = new Sequence();

            sequence.create("transfer")
                    .add(intakePivotServoOne, .59f, 0)
                    .add(intakeWristServo, 0f, 0)
                    .add(intakeWristServoTwo, .5f, 0)
                    .add(outtakeClawServo, 0.8f, 500)
                    .add(intakeClawServo, 0.4f, 100)
                    .add(outtakePivotServo, .55f, 0)
                    .build();


            sequence.create("intakeNeutral")
                    .add(intakeWristServoTwo, .5f, 0)
                    .add(outtakePivotServo, .85f, 0)
                    .add(outtakeClawServo, .98f, 0 )
                    .add(intakePivotServoOne, .07f, 0)
                    .add(intakeWristServo, .75f, 0)
                    .add(intakeClawServo, .4f, 0)
                    .build();

            sequence.create("intakeGrab")
                    .add(outtakeClawServo, .9f, 0)
                    .add(outtakePivotServo, .2f, 0)
                    .build();

            currentTransferState = TransferState.H_IDLE;

            sequence.create("Idle")
                    .add(outtakeClawServo, .4f, 0)
                    .add(outtakePivotServo, .4f, 0)
                    .build();
        }

        public class Transfer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sequence.run("transfer");
                return false;
            }
        }

        public Action transfer() {
            return new Transfer();
        }

        class SpecimenScoring implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vSlideTarget = 275;
                outtakeClawServo.setPosition(.4);
                outtakePivotServo.setPosition(.27);
                return false;
            }
        }

        public Action specimenScoring(){
            return new SpecimenScoring();

        }

        class outtakeClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeClawServo.setPosition(1); // open servo
                vSlideTarget = 190;


                return false;
            }
        }

        public Action OuttakeClawOpen(){
            return new outtakeClawOpen();

        }

        class outtakePullDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakePivotServo.setPosition(.24);
                vSlideTarget = 210;
                return false;
            }
        }

        public Action OuttakePullDown(){
            return new outtakePullDown();

        }

        public class vSlidePIDF implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                vSlideMotorOne.setPIDFCoefficients(.01, 0, .02,0);
                vSlideMotorOne.setPositionWithPIDF(vSlideTarget, vSlideMotorOne.getCurrentPosition());
                vSlideMotorTwo.setPower(vSlideMotorOne.getPower());
//                vSlideMotorTwo.setPower(vSlideMotorOne.getPower());

                telemetry.addData("Ticks", "VSlideMotorTwo: " + vSlideMotorTwo.getCurrentPosition());
                telemetry.addData("Ticks", "vSlideMotorOne: " + vSlideMotorOne.getCurrentPosition());
                telemetry.update();
                return true;
            }
        }

        public Action SlidePIDF(){
            return new vSlidePIDF();

        }



        public class OuttakeNeutral implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vSlideTarget = 0;
                outtakePivotServo.setPosition(.5);
                return false;
            }
        }

        public Action outtakeNeutral() {
            return new OuttakeNeutral();
        }

        public class outtakeIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vSlideTarget = 0;
                outtakePivotServo.setPosition(.11);
                outtakeClawServo.setPosition(.98);
                return false;
            }
        }


        public Action OuttakeIntake() {
            return new outtakeIntake();
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

        public class outtakeClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeClawServo.setPosition(.8);
                try {
                    Thread.sleep(150);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }// open servo

                return false;
            }
        }

        public Action OuttakeClawClose() {
            return new outtakeClawClose();
        }

        public class InitPosOuttake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakePivotServo.setPosition(.55);


                return false;
            }
        }

        public Action initPosOuttake() {
            return new InitPosOuttake();
        }




        public class IntakeGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sequence.run("intakeGrab");
                return false;
            }
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

        public Action sleep(int timeMS) {
            return new Sleep(timeMS);
        }

        public Action intakeGrab() {
            return new IntakeGrab();
        }

        public class hSlideIdle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hSlideMotor.setTargetPosition(0);
                hSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hSlideMotor.setPower(.5);
                return true;
            }
        }

        public Action HSlideIdle() {
            return new hSlideIdle();
        }
    }


    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(5, -65, Math.toRadians(270));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        outtake claw = new outtake(hardwareMap);
        claw.OuttakeClawClose();
        claw.initPosOuttake();



        Actions.runBlocking(
                new ParallelAction(
                        claw.OuttakeClawClose(),
                        claw.initPosOuttake()

                )
        );




        // vision here that outputs position
//        int initialPosition = 1;
        Action run_auton = drive.actionBuilder(initialPose)
                .stopAndAdd(claw.specimenScoring())
                .waitSeconds(.2)
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(7, -31), Math.toRadians(90))
                .afterTime(.01, claw.OuttakePullDown())
                .waitSeconds(.3)
                .stopAndAdd(claw.OuttakeClawOpen())
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(35,-37 ), Math.toRadians(90)) //waypoint to first sample
                .splineToConstantHeading(new Vector2d(35, -20), Math.toRadians(90)) // align with first sampleample

                .afterTime(.5, claw.outtakeNeutral())
                .splineToConstantHeading(new Vector2d(46, -7), Math.toRadians(90), new TranslationalVelConstraint(30)) // align with first sampleample
                .splineToConstantHeading(new Vector2d(46, -40), Math.toRadians(270), new TranslationalVelConstraint(30)) //push first
                .splineToConstantHeading(new Vector2d(46, -50), Math.toRadians(270), new TranslationalVelConstraint(40)) //push first

                .splineToConstantHeading(new Vector2d(50, -15), Math.toRadians(270),new TranslationalVelConstraint(25)) // return
                .splineToConstantHeading(new Vector2d(62, -20), Math.toRadians(270),new TranslationalVelConstraint(30)) // waypoint to push second // waypoint to push second
                .afterTime(.5, claw.OuttakeIntake())
                .splineToConstantHeading(new Vector2d(62, -40.5), Math.toRadians(270),new TranslationalVelConstraint(30)) // push second and grab
                .splineToConstantHeading(new Vector2d(62, -60.5), Math.toRadians(270),new TranslationalVelConstraint(30)) // push second and grab


                .stopAndAdd(claw.OuttakeClawClose())
                .waitSeconds(.1)
                .stopAndAdd(claw.specimenScoring())
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(0, -45, Math.toRadians(-90)), Math.toRadians(90))
                .splineTo(new Vector2d(0, -31), Math.toRadians(90))

                .afterTime(.01, claw.OuttakePullDown())
                .waitSeconds(.3)
                .stopAndAdd(claw.OuttakeClawOpen())
                .afterTime(1, claw.OuttakeIntake())
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(47, -50, Math.toRadians(90)), Math.toRadians(270))
                .splineTo(new Vector2d(47, -60.75), Math.toRadians(270))

                .stopAndAdd(claw.OuttakeClawClose())
                .waitSeconds(.1)
                .stopAndAdd(claw.specimenScoring())

                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-3, -45, Math.toRadians(-90)), Math.toRadians(90))
                .splineTo(new Vector2d(-3, -31), Math.toRadians(90))
                .afterTime(.01, claw.OuttakePullDown())

                .waitSeconds(.3)
                .stopAndAdd(claw.OuttakeClawOpen())

                .afterTime(1, claw.OuttakeIntake())
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(47, -50, Math.toRadians(90)), Math.toRadians(270))
                .splineTo(new Vector2d(47, -60.75), Math.toRadians(270))


                .stopAndAdd(claw.OuttakeClawClose())
                .waitSeconds(.1)
                .stopAndAdd(claw.specimenScoring())

                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(4, -45, Math.toRadians(-90)), Math.toRadians(90))
                .splineTo(new Vector2d(4, -31), Math.toRadians(90))
                .afterTime(.01, claw.OuttakePullDown())
                .waitSeconds(.3 )
                .stopAndAdd(claw.OuttakeClawOpen())
                .strafeTo(new Vector2d(2, -40))
                .stopAndAdd(claw.outtakeNeutral())




                .build();


        waitForStart();



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

        Actions.runBlocking(
                new ParallelAction(
                        claw.SlidePIDF(),
                        claw.sequenceUpdater(),
                        claw.HSlideIdle(),
                        new SequentialAction(run_auton)
                )
        );
//                        claw.intakeGrab(),
////                        claw.transfer(),
////
//                      claw.intakeNeutral(),
////                        claw.idle(),
//                        tab2.build(),
//                        tab3.build()
        // TODO: add placing the thing



    }
}