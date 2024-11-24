package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

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


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hermeshelper.datatypes.TransferState;
import org.firstinspires.ftc.teamcode.hermeshelper.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.hermeshelper.util.Sequence;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;
import org.firstinspires.ftc.teamcode.RobotGoBrrr;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@Autonomous(name = "BLUE_SPECIMEN 4", group = "Autonomous")
public class BigBoiAuto extends LinearOpMode {

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
            hSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            hSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            vSlideMotorOne = new DcMotorV2("v_slide_one", hardwareMap);
            vSlideMotorOne.setDirection(DcMotorSimple.Direction.REVERSE);
            vSlideMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vSlideMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            vSlideMotorTwo = new DcMotorV2("v_slide_two", hardwareMap);
            vSlideMotorTwo.setDirection(DcMotorSimple.Direction.FORWARD);
            vSlideMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vSlideMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            hSlideMotor.setDirection(FORWARD);

            sequence = new Sequence();


            sequence.create("transfer")
                    .add(intakePivotServoOne, .59f, 0)
                    .add(intakeWristServo, 0f, 0)
                    .add(intakeWristServoTwo, .5f, 0)
                    .add(hSlideMotor, 0f, 300)
                    .add(outtakeClawServo, 0.68f, 500)
                    .add(intakeClawServo, 0.4f, 100)
                    .add(outtakePivotServo, .25f, 0)
                    .build();

            sequence.create("intakeNeutral")
                    .add(hSlideMotor, 450f, 0)
                    .add(intakeWristServoTwo, .5f, 0)
                    .add(outtakePivotServo, .76f, 0)
                    .add(outtakeClawServo, .4f, 0 )
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
                vSlideTarget = 200;

                return false;
            }
        }

        public Action specimenScoring(){
            return new SpecimenScoring();

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



        public class IntakeNeutral implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakePivotServo.setPosition(.25);
                outtakeClawServo.setPosition(.68);
                return false;
            }
        }

        public Action intakeNeutral() {
            return new IntakeNeutral();
        }

        public class IntakeGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sequence.run("intakeGrab");
                return false;
            }
        }

        public Action intakeGrab() {
            return new IntakeGrab();
        }

        public class Idle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sequence.run("intakeGrab");
                return false;
            }
        }

        public Action idle() {
            return new Idle();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-23.5, 63, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        outtake claw = new outtake(hardwareMap);

        // vision here that outputs position
//        int initialPosition = 1;


        waitForStart();
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)

                // place 1st specimen
                .setReversed(true)
                .splineTo(new Vector2d(-5, 36), Math.toRadians(270))
                // once at -5, 36, open claw

                //pivotservo set to specimen
                //slides set to specimen
                .waitSeconds(.2)
                //action

                // drive to push samples
                .setReversed(false)
                .splineTo(new Vector2d(-34, 28), Math.toRadians(270)) //waypoint to first sample

                .splineTo(new Vector2d(-45, 6), Math.toRadians(270)) // align with first sample


                .setReversed(true)
                .splineTo(new Vector2d(-46,53), Math.toRadians(90))// push first sample
                .setReversed(false)// push first sample

                .splineTo(new Vector2d(-47, 4), Math.toRadians(270))  // return
                .setReversed(true)
                .splineTo(new Vector2d(-56, 22), Math.toRadians(90)) // waypoint
                .splineTo(new Vector2d( -57, 53), Math.toRadians(90))  //push 2nd
                .setReversed(false)


                //cycle 2nd specimen
                .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
                //grab 2nd
                .waitSeconds(.2)
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(-37, 57), Math.toRadians(90)); //intake


        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)


                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
                .waitSeconds(.2)

                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
                .setReversed(false)
                .strafeTo(new Vector2d(-37, 57))
                .waitSeconds(.2) // grab


                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
                .waitSeconds(.2)


                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
                .setReversed(false)
                .strafeTo(new Vector2d(-37, 57))
                .waitSeconds(.3) // grab

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
                .waitSeconds(.3)

                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking

                .strafeTo(new Vector2d(-37, 57))
                .waitSeconds(.1) // grab

                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270));
//                        .waitSeconds(.1)
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-5, 37, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(-42, 54, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-5, 37, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-42, 54, Math.toRadians(90)), Math.toRadians(0));

        //Actions.runBlocking(claw.intakeNeutral());
        Actions.runBlocking(claw.specimenScoring());
       // Actions.runBlocking(claw.intakeGrab());
       // Actions.runBlocking(claw.transfer());




        // actions that need to happen on init; for instance, a claw tightening.
//        Actions.runBlocking(claw.idle());


//        while (!isStopRequested() && !opModeIsActive()) {
//            int position = initialPosition;
//            telemetry.addData("Position during Init", position);
//            telemetry.update();
//        }
        waitForStart();
//        int startPosition = initialPosition;
//        telemetry.addData("Starting Position", startPosition);
//        telemetry.update();
//        waitForStart();
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
                        new SequentialAction(
                                claw.specimenScoring(),
                                tab1.build() // Run the trajectory
                                 ),
                        claw.SlidePIDF()// Run the PID loop for the slide
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