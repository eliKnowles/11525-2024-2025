package org.firstinspires.ftc.teamcode.code;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.InstantFunction;
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
            hSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                    .add(outtakeClawServo, 0.68f, 500)
                    .add(intakeClawServo, 0.4f, 100)
                    .add(outtakePivotServo, .55f, 0)
                    .build();

            sequence.create("intakeNeutral")
                    .add(intakeWristServoTwo, .5f, 0)
                    .add(outtakePivotServo, .85f, 0)
                    .add(outtakeClawServo, .75f, 0 )
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
                outtakeClawServo.setPosition(.6);
                vSlideTarget = 260;
                outtakePivotServo.setPosition(.38);



                return false;
            }
        }

        public Action specimenScoring(){
            return new SpecimenScoring();

        }

        class outtakeClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeClawServo.setPosition(.75); // open servo

                return false;
            }
        }

        public Action OuttakeClawOpen(){
            return new outtakeClawOpen();

        }

        public class vSlidePIDF implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                
                vSlideMotorOne.setPIDFCoefficients(.03, 0, .02,0);
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
                vSlideTarget = 150;
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
                outtakePivotServo.setPosition(.20);
                outtakeClawServo.setPosition(.75);
                return false;
            }
        }

        public Action OuttakeIntake() {
            return new outtakeIntake();
        }

        public class outtakeClawClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeClawServo.setPosition(.6);

                return false;
            }
        }

        public Action OuttakeClawClose() {
            return new outtakeClawClose();
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

        public class hSlideIdle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                    hSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                return false;
            }
        }

        public Action HSlideIdle() {
            return new hSlideIdle();
        }
    }


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-5, 65, Math.toRadians(90));

        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        outtake claw = new outtake(hardwareMap);



        // vision here that outputs position
//        int initialPosition = 1;



        waitForStart();
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)

                .setReversed(true)
                .splineTo(new Vector2d(8, 31.5), Math.toRadians(270)) ; //waypoint to first sample

        // once at -5, 36, open claw

                //pivotservo set to specimen
                //slides set to specimen
                //action
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-5, 31.5, Math.toRadians(90 )))

                .setReversed(false)
                .splineTo(new Vector2d(-35,37 ), Math.toRadians(270)) //waypoint to first sample

                .splineTo(new Vector2d(-50, 6), Math.toRadians(270)) // align with first sampleample


                .setReversed(true)
                .splineTo(new Vector2d(-46,53), Math.toRadians(90));// push first sample
                //.setReversed(false) ;// push first sample

        //        .splineTo(new Vector2d(-47, 4), Math.toRadians(270))  // return
       //         .setReversed(true)
       //         .splineTo(new Vector2d(-58, 22), Math.toRadians(90)) // waypoint
       //         .splineTo(new Vector2d( -58, 53), Math.toRadians(90)); //push 2nd  //push 2nd


                //cycle 2nd specimen

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-46, 53, Math.toRadians(270)))
                .waitSeconds(0.1)
                .strafeTo(new Vector2d(-46, 61))// position for intaking 2nd
                .waitSeconds(.3);




        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(-37, 61, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-12, 50, Math.toRadians(90)), Math.toRadians(270)) //intak
                .waitSeconds(.1)
                .strafeTo(new Vector2d(2, 30.7))
                .waitSeconds(.2); // place 2nd specimen

        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-2, 30.7, Math.toRadians(90)))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking 2nd

                .waitSeconds(.4);
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(-37,50, Math.toRadians(270)))
                .strafeTo(new Vector2d(-37, 61))
                .waitSeconds(.4);                //grab 3rd


        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(-37, 61, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-12, 50, Math.toRadians(90)), Math.toRadians(270)) //intak
                .waitSeconds(.1)
                .strafeTo(new Vector2d(5, 30.7));
        TrajectoryActionBuilder tabIntakePosition4th = drive.actionBuilder(new Pose2d(1, 30.7, Math.toRadians(90)))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
                //grab 2nd
                .waitSeconds(.4);
        TrajectoryActionBuilder tabIntake4th = drive.actionBuilder(new Pose2d(-37,50, Math.toRadians(270)))
                .strafeTo(new Vector2d(-37, 61))
                .waitSeconds(.4);
        TrajectoryActionBuilder tabPlace4th = drive.actionBuilder(new Pose2d(-37, 61, Math.toRadians(90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-12, 50, Math.toRadians(90)), Math.toRadians(270)) //intak
                .waitSeconds(.1)
                .strafeTo(new Vector2d(-12, 30.7))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(-12, 30.5)); // place 4th specimen // 4th cycle









        Actions.runBlocking(claw.specimenScoring());




        // actions that need to happen on init; for instance, a claw tightening.


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
                        claw.SlidePIDF(),
                        claw.HSlideIdle(),
                        new SequentialAction(
                                claw.specimenScoring(),
                                tab1.build(),
                                claw.OuttakeClawOpen(),
                                new ParallelAction(
                                        claw.outtakeNeutral()
                                ),
                                tab2.build(),
                                claw.OuttakeIntake(),
                                tab3.build(),
                                claw.OuttakeClawClose(),
                                claw.specimenScoring(),
                                tab4.build(),
                                claw.OuttakeClawOpen(),
                                new ParallelAction(
                                    claw.outtakeNeutral()
                                ),
                                tab5.build(),
                                claw.OuttakeIntake(),
                                tab6.build(),
                                claw.OuttakeClawClose(),
                                claw.specimenScoring(),
                                tab7.build(),
                                claw.OuttakeClawOpen(),

                                new ParallelAction(
                                    claw.outtakeNeutral()
                                 ),
                                tabIntakePosition4th.build(),
                                claw.OuttakeIntake(),
                                tabIntake4th.build(),
                                claw.OuttakeClawClose(),
                                claw.specimenScoring(),
                                tabPlace4th.build(),
                                claw.outtakeNeutral()




                                //  tab3.build()
                                 )
                       // Run the PID loop for the slide
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