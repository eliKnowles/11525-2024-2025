package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hermeshelper.datatypes.TransferState;
import org.firstinspires.ftc.teamcode.hermeshelper.util.Sequence;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class BigBoiAuto extends LinearOpMode {

    public class Intake {
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

        public Intake(HardwareMap hardwareMap) {
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
            vSlideMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            vSlideMotorTwo = new DcMotorV2("v_slide_two", hardwareMap);
            vSlideMotorTwo.setDirection(DcMotorSimple.Direction.FORWARD);
            vSlideMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vSlideMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                    .add(intakeClawServo, .9f, 0)
                    .add(intakePivotServoOne, .2f, 300)
                    .build();
            currentTransferState = TransferState.H_IDLE;
            sequence.create("Idle")
                    .add(intakePivotServoOne, .5f, 0)
                    .add(intakeWristServoTwo, .5f, 0)
                    .add(intakeWristServo, .7f, 0)
                    .add(outtakePivotServo, .76f, 0)
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

        public class IntakeNeutral implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                sequence.run("intakeNeutral");
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
                sequence.run("idle");
                return false;
            }
        }

        public Action idle() {
            return new Idle();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-26, 63, 4.71239);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake claw = new Intake(hardwareMap);

        // vision here that outputs position
//        int initialPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-36, 13), 4.71239)
                //go to starting pos for sample pusher
                .strafeTo(new Vector2d(-47,13))
                //align to sample
                .strafeTo(new Vector2d(-47,52))
                // push sample
                .strafeTo(new Vector2d(-47,13))
                //return to position
                .strafeTo(new Vector2d(-57, 13))
                //align to sample
                .strafeTo(new Vector2d(-57, 52))
                //push sample
                .strafeTo(new Vector2d(-57, 13))
                //return to position
                .strafeTo(new Vector2d(-61, 13))
                //align to sample
                .strafeTo(new Vector2d(-61, 52))
                //push sample
                .strafeTo(new Vector2d(-42,54))
                .waitSeconds(.8);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-5, 37, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(-42, 54, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-5, 37, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-42, 54, Math.toRadians(90)), Math.toRadians(0));

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.idle());


//        while (!isStopRequested() && !opModeIsActive()) {
//            int position = initialPosition;
//            telemetry.addData("Position during Init", position);
//            telemetry.update();
//        }
//
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
                new SequentialAction(
                        tab1.build(),
                        claw.intakeGrab(),
                        claw.transfer(),
                        claw.intakeNeutral(),
                        claw.idle(),
                        tab2.build()
                        // TODO: add placing the thing
                )
        );
    }
}