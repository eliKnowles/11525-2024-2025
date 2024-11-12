package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hermeshelper.datatypes.ExtensionMode;
import org.firstinspires.ftc.teamcode.hermeshelper.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.hermeshelper.util.Sequence;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.IMUV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;

@TeleOp(name="Robot Go Brrr", group="Linear OpMode")
public class RobotGoBrrr extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
//    private final ElapsedTime slideTimer = new ElapsedTime();
//    public static ExtensionMode slidestate = ExtensionMode.IDLE;

    // Create vars
    public static double x = 0;
    public static double y = 0;
    public static double rx = 0;

    //public static ExtensionMode vSlideState = ExtensionMode.IDLE;

    enum transferState {            // states for scoring and whatnot
        H_IDLE, V_IDLE, TRANSFERED, H_EXTENDED, H_INTAKEN
    }

    private transferState currentTransferState = transferState.H_IDLE;

    GoBildaPinpointDriver odo;

    double oldTime = 0;

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        GlobalTelemetry.init(telemetry);

        GlobalTelemetry.get().addData("Status", "Initialized");
        GlobalTelemetry.get().update();

        //MotorUtilV2 testMotor = new MotorUtilV2("test_motor");
        //testMotor.motor.setPower(1.0);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(145, 60);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        DcMotorV2 fLMotor = new DcMotorV2("leftFront", hardwareMap);
        DcMotorV2 fRMotor = new DcMotorV2("rightFront", hardwareMap);
        DcMotorV2 bLMotor = new DcMotorV2("leftBack", hardwareMap);
        DcMotorV2 bRMotor = new DcMotorV2("rightBack", hardwareMap);

        ServoV2 intakePivotServoOne = new ServoV2("intake_pivot_one", hardwareMap);
        ServoV2 intakePivotServoTwo = new ServoV2("intake_pivot_two", hardwareMap);

        ServoV2 intakeClawServo = new ServoV2("intake_claw", hardwareMap);
        ServoV2 outtakeClawServo = new ServoV2("outtake_claw", hardwareMap);

        ServoV2 outtakePivotServo = new ServoV2("outtake_pivot_one", hardwareMap);

        ServoV2 intakeWristServo = new ServoV2("intake_wrist", hardwareMap);

        DcMotorV2 hSlideMotor = new DcMotorV2("h_slide", hardwareMap);
        hSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorV2 vSlideMotorOne = new DcMotorV2("v_slide_one", hardwareMap);
        vSlideMotorOne.setDirection(DcMotorSimple.Direction.FORWARD);
        vSlideMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlideMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorV2 vSlideMotorTwo = new DcMotorV2("v_slide_two", hardwareMap);
        vSlideMotorTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        vSlideMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlideMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fLMotor.setDirection(FORWARD);
        fRMotor.setDirection(REVERSE);
        bLMotor.setDirection(FORWARD);
        bRMotor.setDirection(REVERSE);

        fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hSlideMotor.setDirection(FORWARD);


        IMUV2 imu = new IMUV2("imu", hardwareMap);

//         MechanumDrive mechanumDrive = new MechanumDrive
//                (fLMotor,
//                fRMotor,
//                bLMotor,
//                bRMotor,
//                imu,
//                gamepad1, gamepad2);


        Sequence sequence = new Sequence();

        sequence.create("transfer")
                .add(intakePivotServoOne, .59f, 0) // intake arm servos move to transfer position
                //   .add(intakePivotServoTwo, .57f, 0)
                .add(intakeWristServo, 0f, 700)
                .add(hSlideMotor, 0f, 400)
                .add(outtakeClawServo, 0.65f, 400) //outtake claw grabs sample
                .add(intakeClawServo, 0.4f, 0) //release sample
                .add(outtakePivotServo, .35f, 0)


                //.add(outtakePivotServo, 0.5f, 0) //outtake pivot/claw moves to scoring position
                .build();

        sequence.create("intakeNeutral")
                //.add(intakeClawServo, 0.5f, 10)
                .add(hSlideMotor, 200f, 300)
                .add(outtakePivotServo, .6f, 0)
                .add(intakePivotServoOne, .05, 0) // intake arm servos move to transfer position
                //   .add(intakePivotServoTwo, 0f, 0)
                .add(intakeWristServo, .98f, 0)
                .add(intakeClawServo, .4f, 0)
                .build();

        sequence.create("intakeGrab")

                .add(intakeClawServo, .9f, 400)
                .add(intakePivotServoOne, .2f, 0)
                //     .add(intakePivotServoTwo, .2f, 0)


                .build();

        intakePivotServoTwo.setDirection(Servo.Direction.REVERSE);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // Wait for the game to start
        waitForStart();
        runtime.reset();

        // run until the end of the match
        while (opModeIsActive()) {

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            odo.update();
            Pose2D pos = odo.getPosition();
            Pose2D vel = odo.getVelocity();

            if (gamepad1.options) imu.resetYaw();

            x = gamepad1.left_stick_y; // Y is reversed because gamepads are dumb
            y = -gamepad1.left_stick_x;
            rx = -gamepad1.right_stick_x;

//            if (gamepad1.a) { // Extend the slide
//                hSlideMotor.runToPosition(280);
//            } else if (gamepad1.b) { // Retract the slide
//                hSlideMotor.runToPosition(0);
//            }

//            if (gamepad1.y && slidestate == ExtensionMode.IDLE) { // Extend the slide
//                vSlideMotorOne.runToPosition(-1000);
//                vSlideMotorTwo.runToPosition(-1000);
//                if(!vSlideMotorOne.isBusy()) vSlideMotorOne.stop();
//                if(!vSlideMotorTwo.isBusy()) vSlideMotorTwo.stop();
//                slidestate = ExtensionMode.EXTENDED;
//            } else if (gamepad1.x && slidestate == ExtensionMode.EXTENDED) { // Retract the slide
//                vSlideMotorOne.setPowerWithoutPosition(1.0d);
//                vSlideMotorTwo.setPowerWithoutPosition(1.0d);
//                vSlideMotorOne.setBreak();
//                vSlideMotorTwo.setBreak();
//                slideTimer.reset();
//                slidestate = ExtensionMode.RETRACTED;
//            }
//
//            if(vSlideMotorOne.getCurrent(CurrentUnit.AMPS) > 6.5 && slidestate == ExtensionMode.RETRACTED) {
//                vSlideMotorOne.stopAndReset();
//                vSlideMotorTwo.stopAndReset();
//                vSlideMotorOne.setBreak();
//                vSlideMotorTwo.setBreak();
//                slidestate = ExtensionMode.IDLE;
//            }

            if (gamepad1.dpad_right) {
                if (currentTransferState == transferState.H_EXTENDED) {
                    //sequence.run("intakeNeutral");
                    sequence.run("intakeGrab");
                    currentTransferState = transferState.H_INTAKEN;
                }
            }

            if (gamepad1.dpad_down) {
                if (currentTransferState == transferState.H_INTAKEN) {
                    sequence.run("transfer");
                    currentTransferState = transferState.TRANSFERED;
                }
            }
            if (gamepad1.dpad_left) {
                if (currentTransferState == transferState.H_IDLE || currentTransferState == transferState.H_INTAKEN) {
                    sequence.run("intakeNeutral");
                    currentTransferState = transferState.H_EXTENDED;
                }
            }
            if (gamepad1.dpad_up) {
                if (currentTransferState == transferState.TRANSFERED) {
                    outtakeClawServo.setPosition(.4f);
                    currentTransferState = transferState.H_IDLE;
                }
            }
            if (gamepad2.dpad_up) {
                Actions.runBlocking(drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
            }

//            if(slideTimer.milliseconds() > 2000 && slidestate == ExtensionMode.RETRACTED) {
//                vSlideMotorOne.stopAndReset();
//                vSlideMotorTwo.stopAndReset();
//                vSlideMotorOne.setBreak();
//                vSlideMotorTwo.setBreak();
//                slidestate = ExtensionMode.IDLE;
//            }

            //mechanumDrive.fieldCentricDrive(x, y, rx);

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;

            // Show the elapsed game time and wheel power.
            GlobalTelemetry.get().addData("Status", "Run Time: " + runtime.toString());
            GlobalTelemetry.get().addData("Motors", "x (%.2f), y (%.2f), rx (%.2f)", x, y, rx);
            GlobalTelemetry.get().addData("Ticks", "hSlideMotor: " + hSlideMotor.getCurrentPosition());

            GlobalTelemetry.get().update();
        }
    }
}
