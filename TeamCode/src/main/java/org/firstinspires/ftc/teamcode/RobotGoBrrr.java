package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hermeshelper.datatypes.TransferState;
import org.firstinspires.ftc.teamcode.hermeshelper.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.hermeshelper.util.Sequence;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.IMUV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@TeleOp(name="Robot Go Brrr", group="Linear OpMode")
public class RobotGoBrrr extends OpMode {
    private Follower follower;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // PIDF Constants for Vertical Slides
    private static final double kP = .01;
    private static final double kI = 0.0;
    private static final double kD = 0.02;
    private static final double kF = 0.0;

    // PIDF control variables for vertical slides
    private double targetSlidePosition = 0;
    private double lastError = 0;
    private double integral = 0;

    private DcMotorV2 fLMotor;
    private DcMotorV2 fRMotor;
    private DcMotorV2 bLMotor;
    private DcMotorV2 bRMotor;

    private ServoV2 intakePivotServoOne;
    private ServoV2 intakePivotServoTwo;
    private ServoV2 intakeClawServo;
    private ServoV2 outtakeClawServo;
    private ServoV2 outtakePivotServo;
    private ServoV2 intakeWristServo;

    private DcMotorV2 hSlideMotor;
    private DcMotorV2 vSlideMotorOne;
    private DcMotorV2 vSlideMotorTwo;

    private IMUV2 imu;

    private Sequence sequence;

//    private final ElapsedTime slideTimer = new ElapsedTime();
//    public static ExtensionMode slidestate = ExtensionMode.IDLE;

    // Create vars
    public static double x = 0;
    public static double y = 0;
    public static double rx = 0;

    // public static ExtensionMode vSlideState = ExtensionMode.IDLE;

    private TransferState currentTransferState = TransferState.H_IDLE;

//    GoBildaPinpointDriver odo;

    double oldTime = 0;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        Pose2d beginPose = new Pose2d(0, 0, 0);

        GlobalTelemetry.init(telemetry);
        GlobalTelemetry.get().addData("Status", "Initialized");
        GlobalTelemetry.get().update();

        //MotorUtilV2 testMotor = new MotorUtilV2("test_motor");
        //testMotor.motor.setPower(1.0);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
//        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        odo.setOffsets(145, 60);
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();

        fLMotor = new DcMotorV2(leftFrontMotorName, hardwareMap);
        fRMotor = new DcMotorV2(leftRearMotorName, hardwareMap);
        bLMotor = new DcMotorV2(rightFrontMotorName, hardwareMap);
        bRMotor = new DcMotorV2(rightRearMotorName, hardwareMap);

        intakePivotServoOne = new ServoV2("intake_pivot_one", hardwareMap);
        intakePivotServoTwo = new ServoV2("intake_pivot_two", hardwareMap);

        intakeClawServo = new ServoV2("intake_claw", hardwareMap);
        outtakeClawServo = new ServoV2("outtake_claw", hardwareMap);

        outtakePivotServo = new ServoV2("outtake_pivot_one", hardwareMap);
        intakeWristServo = new ServoV2("intake_wrist", hardwareMap);

        hSlideMotor = new DcMotorV2("h_slide", hardwareMap);
        hSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vSlideMotorOne = new DcMotorV2("v_slide_one", hardwareMap);
        vSlideMotorOne.setDirection(DcMotorSimple.Direction.FORWARD);
        vSlideMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vSlideMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vSlideMotorTwo = new DcMotorV2("v_slide_two", hardwareMap);
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

        imu = new IMUV2("imu", hardwareMap);

        sequence = new Sequence();

        sequence.create("transfer")
                .add(intakePivotServoOne, .59f, 0)
                .add(intakeWristServo, 0f, 700)
                .add(hSlideMotor, 0f, 400)
                .add(outtakeClawServo, 0.65f, 400)
                .add(intakeClawServo, 0.4f, 0)
                .add(outtakePivotServo, .28f, 0)
                .build();

        sequence.create("intakeNeutral")
                .add(hSlideMotor, 200f, 300)
                .add(outtakePivotServo, .7f, 0)
                .add(intakePivotServoOne, .02, 0)
                .add(intakeWristServo, 1f, 0)
                .add(intakeClawServo, .4f, 0)
                .build();

        sequence.create("intakeGrab")
                .add(intakeClawServo, .9f, 400)
                .add(intakePivotServoOne, .2f, 0)
                .build();

        intakePivotServoTwo.setDirection(Servo.Direction.REVERSE);

        follower.startTeleopDrive();

        runtime.reset();
    }

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();

//        odo.update();
//        Pose2D pos = odo.getPosition();
//        Pose2D vel = odo.getVelocity();

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

        if (gamepad1.dpad_right && currentTransferState == TransferState.H_EXTENDED) {
            //sequence.run("intakeNeutral");
            sequence.run("intakeGrab");
            currentTransferState = TransferState.H_INTAKEN;
        }

        if (gamepad1.dpad_down && currentTransferState == TransferState.H_INTAKEN) {
            sequence.run("transfer");
            currentTransferState = TransferState.TRANSFERED;
        }
        if (gamepad1.dpad_left && (currentTransferState == TransferState.H_IDLE || currentTransferState == TransferState.H_INTAKEN)) {
            sequence.run("intakeNeutral");
            currentTransferState = TransferState.H_EXTENDED;
        }
        if (gamepad1.dpad_up && currentTransferState == TransferState.TRANSFERED) {
            outtakeClawServo.setPosition(.4f);
            currentTransferState = TransferState.H_IDLE;
        }

        // Set target positions for slides based on gamepad input
        if (gamepad1.y) {
            targetSlidePosition = 800; // Example extension position
        } else if (gamepad1.a) {
            targetSlidePosition = 0;
        }

        // Wrist Servo Control
        int wristPos = 0;
        if (gamepad1.left_bumper) wristPos -= 1;

        if (gamepad1.right_bumper) wristPos += 1;

        wristPos = Range.clip(wristPos, -1, 1);
        intakeWristServo.setPosition(wristPos);

        // PIDF Control for Vertical Slides
        double currentSlidePosition = (vSlideMotorOne.getCurrentPosition());
        double pidfOutput = computePIDFOutput(targetSlidePosition, currentSlidePosition);
        vSlideMotorOne.setPower(pidfOutput);
        vSlideMotorTwo.setPower(pidfOutput);

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

    private double computePIDFOutput(double targetPosition, double currentPosition) {
        double error = targetPosition - currentPosition;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        return kP * error + kI * integral + kD * derivative + kF * targetPosition;
    }
}