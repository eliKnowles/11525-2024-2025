package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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

@TeleOp(name="Robot Go Brrr", group="Linear OpMode")
public class RobotGoBrrr extends OpMode {

    private static PinpointDrive otosDrive;

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
    private double speed = 1;

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
    private ServoV2 intakeWristServoTwo;

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

        otosDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

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

        // TODO: Set PIDF coefficients for hSlideMotor
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0, 0, 0, 0);
        hSlideMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        imu = new IMUV2("imu", hardwareMap);

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
//        sequence.create("Specimen_open_claw")
//                        .add(outtakePivotServo, .15f,0)
//                        .add(outtakeClawServo,.4f,0)
//                        .build();
//        sequence.create("specimen_close_claw")
//                        .add(outtakeClawServo, .68,0)
//                        .add(outtakePivotServo ,.25,0)
//                        .build();
        intakePivotServoTwo.setDirection(Servo.Direction.REVERSE);

        runtime.reset();
    }

    @Override
    public void loop() {
        otosDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        otosDrive.updatePoseEstimate();


//        odo.update();
//        Pose2D pos = odo.getPosition();
//        Pose2D vel = odo.getVelocity();

        if (gamepad1.options) imu.resetYaw();

        double wristLeftPosition = .05;
        double wristCenterPosition = .5;
        double wristRightPosition = .95;
        int wristPos = 0;

        x = gamepad1.left_stick_y; // Y is reversed because gamepads are dumb
        y = -gamepad1.left_stick_x;
        rx = -gamepad1.right_stick_x;


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
            wristPos = 0;
            sequence.run("intakeNeutral");
            currentTransferState = TransferState.H_EXTENDED;
            speed = 0.4;
        }
        if (gamepad1.dpad_up && currentTransferState == TransferState.TRANSFERED) {
            outtakeClawServo.setPosition(.4f);
            currentTransferState = TransferState.H_IDLE;
            speed = 1;
        }

        if (currentTransferState == TransferState.H_IDLE)
            sequence.run("idle");

        // Set target positions for slides based on gamepad input
        if (gamepad1.y) {
            targetSlidePosition = 830; // Example extension position for PIDF
        }else if (gamepad1.a) {
            targetSlidePosition = 0;
            outtakePivotServo.setPosition(.7);
        }

        if (hSlideMotor.getCurrentPosition() == 0);
        hSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wrist Servo Control
        if (gamepad1.left_bumper) wristPos -= 1;
        if (gamepad1.right_bumper) wristPos += 1;

        wristPos = Range.clip(wristPos, -1, 1);
        // intakeWristServo.setPosition(wristPos);

// Set servo position based on wristPos
        if (wristPos == -1) {
            intakeWristServoTwo.setPosition(wristLeftPosition);
        } else if (wristPos == 0) {
            intakeWristServoTwo.setPosition(wristCenterPosition);
        } else if (wristPos == 1) {
            intakeWristServoTwo.setPosition(wristRightPosition);
        }

//        if (currentTransferState  == TransferState.H_EXTENDED) {
//            fRMotor.setPower();  TODO: variables in mecanum drive code need to be made, so when extended you can multiply the motor power by .5 or smth
//            fLMotor.setPower();
//            bRMotor.setPower();
//            bLMotor.setPower();
//        }
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
        sequence.update();

    }

    private double computePIDFOutput(double targetPosition, double currentPosition) {
        double error = targetPosition - currentPosition;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        return kP * error + kI * integral + kD * derivative + kF * targetPosition;
    }
}