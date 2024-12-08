package org.firstinspires.ftc.teamcode.code;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hermeshelper.datatypes.TransferState;
import org.firstinspires.ftc.teamcode.hermeshelper.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.hermeshelper.util.Sequence;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.IMUV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;
import org.firstinspires.ftc.teamcode.rr.PinpointDrive;

@TeleOp(name = "Robot Go Brrr", group = "Linear OpMode")
public class RobotGoBrrr extends OpMode {

    private static PinpointDrive drive;
    
    private TrajectoryActionBuilder spec;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // PIDF Constants for Vertical Slides
    private static final double kP = 0.01;
    private static final double kI = 0.0;
    private static final double kD = 0.02;
    private static final double kF = 0.0;

    // PIDF control variables for vertical slides
    private boolean force = false;
    private double targetSlideReset = 0;
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

    // Create vars
    public static double x = 0;
    public static double y = 0;
    public static double rx = 0;

    private TransferState currentTransferState = TransferState.H_IDLE;

    double oldTime = 0;

    int wristPos = 0;

    @Override
    public void init ( ) {
        drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        
        Pose2d beginPose = new Pose2d(0, 0, 0);

        GlobalTelemetry.init(telemetry);
        GlobalTelemetry.get().addData("Status", "Initialized");
        GlobalTelemetry.get().update();

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
        vSlideMotorTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeWristServo.setDirection(Servo.Direction.REVERSE);
        outtakePivotServo.setDirection(Servo.Direction.REVERSE);

        hSlideMotor.setDirection(FORWARD);

        // TODO: Set PIDF coefficients for hSlideMotor


        imu = new IMUV2("imu", hardwareMap);

        sequence = new Sequence();

        currentTransferState = TransferState.H_IDLE;

        sequence.create("transfer")
                .add(intakeWristServo, .14f, 0)
                .add(intakePivotServoOne, .55f, 100)
                .add(intakeWristServoTwo, .5f, 0)
                .add(hSlideMotor, 0f, 0)
                .add(outtakeClawServo, 0.85f, 700)
                .add(intakeClawServo, 0.4f, 100)
                .add(outtakePivotServo, .38f, 0)
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

        sequence.create("intakeGrab")
                .add(intakePivotServoOne, .02, 0)
                .add(intakeWristServo, .72f, 0)
                .add(intakeClawServo, .92f, 100)
                .add(intakePivotServoOne, .2f, 300)
                .build();
        
        sequence.create("Idle")
                .add(intakePivotServoOne, .5f, 0)
                .add(intakeWristServoTwo, .5f, 0)
                .add(intakeWristServo, .7f, 0)
                .add(outtakePivotServo, .76f, 0)
                .build();

        intakePivotServoTwo.setDirection(Servo.Direction.REVERSE);

        runtime.reset();
    }

    @Override
    public void loop ( ) {

        if(gamepad2.a){
            outtakePivotServo.setPosition(.2);
            outtakeClawServo.setPosition(.98);
            targetSlidePosition = 0;
        }

        if(gamepad2.b) {
            outtakeClawServo.setPosition(.83);

        }
        if(gamepad2.triangle) {
            targetSlidePosition = 255;
            outtakePivotServo.setPosition(.36);
        }

        if(gamepad2.square) {
            outtakeClawServo.setPosition(.98);
        }
        
        wristPos = 0;
        
        drive.setDrivePowers(new PoseVelocity2d(
            new Vector2d(
                -gamepad1.left_stick_y * speed,
                -gamepad1.left_stick_x * speed
            ),
            -gamepad1.right_stick_x
        ));

        drive.updatePoseEstimate();

        if(gamepad1.options) imu.resetYaw();

        x = gamepad1.left_stick_y; // Y is reversed because gamepads are dumb
        y = -gamepad1.left_stick_x;
        rx = -gamepad1.right_stick_x;

        // Set target positions for slides based on gamepad input
        if(gamepad1.y) {
            targetSlidePosition = 880; // Example extension position for PIDF
            speed = 0.7;
        } else if(gamepad1.a) {
            targetSlidePosition = 0;
            outtakePivotServo.setPosition(.85);
            speed = 1;
        }
        
        runTransfer();

        if(hSlideMotor.getCurrentPosition() == 0) {
            hSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        
        if(gamepad2.dpad_up) {
            targetSlideReset = targetSlidePosition - vSlideMotorOne.getCurrentPosition();
        }
        
        if(Math.abs(gamepad2.right_stick_y) > 0.25) {
            vSlideMotorOne.setPower(gamepad2.right_stick_y);
            vSlideMotorTwo.setPower(gamepad2.right_stick_y);
            force = true;
        } else if (gamepad2.right_stick_y < 0.05) {
            force = false;
        }

        // Wrist Servo Control
        if(gamepad1.left_bumper) wristPos -= 1;
        if(gamepad1.right_bumper) wristPos += 1;

        wristPos = Range.clip(wristPos, -1, 1);
        // intakeWristServo.setPosition(wristPos);

        // Set servo position based on wristPos
        runWrist();

        // PIDF Control for Vertical Slides
        if (!force) vSlidePIDF();

        // Show the elapsed game time and wheel power.
        GlobalTelemetry.get().addData("Status", "Run Time: " + runtime.toString());
        GlobalTelemetry.get().addData("Motors", "x (%.2f), y (%.2f), rx (%.2f)", x, y, rx);
        GlobalTelemetry.get().addData("Ticks", "hSlideMotor: " + hSlideMotor.getCurrentPosition());

        GlobalTelemetry.get().update();
        sequence.update();
    }

    public double computePIDFOutput (double targetPosition, double currentPosition) {
        double error = targetPosition - currentPosition;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        return kP * error + kI * integral + kD * derivative + kF * targetPosition;
    }

    public void runWrist ( ) {
        double wristLeftPosition = .05;
        double wristCenterPosition = .5;
        double wristRightPosition = .95;

        switch (wristPos) {
            case -1:
                intakeWristServoTwo.setPosition(wristLeftPosition);
                break;
            case 0:
                intakeWristServoTwo.setPosition(wristCenterPosition);
                break;
            case 1:
                intakeWristServoTwo.setPosition(wristRightPosition);
                break;
        }
    }

    public void runTransfer ( ) {
        
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
            
            if (currentTransferState != TransferState.H_IDLE) {
                speed = 0.5;
            } else {
                speed = 1;
            }
            
            currentTransferState = TransferState.H_EXTENDED;
        }
        
        if (gamepad1.dpad_up && currentTransferState == TransferState.TRANSFERED) {
            outtakeClawServo.setPosition(.98f);
            currentTransferState = TransferState.H_IDLE;
            speed = 1;
        }
        
        if (gamepad1.square && (currentTransferState == TransferState.H_INTAKEN || currentTransferState == TransferState.H_IDLE)) {
            sequence.run("intakeNeutralNoExtendo");
            currentTransferState = TransferState.H_EXTENDED;
        }
        
        if (currentTransferState == TransferState.H_IDLE) {
            sequence.run("idle");
        }

//        // Transfer code
//        switch (currentTransferState) {
//            case H_EXTENDED:
//                if(gamepad1.dpad_right) {
//                    sequence.run("intakeGrab");
//                    currentTransferState = TransferState.H_INTAKEN;
//                }
//                break;
//            case H_INTAKEN:
//                if(gamepad1.dpad_down) {
//                    sequence.run("transfer");
//                    currentTransferState = TransferState.TRANSFERED;
//                }
//                break;
//            case TRANSFERED:
//                if(gamepad1.dpad_up) {
//                    outtakeClawServo.setPosition(.7f);
//                    currentTransferState = TransferState.H_IDLE;
//                    speed = 1;
//                }
//                break;
//            case H_IDLE:
//                if(gamepad1.square) {
//                    sequence.run("intakeNeutralNoExtendo");
//                    currentTransferState = TransferState.H_EXTENDED;
//                } else {
//                    sequence.run("Idle");
//                }
//                break;
//            default:
//                break;
//        }
//
//        if(gamepad1.dpad_left && (currentTransferState == TransferState.H_IDLE || currentTransferState == TransferState.H_INTAKEN)) {
//            wristPos = 0;
//            sequence.run("intakeNeutral");
//            currentTransferState = TransferState.H_EXTENDED;
//            speed = 0.4;
//        }
    }

    public void vSlidePIDF ( ) {
        double currentSlidePosition = (vSlideMotorOne.getCurrentPosition());
        double pidfOutput = computePIDFOutput(targetSlidePosition - targetSlideReset, currentSlidePosition);
        vSlideMotorOne.setPower(pidfOutput);
        vSlideMotorTwo.setPower(pidfOutput);

        double newTime = getRuntime();
        double loopTime = newTime - oldTime;
        double frequency = 1 / loopTime;
        oldTime = newTime;
    }
}