package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hermeshelper.datatypes.ExtensionMode;
import org.firstinspires.ftc.teamcode.hermeshelper.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.hermeshelper.util.Sequence;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.IMUV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;

@TeleOp(name="Robot Go Brrr", group="Linear OpMode")
public class RobotGoBrrr extends LinearOpMode {

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

    // Slide state and timer
    enum transferState { H_IDLE, TRANSFERED, H_EXTENDED, H_INTAKEN }
    private transferState currentTransferState = transferState.H_IDLE;

    // Motor and hardware variables
    GoBildaPinpointDriver odo;
    DcMotorV2 vSlideMotorOne;
    DcMotorV2 vSlideMotorTwo;
    IMUV2 imu;
    double oldTime = 0;

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        GlobalTelemetry.init(telemetry);
        GlobalTelemetry.get().addData("Status", "Initialized");
        GlobalTelemetry.get().update();

        // Initialize the hardware variables
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
        hSlideMotor.setDirection(FORWARD);
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

        fLMotor.setDirection(FORWARD);
        fRMotor.setDirection(REVERSE);
        bLMotor.setDirection(FORWARD);
        bRMotor.setDirection(REVERSE);


        fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hSlideMotor.setDirection(FORWARD);
        intakePivotServoTwo.setDirection(Servo.Direction.REVERSE);

        imu = new IMUV2("imu", hardwareMap);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Sequence sequence = new Sequence();

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

        // Wait for the game to start
        waitForStart();
        runtime.reset();

        // Main loop
        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();
            odo.update();
            Pose2D pos = odo.getPosition();
            Pose2D vel = odo.getVelocity();

            if (gamepad1.options) imu.resetYaw();

            // Set target positions for slides based on gamepad input
            if (gamepad1.y) {
                targetSlidePosition = 800; // Example extension position
            } else if (gamepad1.a) {
               targetSlidePosition = 0;

            }

            // PIDF Control for Vertical Slides
            double currentSlidePosition = (vSlideMotorOne.getCurrentPosition());
            double pidfOutput = computePIDFOutput(targetSlidePosition, currentSlidePosition);
            vSlideMotorOne.setPower(pidfOutput);
            vSlideMotorTwo.setPower(pidfOutput);



            // Sequence management for scoring or positioning actions
            if (gamepad1.dpad_right && currentTransferState == transferState.H_EXTENDED) {
                sequence.run("intakeGrab");
                currentTransferState = transferState.H_INTAKEN;
            }

            if (gamepad1.dpad_down && currentTransferState == transferState.H_INTAKEN) {
                sequence.run("transfer");
                currentTransferState = transferState.TRANSFERED;
            }
            if (gamepad1.dpad_left && (currentTransferState == transferState.H_IDLE || currentTransferState == transferState.H_INTAKEN)) {
                sequence.run("intakeNeutral");
                currentTransferState = transferState.H_EXTENDED;
            }
            if (gamepad1.dpad_up && currentTransferState == transferState.TRANSFERED) {
                outtakeClawServo.setPosition(.4f);
                currentTransferState = transferState.H_IDLE;
            }
            if (gamepad2.dpad_up) {
                Actions.runBlocking(drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
            }

            // Telemetry updates
            GlobalTelemetry.get().addData("Slide Target Position", targetSlidePosition);
            GlobalTelemetry.get().addData("Slide Current Position", currentSlidePosition);
            GlobalTelemetry.get().addData("PIDF Output", pidfOutput);
            GlobalTelemetry.get().addData("Status", "Run Time: " + runtime.toString());
            GlobalTelemetry.get().addData("Motors", "x (%.2f), y (%.2f), rx (%.2f)", gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            GlobalTelemetry.get().addData("Ticks", "hSlideMotor: " + hSlideMotor.getCurrentPosition());
            GlobalTelemetry.get().update();

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;
        }
    }

    private double computePIDFOutput(double targetPosition, double currentPosition) {
        double error = targetPosition - currentPosition;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        return kP * error + kI * integral + kD * derivative + kF * targetPosition;
    }
}