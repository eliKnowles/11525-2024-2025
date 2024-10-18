package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hermes_helper.data_types.ExtensionMode;
import org.firstinspires.ftc.teamcode.hermes_helper.util.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermes_helper.util.IMUV2;
import org.firstinspires.ftc.teamcode.hermes_helper.util.MechanumDrive;
import org.firstinspires.ftc.teamcode.hermes_helper.util.ServoV2;

@TeleOp(name="Robot Go Brrr", group="Linear OpMode")
public class RobotGoBrrr extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime slideTimer = new ElapsedTime();
    public static ExtensionMode slidestate = ExtensionMode.IDLE;

    // Create vars
    public static double x = 0;
    public static double y = 0;
    public static double rx = 0;

    public static ExtensionMode vSlideState = ExtensionMode.IDLE;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //MotorUtilV2 testMotor = new MotorUtilV2("test_motor");
        //testMotor.motor.setPower(1.0);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DcMotorV2 fLMotor = new DcMotorV2("front_left", hardwareMap);
        DcMotorV2 fRMotor = new DcMotorV2( "front_right", hardwareMap);
        DcMotorV2 bLMotor = new DcMotorV2("back_left", hardwareMap);
        DcMotorV2 bRMotor = new DcMotorV2("back_right", hardwareMap);

        ServoV2 bucketServoOne = new ServoV2("bucket_one", hardwareMap);
        ServoV2 bucketServoTwo = new ServoV2("bucket_two", hardwareMap);

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

        IMUV2 imu = new IMUV2("imu", hardwareMap,
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        MechanumDrive mechanumDrive = new MechanumDrive
                (fLMotor,
                fRMotor,
                bLMotor,
                bRMotor,
                imu,
                gamepad1, gamepad2, telemetry);

        // Wait for the game to start
        waitForStart();
        runtime.reset();

        // run until the end of the match
        while (opModeIsActive()) {

            if(gamepad1.options) imu.resetYaw();

            x = gamepad1.left_stick_y; // Y is reversed because gamepads are dumb
            y = -gamepad1.left_stick_x;
            rx = -gamepad1.right_stick_x;

            int hSlideTargetPosition;
            if (gamepad1.a) { // Extend the slide
                hSlideMotor.runToPosition(580);
            } else if (gamepad1.b) { // Retract the slide
                hSlideMotor.runToPosition(0);
            }

            int vSlideTargetPosition;
            if (gamepad1.y && slidestate == ExtensionMode.IDLE) { // Extend the slide
                vSlideMotorOne.runToPosition(-1000);
                vSlideMotorTwo.runToPosition(-1000);
                if(!vSlideMotorOne.isBusy()) vSlideMotorOne.stop();
                if(!vSlideMotorTwo.isBusy()) vSlideMotorTwo.stop();
                slidestate = ExtensionMode.EXTENDED;
            } else if (gamepad1.x && slidestate == ExtensionMode.EXTENDED) { // Retract the slide
                vSlideMotorOne.setPowerWithoutPosition(1.0d);
                vSlideMotorTwo.setPowerWithoutPosition(1.0d);
                vSlideMotorOne.setBreak();
                vSlideMotorTwo.setBreak();
                slideTimer.reset();
                slidestate = ExtensionMode.RETRACTED;
            }

            if(vSlideMotorOne.getCurrent(CurrentUnit.AMPS) > 6.5 && slidestate == ExtensionMode.RETRACTED) {
                vSlideMotorOne.stopAndReset();
                vSlideMotorTwo.stopAndReset();
                vSlideMotorOne.setBreak();
                vSlideMotorTwo.setBreak();
                slidestate = ExtensionMode.IDLE;
            }

            if(slideTimer.milliseconds() > 2000 && slidestate == ExtensionMode.RETRACTED) {
                vSlideMotorOne.stopAndReset();
                vSlideMotorTwo.stopAndReset();
                vSlideMotorOne.setBreak();
                vSlideMotorTwo.setBreak();
                slidestate = ExtensionMode.IDLE;
            }

            if(gamepad1.dpad_up) {
                bucketServoOne.setPosition(0.7d);
                bucketServoTwo.setPosition(0.7d);
            } else if(gamepad1.dpad_down) {
                bucketServoOne.setPosition(1.0d);
                bucketServoTwo.setPosition(1.0d);
            }

            mechanumDrive.fieldCentricDrive(x, y, rx);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "x (%.2f), y (%.2f), rx (%.2f)", x, y, rx);
            telemetry.addData("Ticks", "hSlideMotor: " + hSlideMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}