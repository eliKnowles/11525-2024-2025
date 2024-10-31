package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hermeshelper.datatypes.ExtensionMode;
import org.firstinspires.ftc.teamcode.hermeshelper.util.GlobalTelemetry;
import org.firstinspires.ftc.teamcode.hermeshelper.util.Sequence;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.IMUV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.mechanum_drive.MechanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;

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
        GlobalTelemetry.init(telemetry);
        
        GlobalTelemetry.get().addData("Status", "Initialized");
        GlobalTelemetry.get().update();

        //MotorUtilV2 testMotor = new MotorUtilV2("test_motor");
        //testMotor.motor.setPower(1.0);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DcMotorV2 fLMotor = new DcMotorV2("front_left", hardwareMap);
        DcMotorV2 fRMotor = new DcMotorV2( "front_right", hardwareMap);
        DcMotorV2 bLMotor = new DcMotorV2("back_left", hardwareMap);
        DcMotorV2 bRMotor = new DcMotorV2("back_right", hardwareMap);

        ServoV2 intakePivotServoOne = new ServoV2("intake_pivot_one", hardwareMap);
        ServoV2 intakePivotServoTwo = new ServoV2("intake_pivot_two", hardwareMap);

       // ServoV2 intakeClawServo = new ServoV2("intake_claw", hardwareMap);
       // ServoV2 outtakeClawServo = new ServoV2("outtake_claw", hardwareMap);

        //ServoV2 outtakePivotServo = new ServoV2("outtake_pivot", hardwareMap);

      //  ServoV2 intakeWristServo = new ServoV2("intake_wrist", hardwareMap);

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

        MechanumDrive mechanumDrive = new MechanumDrive
                (fLMotor,
                fRMotor,
                bLMotor,
                bRMotor,
                imu,
                gamepad1, gamepad2);

        Sequence sequence = new Sequence();
        sequence.create("transfer")
                .add(intakePivotServoOne, 0, 0) // intake arm servos move to transfer position
                .add(intakePivotServoTwo, 0, 0)
                //.add(intakeClawServo, 0.5f, 20 ) //release sample
                //.add(outtakeClawServo, 0.5f, 10) //outtake claw grabs sample
                //.add(outtakePivotServo, 0.5f, 0) //outtake pivot/claw moves to scoring position
                .build();

        sequence.create("intakeNeutral")
                //.add(intakeClawServo, 0.5f, 10)
                .add(intakePivotServoOne, 0.7f, 0) // intake arm servos move to transfer position
                .add(intakePivotServoTwo, 0.7f, 0)
                .build();

        sequence.create("intakeExtended")
                //.add(intakeClawServo, 0.5f, 10)
                .add(intakePivotServoOne, 1f, 0) // intake arm servos move to transfer position
                .add(intakePivotServoTwo, 1f, 0)
                .build();

//        sequence.create("intakeNeutralButton")
//                .add(intakeWristServo ,0,0) //TODO: add the value of whatever the second driver joystick
//                .add(outtakeClawServo, 0, 10) //outtake claw grabs sample
//                .add(outtakePivotServo, 0, 0) //outtake pivot/claw moves to scoring position
//                .build();


        intakePivotServoTwo.setDirection(Servo.Direction.REVERSE);

        // Wait for the game to start
        waitForStart();
        runtime.reset();

        // run until the end of the match
        while (opModeIsActive()) {

            if(gamepad1.options) imu.resetYaw();

            x = gamepad1.left_stick_y; // Y is reversed because gamepads are dumb
            y = -gamepad1.left_stick_x;
            rx = -gamepad1.right_stick_x;

            if (gamepad1.a) { // Extend the slide
                hSlideMotor.runToPosition(580);
            } else if (gamepad1.b) { // Retract the slide
                hSlideMotor.runToPosition(0);
            }

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

            if(gamepad1.dpad_right) {
                //sequence.run("intakeNeutral");
                intakePivotServoOne.setPosition(1f);
                intakePivotServoTwo.setPosition(1f);
            }
            if(gamepad1.dpad_left) {
                //sequence.run("intakeExtended");
                intakePivotServoOne.setPosition(0f);
                intakePivotServoTwo.setPosition(0f);
            }

            if(slideTimer.milliseconds() > 2000 && slidestate == ExtensionMode.RETRACTED) {
                vSlideMotorOne.stopAndReset();
                vSlideMotorTwo.stopAndReset();
                vSlideMotorOne.setBreak();
                vSlideMotorTwo.setBreak();
                slidestate = ExtensionMode.IDLE;
            }

            mechanumDrive.fieldCentricDrive(x, y, rx);

            // Show the elapsed game time and wheel power.
            GlobalTelemetry.get().addData("Status", "Run Time: " + runtime.toString());
            GlobalTelemetry.get().addData("Motors", "x (%.2f), y (%.2f), rx (%.2f)", x, y, rx);
            GlobalTelemetry.get().addData("Ticks", "hSlideMotor: " + hSlideMotor.getCurrentPosition());

            GlobalTelemetry.get().update();
        }
    }
}