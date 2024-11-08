package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="LucasCode", group="Linear OpMode")
public class LucasCode extends LinearOpMode {

    // Declare OpMode members.
    public static ElapsedTime runtime = new ElapsedTime();
    public static DcMotor fLMotor = null;
    public static DcMotor fRMotor = null;
    public static DcMotor bLMotor = null;
    public static DcMotor bRMotor = null;
//    public static DcMotor hSlide_motor = null;
//    public static DcMotorEx vSlide_motor_one = null;
//    public static DcMotorEx vSlide_motor_two = null;

    // Variables for horizontal slides movement
//    int hSlideTargetPosition;
//    int vSlideTargetPosition;
//    private enum vSlidestate {
//        EXTENDING,RETRACTING, DOWN
//    }
//    private vSlidestate slidestate = vSlidestate.DOWN;
//

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fLMotor = hardwareMap.get(DcMotor.class, "front_left");
        fRMotor = hardwareMap.get(DcMotor.class, "front_right");
        bLMotor = hardwareMap.get(DcMotor.class, "back_left");
        bRMotor = hardwareMap.get(DcMotor.class, "back_right");
        fLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        fRMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bRMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        hSlide_motor = hardwareMap.get(DcMotorEx.class, "h_slide");
//        hSlide_motor.setDirection(DcMotorSimple.Direction.FORWARD);
//        hSlide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        hSlide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        vSlide_motor_one = hardwareMap.get(DcMotorEx.class, "v_slide_one");
//        vSlide_motor_one.setDirection(DcMotorSimple.Direction.FORWARD);
//        vSlide_motor_one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        vSlide_motor_one.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        vSlide_motor_two = hardwareMap.get(DcMotorEx.class, "v_slide_two");
//        vSlide_motor_two.setDirection(DcMotorSimple.Direction.FORWARD);
//        vSlide_motor_two.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        vSlide_motor_two.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start
        waitForStart();
        runtime.reset();

        // run until the end of the match
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            double y = gamepad1.left_stick_x; // Y is reversed because gamepads are dumb
            double x = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            fLMotor.setPower(y + x + rx);
            fRMotor.setPower(y - x - rx);
            bLMotor.setPower(y - x + rx);
            bRMotor.setPower(y + x - rx);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
//            telemetry.addData("Slide Motor Position:", hSlide_motor.getCurrentPosition());
//            telemetry.addData("Vertical slide motor 1", vSlide_motor_one.getCurrentPosition());
//            telemetry.addData("Vertical slide motor 2", vSlide_motor_two.getCurrentPosition());
            telemetry.update();

//            if (gamepad1.a) { // Extend the slide
//                hSlideTargetPosition = 580; // Target position for extension
//                hSlide_motor.setTargetPosition(hSlideTargetPosition);
//                hSlide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hSlide_motor.setPower(1.0);
//            } else if (gamepad1.b) { // Retract the slide
//                hSlideTargetPosition = 0; // Target position for retraction
//                hSlide_motor.setTargetPosition(hSlideTargetPosition);
//                hSlide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hSlide_motor.setPower(1.0);
//            }
//
//            if (gamepad1.y && slidestate == vSlidestate.DOWN) { // Extend the slide
//                vSlideTargetPosition = -1000; // Target position for extension
//                vSlide_motor_one.setTargetPosition(vSlideTargetPosition);
//                vSlide_motor_one.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                vSlide_motor_one.setPower(1.0);
//                vSlide_motor_two.setTargetPosition(vSlideTargetPosition);
//                vSlide_motor_two.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                vSlide_motor_two.setPower(1.0);
//                slidestate = vSlidestate.EXTENDING;
//            } else if (gamepad1.x && slidestate == vSlidestate.EXTENDING) { // Retract the slide
//                vSlide_motor_one.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                vSlide_motor_two.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                vSlide_motor_one.setPower(.7);
//                vSlide_motor_two.setPower(.7);
//                slidestate = vSlidestate.RETRACTING;
//            }

//            if(vSlide_motor_one.getCurrent(CurrentUnit.AMPS) > 6.5 && slidestate == vSlidestate.RETRACTING ) {
//                vSlide_motor_one.setPower(0);
//                vSlide_motor_one.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                vSlide_motor_two.setPower(0);
//                vSlide_motor_two.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                slidestate = vSlidestate.DOWN;
            }
        }
    }
