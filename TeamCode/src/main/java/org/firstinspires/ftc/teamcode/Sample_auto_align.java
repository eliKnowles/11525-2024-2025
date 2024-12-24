package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.teamcode.rr.PinpointDrive;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import java.lang.Math;

public abstract class Sample_auto_align {
    // Declare Limelight
    private Limelight3A limelight;
    private LLResult llResult;

    public Sample_auto_align(HardwareMap hardwareMap) {
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public double calculateDistanceFromLimelight() {
        llResult = limelight.getLatestResult();
        double targetHeight = 10.0; // Height of the target in inches or cm
        double cameraHeight = 5.0;  // Height of the Limelight in inches or cm
        double cameraAngle = Math.toRadians(30); // Limelight mounting angle in degrees
        double ty = llResult.getTy(); // Replace with appropriate getter

        return (targetHeight - cameraHeight) / Math.tan(cameraAngle + Math.toRadians(ty));
    }

    public void centerClawOverTarget(HardwareMap hardwareMap) {
        llResult = limelight.getLatestResult();
        // Initialize PinpointDrive with starting pose
        Pose2d initialPose = new Pose2d(-37, -65, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        // Constants
        double clawOffsetX = 0;  // Lateral offset of claw (inches or cm)
        double clawOffsetY = 10; // Forward offset of claw (inches or cm)

        // Get Limelight data
        double tx = llResult.getTx(); // Replace with appropriate getter
        double distanceToTarget = calculateDistanceFromLimelight();

        // Adjust target position for claw offset
        double adjustedTargetX = distanceToTarget * Math.tan(Math.toRadians(tx)) - clawOffsetX;
        double adjustedTargetY = distanceToTarget - clawOffsetY;


        // Calculate the new angle the robot should align to
        double robotHeading = drive.pinpoint.getHeading();
        double newTargetAngle = Math.atan2(adjustedTargetX, adjustedTargetY);
        Pose2d currentPose = drive.pinpoint.getPositionRR();
        // Angle in radians

// Debug adjustedTargetY
        System.out.println("Adjusted Target Y: " + adjustedTargetY);

// Drive forward to position claw directly over the game piece
        drive.actionBuilder(drive.pinpoint.getPositionRR())
                .turn(newTargetAngle - robotHeading)// Turn to align
                // TODO: eli add it so it moves forward given the adjusted target y
                .build();
    }
}
