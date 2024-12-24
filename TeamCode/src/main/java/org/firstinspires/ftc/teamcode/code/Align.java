package org.firstinspires.ftc.teamcode.code;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.rr.PinpointDrive;
import java.lang.Math;

public class Align {
	// Declare Limelight
	private final Limelight3A limelight;
	private final HardwareMap hardwareMap;
	
	public Align(HardwareMap hardwareMap) {
		// Initialize Limelight
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		this.hardwareMap = hardwareMap;
	}
	
	double kpLateral = 0.05;  // Proportional gain for lateral adjustment
	double kpForward = 0.1;   // Proportional gain for forward/backward adjustment
	double forwardOffset = 5.0; // Fixed offset (camera-to-claw, in inches)
	double clawYOffset = -3.0; // Claw Y offset relative to the robot's center (adjust as needed)
	double tolerance = 0.5;   // Acceptable error range for tx and ty
	
	
	public void centerClawOverTarget(HardwareMap hardwareMap) {
		
		LLResult llResult = limelight.getLatestResult();
		
		// Initialize PinpointDrive with starting pose
		Pose2d initialPose = new Pose2d(-37, -65, Math.toRadians(90));
		PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
		Pose2d currentPose = drive.pinpoint.getPositionRR();
		
		double tx = llResult.getTx();
		double ty = llResult.getTy(); // Vertical angle offset
		
		double currentX = currentPose.position.x;
		double currentY = currentPose.position.y;
		
		if (!(Math.abs(tx) < tolerance && Math.abs(ty) < tolerance)) {
			
			// Step 2: Calculate new target position
			double lateralAdjustment = kpLateral * tx; // Strafe adjustment (X direction)
			double totalForwardAdjustment = (kpForward * ty) - forwardOffset + clawYOffset; // Adjust Y with offsets
			
			// New pose after adjustments
			double targetX = currentX + lateralAdjustment; // Adjust X for strafe
			double targetY = currentY + totalForwardAdjustment; // Adjust Y for backward/forward
			
			
			drive.actionBuilder(drive.pinpoint.getPositionRR())
				.strafeTo(new Vector2d(targetX, targetY))
				.build();
		}
	}
	
	public class centerOverTarget implements Action {
		
		@Override
		public boolean run (@NonNull TelemetryPacket telemetryPacket) {
			centerClawOverTarget(hardwareMap);
			return false;
		}
	}
	
	public Action CenterOverTarget() {
		return new centerOverTarget();
	}
}