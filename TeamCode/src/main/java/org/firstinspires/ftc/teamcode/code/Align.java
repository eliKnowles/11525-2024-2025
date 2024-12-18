package org.firstinspires.ftc.teamcode.code;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.rr.PinpointDrive;

public class Align {
    public final Limelight3A limelight;
    public final PinpointDrive drive; // PinpointDrive instance passed into Align
    private boolean aligned = false;


    // Constructor to accept the PinpointDrive instance
    public Align(Limelight3A limelight, PinpointDrive drive) {
        this.limelight = limelight;
        this.drive = drive;
        limelight.start();
        limelight.setPollRateHz(90); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(1);
    }

    // Control Gains
    double kpLateral = 0.09;  // Proportional gain for lateral adjustment
    double kpForward = 0.07;  // Proportional gain for forward/backward adjustment
    double forwardOffset = -1; // Fixed offset camera to claw
    double clawYOffset = 0;      // Claw Y offset relative to the robot's center
    double tolerance = 0.12;     // Tolerance for tx and ty

    // Add D gains
    double kdLateral = 0.03;  // Derivative gain for lateral adjustment
    double kdForward = 0.03;  // Derivative gain for forward adjustment

    // Variables for error tracking
    double lastTx = 0;
    double lastTy = 0;

    public void centerClawOverTarget() {
        aligned = false; // Reset alignment status

        while (!aligned) {
            LLResult llResult = limelight.getLatestResult();


            double tx = llResult.getTx();
            double ty = llResult.getTy();

            // Calculate derivative terms
            double dTx = tx - lastTx;  // Change in tx
            double dTy = ty - lastTy;  // Change in ty

            lastTx = tx;
            lastTy = ty;

            // Check alignment
            if (Math.abs(tx) < tolerance && Math.abs(ty) < tolerance) {
                aligned = true;
                System.out.println("Alignment complete!");
                break;
            }

            // PD control adjustments
            double lateralAdjustment = (kpLateral * tx) + (kdLateral * dTx);
            double totalForwardAdjustment = (kpForward * ty) + (kdForward * dTy) - forwardOffset + clawYOffset;

            // Use the latest pose to calculate adjustments
            Pose2d currentPose = drive.getPoseEstimate();
            double targetX = currentPose.position.x + lateralAdjustment;
            double targetY = currentPose.position.y + totalForwardAdjustment;

            // Build and run the action
            Action moveAction = drive.actionBuilder(drive.getPoseEstimate())
                    .strafeTo(new Vector2d(targetX, targetY))
                    .build();

            Actions.runBlocking(moveAction);

            // Force update drive.pose
            drive.updatePoseEstimate();
            drive.pose = new Pose2d(targetX, targetY, Math.toRadians(90));
        }
    }


    public class centerOverTarget implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            centerClawOverTarget();
            return false; // Action completed
        }
    }

    public Action CenterOverTarget() {
        return new centerOverTarget();
    }
}
