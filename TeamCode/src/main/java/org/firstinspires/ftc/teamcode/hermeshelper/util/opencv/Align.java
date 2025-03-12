package org.firstinspires.ftc.teamcode.hermeshelper.util.opencv;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.hermeshelper.rr.PinpointDrive;

public class Align {
    public final Limelight3A limelight;
    public final PinpointDrive drive; // PinpointDrive instance passed into Align
    private boolean aligned = false;


    // Constructor to accept the PinpointDrive instance
    public Align(Limelight3A limelight, PinpointDrive drive) {
        this.limelight = limelight;
        this.drive = drive;
        limelight.start();
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(1);
    }

    // Control Gains
    double kpLateral = 0.24


            ;  // Proportional gain for lateral adjustment
    double kpForward = 0.1;  // Proportional gain for forward/backward adjustment
    double forwardOffset = -1.2; // Fixed offset camera to claw
    double clawYOffset = 0;      // Claw Y offset relative to the robot's center
    double tolerance = 1;     // Tolerance for tx and ty

    // Add D gains
    double kdLateral = 0.001;  // Derivative gain for lateral adjustment
    double kdForward = 0.001;  // Derivative gain for forward adjustment

    // Variables for error tracking
    double lastTx = 0;
    double lastTy = 0;

    public void centerClawOverTarget() {
        aligned = false; // Reset alignment status
        long targetLostStartTime = -1; // Tracks when the target was first lost

        if (!aligned) {
            LLResult llResult = limelight.getLatestResult();

            if (!llResult.isValid()) {
                // If target is lost, start the timeout timer
                if (targetLostStartTime == -1) {
                    targetLostStartTime = System.currentTimeMillis();
                }

                // Check if the target has been missing for 500 ms
                if (System.currentTimeMillis() - targetLostStartTime >= 100) {
                    System.out.println("Target missing for too long. Completing without alignment.");
                   // break; // Exit the loop without completing alignment
                }

               // continue; // Skip the rest of the loop if no target is found
            } else {
                // Reset the timer if the target is found
                targetLostStartTime = -1;
            }

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
              //  break;
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