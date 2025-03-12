package org.firstinspires.ftc.teamcode.hermeshelper.util.opencv;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.hermeshelper.rr.PinpointDrive;

public class TeleOpAlign {
    public final Limelight3A limelight;
    public final PinpointDrive drive;

    public TeleOpAlign(Limelight3A limelight, PinpointDrive drive) {
        this.limelight = limelight;
        this.drive = drive;
        limelight.start();
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
    }

    double kpLateral = 0.13;
    double kpForward = 0;
    double forwardOffset = -2.1;
    double clawYOffset = 0;

    double kdLateral = 0.001;
    double kdForward = 0.001;

    double lastTx = 0;
    double lastTy = 0;

    public void centerClawTeleOp() {
        drive.updatePoseEstimate();  // Add this at the start of centerClawTeleOp
        Pose2d currentPose = drive.getPoseEstimate();
        LLResult llResult = limelight.getLatestResult();

        double tx = llResult.getTx();
        double ty = llResult.getTy();

        double dTx = tx - lastTx;
        double dTy = ty - lastTy;

        lastTx = tx;
        lastTy = ty;

        double lateralAdjustment = (kpLateral * tx) + (kdLateral * dTx);
        double totalForwardAdjustment = (kpForward * ty) + (kdForward * dTy) - forwardOffset + clawYOffset;

        double targetX = currentPose.position.x + lateralAdjustment;
        double targetY = currentPose.position.y + totalForwardAdjustment;

        Action moveAction = drive.actionBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(targetX, targetY))
                .build();

        Actions.runBlocking(moveAction);
        drive.updatePoseEstimate();
    }

    public class CenterOverTargetTele implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            centerClawTeleOp();
            return false;
        }
    }

    public Action centerOverTargetTele() {
        return new CenterOverTargetTele();
    }
}
