package org.firstinspires.ftc.teamcode.hermeshelper.util.opencv;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hermeshelper.util.GlobalTelemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class OpenCamera {
    public OpenCvCamera camera;
    Pipeline visionPipeline;

    public OpenCamera(OpenCvCamera CVCamera, HardwareMap hardwareMap) {
        this.camera = CVCamera;

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        String webcamName = "Webcam 1"; // TODO: Replace with your actual webcam name in FTC config
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        // Initialize the custom VisionPipeline class
        visionPipeline = new Pipeline();
        camera.setPipeline(visionPipeline);

        // Implement AsyncCameraOpenListener directly
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming once the camera has opened successfully
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                GlobalTelemetry.get().addData("Camera Error", "Error code: " + errorCode);
                GlobalTelemetry.get().update();
            }
        });
    }
}