package org.firstinspires.ftc.teamcode.hermeshelper.util.opencv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {

    // Example variable to hold processed data
    private double someProcessedValue = 0.0;

    // Process each frame, applying color conversion and thresholding
    @Override
    public Mat processFrame(Mat input) {
        // Convert the frame to grayscale
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);

        // Apply a threshold to isolate specific colors
        Core.inRange(input, new Scalar(100), new Scalar(255), input);

        // Example processing: count non-zero pixels
        someProcessedValue = Core.countNonZero(input);

        return input; // Return the processed frame
    }

    // Getter method to access processed data from OpMode
    public double getSomeProcessedValue() {
        return someProcessedValue;
    }
}
