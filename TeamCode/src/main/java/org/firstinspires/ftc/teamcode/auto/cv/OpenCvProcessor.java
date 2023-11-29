package org.firstinspires.ftc.teamcode.auto.cv;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.Date;

public abstract class OpenCvProcessor implements VisionProcessor {

    Telemetry telemetry;
    Mat hsvOutput = new Mat();
    Mat blurOutput = new Mat();
    int lowHue = 120;
    int highHue = 180;
    public enum Location {
        LEFT,
        RIGHT,
        MID,
        NOT_FOUND
    }
    private Location location;

    static final Rect MID_ROI = new Rect(new Point(500, 100), new Point(750, 300)); //470-810
    static final Rect LEFT_ROI = new Rect(new Point(0, 100), new Point(200, 300));
    static final Rect RIGHT_ROI = new Rect(new Point(1280, 100), new Point(1080, 400));
    static double PERCENT_COLOR_THRESHOLD = 0.4;
    double[] hsvThresholdHue = {Constants.HSV_HUE_LOW_BLUE, Constants.HSV_HUE_HIGH_BLUE};
    double[] hsvThresholdSaturation = {Constants.HSV_SATURATION_LOW_BLUE, Constants.HSV_SATURATION_HIGH_BLUE};
    double[] hsvThresholdValue = {Constants.HSV_VALUE_LOW_BLUE, Constants.HSV_VALUE_HIGH_BLUE};


    public static class Builder {
        public OpenCvProcessor build(){
            return new OpenCvProcessorImpl();
        }
    }

    public abstract Pair<Mat, Date> getCameraFrame();

    private void processFrameHSV(Mat input){

        // Step Blur0:
        Mat blurInput = input;
        BlurType blurType = BlurType.get("Box Blur");
        double blurRadius = 5.5;
        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step HSV_Threshold0:
        Mat hsvThresholdInput = blurOutput;
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvOutput); //mat is the output

        Mat left = hsvOutput.submat(LEFT_ROI);
        Mat mid = hsvOutput.submat(MID_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double midValue = Core.sumElems(mid).val[0] / MID_ROI.area() / 255;

        telemetry.addData("Left Value", leftValue);
        telemetry.addData("Mid Value", midValue);

        left.release();
        mid.release();

        boolean elementLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean elementMid = midValue > PERCENT_COLOR_THRESHOLD;

        if (elementMid) {
            location = OpenCvProcessorImpl.Location.MID;
            telemetry.addData("Team Element Location", "mid");
        }
        else if (elementLeft) {
            location = OpenCvProcessorImpl.Location.LEFT;
            telemetry.addData("Team Element Location", "left");
        }
        else {
            location = OpenCvProcessorImpl.Location.RIGHT;
            telemetry.addData("Team Element Location", "right");
        }

        if (telemetry != null) {
            telemetry.addData("Hue Low", getLowHue());
            telemetry.addData("Hue High", getHighHue());
            telemetry.addData("Block Seen", isSeen());
            telemetry.update();
        }

//        Imgproc.cvtColor(mat, output, Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(hsvOutput, MID_ROI, new Scalar(255,0,0), 4);
        Imgproc.rectangle(hsvOutput, LEFT_ROI, new Scalar(255,0,0), 4);
        Imgproc.rectangle(hsvOutput, RIGHT_ROI, new Scalar(255,0,0), 4);

    }

    enum BlurType{
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            }
            else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            }
            else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            }
            else {
                return BOX;
            }
        }

        @Override
        public String toString() {
            return this.label;
        }
    }


    /**
     * Softens an image using one of several filters.
     * @param input The image on which to perform the blur.
     * @param type The blurType to perform.
     * @param doubleRadius The radius for the blur.
     * @param output The image in which to store the output.
     */

    private void blur(Mat input, BlurType type, double doubleRadius,
                      Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    /**
     * Segment an image based on hue, saturation, and value ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue
     * @param sat The min and max saturation
     * @param val The min and max value
     * @param out The image in which to store the output.
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    public OpenCvProcessorImpl.Location getLocation() {
        return location;
    }

    public int getLowHue(){
        return lowHue;
    }
    public int getHighHue(){
        return highHue;
    }

    public void setTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public boolean isSeen(){
        if (getLocation() == OpenCvProcessorImpl.Location.MID){
            return true;
        }
        return false;
    }
}
