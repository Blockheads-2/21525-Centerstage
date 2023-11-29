package org.firstinspires.ftc.teamcode.auto.cv;

import android.graphics.Canvas;
import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.CanvasAnnotator;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.Date;
import java.util.concurrent.atomic.AtomicReference;

public class OpenCvProcessorImpl extends OpenCvProcessor {

    private final AtomicReference<Pair<Mat, Date>> cameraFrame =  new AtomicReference<>(); //we can store camera frames here (assuming that processFrame is called in some other thread)
    //AtomicReference is a faster way (than synchronized / volatile var, which makes an OS system call) to share data between threads because it uses only a single CPU instruction.


    public OpenCvProcessorImpl(){

    }

    @Override
    public void init(int width, int height, CameraCalibration calibration){

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos){

        Imgproc.cvtColor(input, hsvOutput, Imgproc.COLOR_RGB2HSV);
        cameraFrame.set(Pair.create(hsvOutput, new Date()));

        return hsvOutput;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext){
        if (userContext != null)
        {
            ((CanvasAnnotator) userContext).draw(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity);
        }
    }

    @Override
    public Pair<Mat, Date> getCameraFrame(){
        return cameraFrame.getAndSet(null);
    }
}
