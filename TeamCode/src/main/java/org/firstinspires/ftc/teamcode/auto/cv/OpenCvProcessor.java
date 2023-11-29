package org.firstinspires.ftc.teamcode.auto.cv;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

import java.util.Date;

public abstract class OpenCvProcessor implements VisionProcessor {
    public static class Builder{
        public OpenCvProcessor build(Telemetry t){
            return new OpenCvProcessorImpl(t);
        }
    }

    public abstract Pair<Mat, Date> getCameraFrame();
}
