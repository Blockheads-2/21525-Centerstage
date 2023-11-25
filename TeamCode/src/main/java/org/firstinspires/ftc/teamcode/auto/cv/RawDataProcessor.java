package org.firstinspires.ftc.teamcode.auto.cv;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionProcessor;

public abstract class RawDataProcessor implements VisionProcessor {
    public static class Builder{
        public RawDataProcessor build(Telemetry t){
            return new RawDataProcessorImpl(t);
        }
    }
}
