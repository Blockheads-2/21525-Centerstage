package org.firstinspires.ftc.teamcode.auto.routes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.auto.cv.GRIPDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.cv.TeamElementDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;

@Autonomous(name="Detect Element GRIP", group="Autonomous")
public class DetectElementGRIP extends LinearOpMode {
    AutoHub dispatch;

    FtcDashboard dashboard;
    TelemetryPacket packet;

    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        dispatch.initTelemetry(dashboard, packet);
//        dispatch.updateTelemetry();

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("Webcam 1","id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        GRIPDetectionPipeline detector = new GRIPDetectionPipeline(telemetry);
        TeamElementDetectionPipeline detector = new TeamElementDetectionPipeline(telemetry);
        phoneCam.setPipeline(detector);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Error Opening Camera");
                telemetry.update();
            }
        });

        while (!opModeIsActive()){
            updateValueDecrease.update(gamepad1.a);
            updateValueIncrease.update(gamepad1.b);

//            dispatch.updateTelemetry();
        }
        waitForStart();

        dispatch.initCamera(telemetry);

        while (opModeIsActive()){
            telemetryTfod();
        }
    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = dispatch.robot.getTfodProcessor().getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y); //units all in pixels.
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addData("- Image Size", "%d x %d", recognition.getImageWidth(), recognition.getImageHeight());
            telemetry.addData("Area", recognition.getHeight() * recognition.getWidth());
            telemetry.addData("maybe distance", 1.0 / (recognition.getHeight() * recognition.getWidth()));

            telemetry.addData("- Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));

            telemetry.addData("- Distance to Object", recognition.getWidth() / Constants.PIXEL_WIDTH_TO_DISTANCE_FROM_CAMERA);
            telemetry.addData("- Distance to Object (MAYBE)", recognition.getWidth() * Constants.PIXEL_WIDTH_TO_DISTANCE_FROM_CAMERA);

            telemetry.addData("Constant:", Constants.PIXEL_WIDTH_TO_DISTANCE_FROM_CAMERA);

            //todo:
            // look into voltage spikes in intake motor (getCurrent()) to see if motor is intaking stuff


        }   // end for() loop

    }   // end method telemetryTfod()
}
