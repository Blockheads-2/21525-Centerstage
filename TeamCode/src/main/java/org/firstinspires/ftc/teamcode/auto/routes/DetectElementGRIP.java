package org.firstinspires.ftc.teamcode.auto.routes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.cv.GRIPDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.cv.TeamElementDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

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
        dispatch.initCamera();

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        dispatch.initTelemetry(dashboard, packet);
        dispatch.updateTelemetry();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        GRIPDetectionPipeline detector = new GRIPDetectionPipeline(telemetry);
        phoneCam.setPipeline(detector);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
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

            dispatch.updateTelemetry();
        }
        waitForStart();
    }
}
