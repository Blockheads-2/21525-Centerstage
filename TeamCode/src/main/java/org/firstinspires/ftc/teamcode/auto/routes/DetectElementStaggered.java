package org.firstinspires.ftc.teamcode.auto.routes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.auto.cv.TeamElementDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.Methods;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;

@Autonomous(name="Detect Element Staggered", group="Autonomous")
public class DetectElementStaggered extends Methods.auto {

    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        streamOpenCV();

        boolean detectTeamElement = true;


        while (!opModeIsActive() && detectTeamElement){
            UpdateButton();

            if (detector.isSeen()) detectTeamElement = false;
//            dispatch.updateTelemetry();
        }

        stopOpenCV();

        dispatch.initCamera(telemetry);

        boolean aprilTagProcessor = false;
        boolean tfodProcessor = true;

        dispatch.robot.getVisionPortal().setProcessorEnabled(dispatch.robot.getAprilTagProcessor(), aprilTagProcessor);
        dispatch.robot.getVisionPortal().setProcessorEnabled(dispatch.robot.getTfodProcessor(), tfodProcessor);

        waitForStart();

        while (opModeIsActive()){
            UpdateButton();
            telemetry.addData("Team Element Position:", detector.getLocation());
            telemetry.addData("April Tag Processor On?", aprilTagProcessor);
            telemetry.addData("TFOD Processor On?", tfodProcessor);

            if (updateValueDecrease.is(Button.State.TAP)) {
                aprilTagProcessor = !aprilTagProcessor;
                dispatch.robot.getVisionPortal().setProcessorEnabled(dispatch.robot.getAprilTagProcessor(), aprilTagProcessor);
            } else if (updateValueDecrease.is(Button.State.TAP)){
                tfodProcessor = !tfodProcessor;
                dispatch.robot.getVisionPortal().setProcessorEnabled(dispatch.robot.getTfodProcessor(), tfodProcessor);
            }

            if (aprilTagProcessor){
                streamAprilTag();
            } else if (tfodProcessor){
                streamTfod();
            }
        }
    }

    public void UpdateButton(){
        updateValueDecrease.update(gamepad1.a);
        updateValueIncrease.update(gamepad1.b);
    }

//    @Override
//    public void runOpMode() throws InterruptedException {
//        dispatch = new AutoHub(this);
//
//        dashboard = FtcDashboard.getInstance();
//        packet = new TelemetryPacket();
//
//        dispatch.initTelemetry(dashboard, packet);
////        dispatch.updateTelemetry();
//
////        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("Webcam 1","id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
////        GRIPDetectionPipeline detector = new GRIPDetectionPipeline(telemetry);
//        TeamElementDetectionPipeline detector = new TeamElementDetectionPipeline(telemetry);
//        phoneCam.setPipeline(detector);
//
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                telemetry.addLine("Error Opening Camera");
//                telemetry.update();
//            }
//        });
//
//        boolean detectTeamElement = true;
//        while (!opModeIsActive() && detectTeamElement){
//            updateValueDecrease.update(gamepad1.a);
//            updateValueIncrease.update(gamepad1.b);
//
//            if (detector.isSeen()) detectTeamElement = false;
////            dispatch.updateTelemetry();
//        }
//
//        phoneCam.stopStreaming();
//        dispatch.initCamera(telemetry);
//
//        waitForStart();
//
//        while (opModeIsActive()){
//            telemetryTfod();
//        }
//    }
}
