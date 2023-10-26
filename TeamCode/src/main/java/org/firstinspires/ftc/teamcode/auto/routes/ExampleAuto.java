package org.firstinspires.ftc.teamcode.auto.routes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.common.Constants;

@Autonomous(name = "Example Auto", group = "Routes")
//@Disabled
public class ExampleAuto extends LinearOpMode{

    AutoHub dispatch;
    Constants constants = new Constants();
    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);

        dispatch.initCamera();

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id", hardwareMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        CoreDetection detector = new CoreDetection(telemetry);
//        phoneCam.setPipeline(detector);
//
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                telemetry.addLine("Error Opening Camera");
//                telemetry.update();
//            }
//        });


        while (!opModeIsActive()) { //checks if play hasn't been pressed (in init stage)

        }

        waitForStart();

        //all movement goes here

        telemetry.update();
    }
}