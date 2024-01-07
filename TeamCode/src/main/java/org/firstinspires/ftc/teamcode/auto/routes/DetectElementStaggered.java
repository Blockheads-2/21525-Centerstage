package org.firstinspires.ftc.teamcode.auto.routes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class DetectElementStaggered extends Methods.auto {

    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();

    Runnable runnable = new CameraThread();

    boolean aprilTagProcessor = true;
    boolean tfodProcessor = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        streamOpenCV(true);

        boolean detectTeamElement = true;

        while (!opModeIsActive()){
            telemetry.addData("Team Element Position:", detector.getLocation());
            if (detector.isSeen()) detectTeamElement = false;
            updateTelemetry();
        }
        
        waitForStart();

        Thread cameraThread = new Thread(runnable);
        cameraThread.start(); //initializes thread.

        Thread.sleep(1000); //guestimate of how long it takes for cameraThread to fully complete
        //the most band-aid solution in the history of histories, but yeah it works now :)

        while (opModeIsActive()){
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

            //error: can't read getVisionPortal() (thinks its null); some threading crossover issue, I believe
            if (aprilTagProcessor && dispatch.robot.getVisionPortal().getProcessorEnabled(dispatch.robot.getAprilTagProcessor())){
                streamAprilTag();
            } else if (dispatch.robot.getVisionPortal().getProcessorEnabled(dispatch.robot.getTfodProcessor())){
                streamTfod();
            }
            UpdateButton();
            updateTelemetry();
        }
        cameraThread.join();
    }

    public AutoHub getDispatch(){
        return dispatch;
    }

    public void UpdateButton(){
        updateValueDecrease.update(gamepad1.a);
        updateValueIncrease.update(gamepad1.b);
    }

    public class CameraThread implements Runnable{

        public CameraThread(){

        }
        public void run(){
            stopOpenCV(); //for future reference, check if opencv is running before stopping it.

            initVisionPortal();

            telemetry.addLine("Vision Portal Initiated!");
            telemetry.update();

            dispatch.robot.getVisionPortal().setProcessorEnabled(dispatch.robot.getAprilTagProcessor(), aprilTagProcessor);
            dispatch.robot.getVisionPortal().setProcessorEnabled(dispatch.robot.getTfodProcessor(), tfodProcessor);

//            while (true){
//                if (aprilTagProcessor){
//                    streamAprilTag();
//                } else if (tfodProcessor){
//                    streamTfod();
//                }
//            }

        }
    }

}
