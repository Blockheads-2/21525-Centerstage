package org.firstinspires.ftc.teamcode.auto.routes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.auto.cv.TeamElementDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.Methods;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="More Complex Test Routes", group="Autonomous")
@Disabled
public class MoreComplexTestRoutes extends Methods.auto{ //currently oriented for F2 route
//    Runnable runnable = new CameraThread();
    boolean aprilTagProcessor = false;
    boolean tfodProcessor = true;
    volatile int desiredTag = -1;
    volatile AprilTagDetection tag = null;

    Runnable runnable = new CameraThread();

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        streamOpenCV(false);

        boolean detectTeamElement = true;


        TeamElementDetectionPipeline.Location elementLocation;

        while (!opModeIsActive() && detectTeamElement){
//            if (detector.isSeen()) {
//                detectTeamElement = false;
//            }
//            dispatch.updateTelemetry();
            elementLocation = detector.getLocation();
        }

        elementLocation = detector.getLocation();

        if (elementLocation == TeamElementDetectionPipeline.Location.LEFT){ //refer to Appendix G of the FTC Centerstage Game Manual 2 for the AprilTag IDs.
            desiredTag = 4;
        } else if (elementLocation == TeamElementDetectionPipeline.Location.MID){
            desiredTag = 5;
        } else if (elementLocation == TeamElementDetectionPipeline.Location.RIGHT){
            desiredTag = 6;
        }

//        stopOpenCV();

//        dispatch.initCamera(telemetry);

        waitForStart();

        Thread cameraThread = new Thread(runnable);
        cameraThread.start(); //initializes thread.

        Thread.sleep(1000); //guestimate of how long it takes for cameraThread to fully complete
        //the most band-aid solution in the history of histories, but yeah it works now :)


        switch (elementLocation) {
            case LEFT:
                //...
                constantHeading(0.2, -7, 37, 0, 0, 0,0);
                constantHeading(0.2, 0, -5, 0, 0, 0);
                turnAbsPID(90);
                constantHeading(0.2, 0, 7, 0, 0, 0);
                break;

            case RIGHT:
                //...
                constantHeading(0.2, 7, 37, 0, 0, 0);
                constantHeading(0.2, 0, -5, 0, 0, 0);
                turnAbsPID(90);
                break;

            case MID:
                //...
                constantHeading(0.2, 0, 37, 0, 0, 0);
                constantHeading(0.2, 0, -5, 0, 0, 0);
                turnAbsPID(90);
                constantHeading(0.2, 0, 7, 0, 0, 0);

                break;
        }

//        constantHeading(0.2, 0, 7, 0, 0, 0);
        constantHeading(0.2, 0, 50, 0, 0, 0);


        boolean shouldMove = AprilTagMove(tag); //might have some crazy threading issues because CameraThread is modifying "tag", and I'm not sure if it'll sync well w/ the main thread.
        while (shouldMove) shouldMove = AprilTagMove(tag);
//        runIntake(-0.6, 4);

        cameraThread.join();
    }

    public AutoHub getDispatch(){
        return dispatch;
    }

    public class CameraThread implements Runnable{

        public CameraThread(){

        }
        public void run(){
            stopOpenCV();

            getDispatch().initCamera(telemetry);

            telemetry.addLine("Vision Portal Initiated!");

            getDispatch().robot.getVisionPortal().setProcessorEnabled(getDispatch().robot.getAprilTagProcessor(), aprilTagProcessor);
            getDispatch().robot.getVisionPortal().setProcessorEnabled(getDispatch().robot.getTfodProcessor(), tfodProcessor);

            while (aprilTagProcessor || tfodProcessor) {
                if (aprilTagProcessor) {
                    tag = streamAprilTag(desiredTag);
                }
                if (tfodProcessor) streamTfod();
            }

        }
    }
}
