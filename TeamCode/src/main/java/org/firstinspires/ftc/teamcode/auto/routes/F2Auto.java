package org.firstinspires.ftc.teamcode.auto.routes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.auto.cv.TeamElementDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.Methods;

@Autonomous(name="F2 Auto (Red Far)", group="Autonomous")
public class F2Auto extends Methods.auto{

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        streamOpenCV(true);

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

        stopOpenCV();

//        dispatch.initCamera(telemetry);

        boolean aprilTagProcessor = false;
        boolean tfodProcessor = false;

//        dispatch.robot.getVisionPortal().setProcessorEnabled(dispatch.robot.getAprilTagProcessor(), aprilTagProcessor);
//        dispatch.robot.getVisionPortal().setProcessorEnabled(dispatch.robot.getTfodProcessor(), tfodProcessor);

//        while (!opModeIsActive()){
//            telemetry.addData("Team Element Position:", detector.getLocation());
//            telemetry.addData("Team Element Position va:", elementLocation);
//            telemetry.addData("April Tag Processor On?", aprilTagProcessor);
//            telemetry.addData("TFOD Processor On?", tfodProcessor);
//
////            if (aprilTagProcessor){
////                streamAprilTag();
////            } else if (tfodProcessor){
////                streamTfod();
////            }
//        }

        waitForStart();

<<<<<<< Updated upstream
        switch (elementLocation) {
            case LEFT:
                //...
                constantHeading(0.2, -7, 37, 0, 0, 0,0);
                constantHeading(0.2, 0, -10, 0, 0, 0);
                turnAbsPID(90);
                constantHeading(0.2, 0, 7, 0, 0, 0);
                break;

            case RIGHT:
                //...
                constantHeading(0.2, 7, 37, 0, 0, 0);
                constantHeading(0.2, 0, -10, 0, 0, 0);
                turnAbsPID(90);
                break;

            case MID:
                //...
                constantHeading(0.2, 0, 37, 0, 0, 0);
                constantHeading(0.2, 0, -10, 0, 0, 0);
                turnAbsPID(90);
                constantHeading(0.2, 0, 7, 0, 0, 0);

                break;
        }

//        constantHeading(0.2, 0, 7, 0, 0, 0);
        constantHeading(0.2, 0, 90, 0, 0, 0);

//        runIntake(-0.6, 4);
=======
        //1 tile = 24, Trapezoid tape to board = 12
        //Run 1
//        dispatch.constantHeading(0.5, 0, 24,  0.03, 0, 0);
//        dispatch.turn(90);
//        dispatch.constantHeading(0.5, 0, 96,  0.03, 0, 0);
//        dispatch.constantHeading(0.5, 0, -96, 0.03, 0, 0);
//        dispatch.turn(-90);
//        dispatch.constantHeading(0.5, 0, -24,  0.03, 0, 0);
//        //Run 2
//        dispatch.constantHeading(0.5, 0, 24,  0.03, 0, 0);
//        dispatch.turn(90);
//        dispatch.constantHeading(0.5, 0, 96,  0.03, 0, 0);
//        dispatch.constantHeading(0.5, 24, 0, 0.03, 0, 0);


        telemetry.update();
>>>>>>> Stashed changes
    }
}
