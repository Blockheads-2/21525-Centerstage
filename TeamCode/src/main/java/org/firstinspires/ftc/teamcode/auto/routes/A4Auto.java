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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="A4 (Blue Near)", group="Autonomous")
public class A4Auto extends Methods.auto{
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        streamOpenCV(true);

        TeamElementDetectionPipeline.Location elementLocation;

        while (!opModeIsActive()){
            elementLocation = detector.getLocation();
        }

        elementLocation = detector.getLocation();

        stopOpenCV();

        waitForStart();

        //1 TILE = 24 INCHES

        switch (elementLocation) {
            //Park Left
            case LEFT:
                //Place
                constantHeading(0.2, 19, -32, 0, 0, 0);

                //Move Out of the way
                constantHeading(0.2, -5, 24, 0, 0, 0);

                //turn
                turnAbsPID(90);

                //Move to park
                constantHeading(0.3, 0, 42, 0, 0, 0); //y-value will changed based on the length of the claw
//
//                runOuttake(Constants.MID_OUTTAKE, 0.6);
//
//                //open claw
//
//                runOuttake(0, 0.5);

//                constantHeading(0.2, -15, 0, 0, 0, 0);


                break;

            case RIGHT:
                constantHeading(0.4, 50, 0, 0, 0, 0);

                //try splining
                //Place
//                spline(0.3, 15, -32, 15);
//
//                turnAbsPID(90);
//
//                //Move out of the way
//                constantHeading(0.2, 0, -15, 0, 0, 0);
//
//                //Park
//                constantHeading(0.3, -15, -44, 0, 0, 0);

                //try 90 degree movement to position
//                //Place
//                constantHeading(0.2, -3, -33, 0, 0, 0);
//
//                constantHeading(0.2, 17, 0, 0, 0, 0);
//
//                //Move Out of the way
//                constantHeading(0.2, 0, 3.5, 0, 0, 0);
//
//                //turn
//                turnAbsPID(90);
//
//                //Move to park
//                constantHeading(0.3, 7, -41, 0, 0, 0); //y-value will changed based on the length of the claw

//                runOuttake(Constants.MID_OUTTAKE, 0.6);
//
//                //open claw
//
//                runOuttake(0, 0.5);

//                constantHeading(0.2, -25, 0, 0, 0, 0);
                break;

            case MID:
                //Place
                constantHeading(0.2, 4, -34, 0, 0, 0);

                //Move Out of the way
                constantHeading(0.2, 0, 28, 0, 0, 0);

                //turn
                turnAbsPID(90);

                //Move to place on board
                constantHeading(0.3, 0, 48, 0, 0, 0); //y-value will changed based on the length of the claw

//                runOuttake(Constants.MID_OUTTAKE, 0.6);
//
//                //open claw
//
//                runOuttake(0, 0.5);

//                constantHeading(0.2, -20, 0, 0, 0, 0);

                break;
        }
    }
}
