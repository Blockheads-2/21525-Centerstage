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
                constantHeading(0.2, 12, -33, 0, 0, 0);

                //Move Out of the way
                constantHeading(0.2, 0, 5, 0, 0, 0);

                //turn
                turnAbsPID(-90);

                //Move to board
                constantHeading(0.3, 4, -40, 0, 0, 0); //y-value will changed based on the length of the claw; prob will have to tune x-value
//
//                runOuttake(Constants.MID_OUTTAKE, 0.6);
//
//                //open claw
//
//                runOuttake(0, 0.5);

                //Move left of the board
                constantHeading(0.3, -34, 0, 0, 0, 0);

                //Move forward & park
                constantHeading(0.3, 0, 27, 0, 0, 0);

                break;

            case RIGHT:
                //Place
                constantHeading(0.2, -3, -33, 0, 0, 0);

                constantHeading(0.2, -17, 0, 0, 0, 0);

                //Move Out of the way
                constantHeading(0.2, 0, 3.5, 0, 0, 0);

                //turn
                turnAbsPID(-90);

                //Move to park
                constantHeading(0.3, -5, -43, 0, 0, 0); //y-value will changed based on the length of the claw; prob will have to tune x-value

//                runOuttake(Constants.MID_OUTTAKE, 0.6);
//
//                //open claw
//
//                runOuttake(0, 0.5);

                //Move left of the board
                constantHeading(0.3, -34, 0, 0, 0, 0);

                //Move forward & park
                constantHeading(0.3, 0, 27, 0, 0, 0);


                break;

            case MID:
                //Place
                constantHeading(0.2, 0, -35, 0, 0, 0);

                //Move Out of the way
                constantHeading(0.2, 0, 7, 0, 0, 0);

                //turn
                turnAbsPID(-90);

                //Move to place on board
                constantHeading(0.3, 0, -40, 0, 0, 0); //y-value will changed based on the length of the claw; prob will have to tune x-value

//                runOuttake(Constants.MID_OUTTAKE, 0.6);
//
//                //open claw
//
//                runOuttake(0, 0.5);

                //Move left of the board
                constantHeading(0.3, -34, 0, 0, 0, 0);

                //Move forward & park
                constantHeading(0.3, 0, 27, 0, 0, 0);

                break;
        }
    }
}
