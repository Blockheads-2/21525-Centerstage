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

@Autonomous(name="F4 (Red Near)", group="Autonomous")
public class F4Auto extends Methods.auto{
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        streamOpenCV(false);

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
                constantHeading(0.2, -3, -33, 0, 0, 0);

                constantHeading(0.2, 17, 0, 0, 0, 0);

                //Move Out of the way
                constantHeading(0.2, 0, 3.5, 0, 0, 0);

                //turn
                turnAbsPID(90);

                //Move to park
                constantHeading(0.3, 7, -41, 0, 0, 0); //y-value will changed based on the length of the claw

//                runOuttake(Constants.MID_OUTTAKE, 0.6);
//
//                //open claw
//
//                runOuttake(0, 0.5);

                //Move right of the board
                constantHeading(0.3, 34, 0, 0, 0, 0);

                //Move forward & park
                constantHeading(0.3, 0, 27, 0, 0, 0);


                break;

            case RIGHT:
                //Place
                constantHeading(0.2, -22, -33, 0, 0, 0);

                //Move Out of the way
                constantHeading(0.2, 0, 5, 0, 0, 0);

                //turn
                turnAbsPID(90);

                //Move to park
                constantHeading(0.3, 0, -44, 0, 0, 0); //y-value will changed based on the length of the claw
//
//                runOuttake(Constants.MID_OUTTAKE, 0.6);
//
//                //open claw
//
//                runOuttake(0, 0.5);

                //Move right of the board
                constantHeading(0.3, 34, 0, 0, 0, 0);

                //Move forward & park
                constantHeading(0.3, 0, 27, 0, 0, 0);

                break;

            case MID:
                //Place
                constantHeading(0.2, -4, -35, 0, 0, 0);

                //Move Out of the way
                constantHeading(0.2, 0, 5, 0, 0, 0);

                //turn
                turnAbsPID(90);

                //DONT PLACE ON THE BOARD --> Move to board and move to right immediately
                constantHeading(0.3, 0, -41, 0, 0, 0); //y-value will changed based on the length of the claw

//                runOuttake(Constants.MID_OUTTAKE, 0.6);
//
//                //open claw
//
//                runOuttake(0, 0.5);

                //Move right of the board
                constantHeading(0.3, 34, 0, 0, 0, 0);

                //Move forward & park
                constantHeading(0.3, 0, 27, 0, 0, 0);

                break;
        }
    }
}
