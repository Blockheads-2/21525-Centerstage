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

@Autonomous(name="F2 (Red Far)", group="Autonomous")
public class F2Auto extends Methods.auto{ //currently oriented for F2 route
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        streamOpenCV(false);

        TeamElementDetectionPipeline.Location elementLocation;

        while (!opModeIsActive()){
            elementLocation = detector.getLocation();
        }

        elementLocation = detector.getLocation();

        waitForStart();

        //1 TILE = 24 INCHES

        switch (elementLocation) {
            //Park Left
            case LEFT:
                //...
                //Place
                constantHeading(0.3, -7, 25, 0, 0, 0);
                //Move out
                constantHeading(0.2, 0, -5, 0, 0, 0);
                //Go forward
                constantHeading(0.2, 5, 7, 0, 0, 0);
                //Turn
                turnAbsPID(90);
                //Go to the board
                constantHeading(0.3, 0, 100, 0, 0, 0);
                //Move to the left
                constantHeading(0.3, -5, 0, 0, 0, 0);
                //Move forward and park
                constantHeading(0.3, 0, 7, 0, 0, 0);
                break;

            case RIGHT:
                //...
                //Place
                constantHeading(0.3, -7, -25, 0, 0, 0);
                //Move out
                constantHeading(0.2, 0, -5, 0, 0, 0);
                //Go forward
                constantHeading(0.2, 5, -7, 0, 0, 0);
                //Turn
                turnAbsPID(90);
                //Go to the board
                constantHeading(0.3, 0, -100, 0, 0, 0);
                //Move to the left
                constantHeading(0.3, -5, 0, 0, 0, 0);
                //Move forward and park
                constantHeading(0.3, 0, 7, 0, 0, 0);
                break;

            case MID:
                //...
                constantHeading(0.3, 0, 30, 0, 0, 0);
                constantHeading(0.3, 5, 0, 0, 0, 0);
                constantHeading(0.2, 0, -10, 0, 0, 0);
                constantHeading(0.2, -5, 10, 0, 0, 0);
                turnAbsPID(90);
                constantHeading(0.3, 0, 100, 0, 0, 0);
                //Move to the left
                constantHeading(0.3, -5, 0, 0, 0, 0);
                //Move forward and park
                constantHeading(0.3, 0, 7, 0, 0, 0);
                break;
        }
    }
}
