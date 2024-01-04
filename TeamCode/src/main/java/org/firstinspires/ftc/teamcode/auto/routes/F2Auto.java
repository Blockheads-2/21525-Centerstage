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
            case LEFT:
                //...
                constantHeading(0.3, -7, 37, 0, 0, 0,0);
                constantHeading(0.2, 0, -7, 0, 0, 0);
                turnAbsPID(90);
                constantHeading(0.3, 0, 90, 0, 0, 0);
                break;

            case RIGHT:
                //...
                constantHeading(0.3, 7, 37, 0, 0, 0);
                constantHeading(0.2, 0, -7, 0, 0, 0);
                turnAbsPID(90);
                constantHeading(0.3, 0, 80, 0, 0, 0);
                break;

            case MID:
                //...
                constantHeading(0.3, 0, 37, 0, 0, 0);
                constantHeading(0.2, 0, -7, 0, 0, 0);

                turnAbsPID(90);
                constantHeading(0.3, 0, 85, 0, 0, 0);
                break;
        }
    }
}
