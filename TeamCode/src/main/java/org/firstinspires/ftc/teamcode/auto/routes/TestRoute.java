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

@Autonomous(name="Test Route", group="Autonomous")
@Disabled
public class TestRoute extends Methods.auto{ //currently oriented for F2 route

    @Config
    public static class RobotConstants {
        public static double turn = 90;
        public static boolean blue = true;

        //LEFT
        public static double power1_left = 0.3;
        public static double x1_left = -7;
        public static double y1_left = 37;
        public static double power2_left = 0.2;
        public static double x2_left = 0;
        public static double y2_left = -7;
        public static double power3_left = 0.3;
        public static double x3_left = 0;
        public static double y3_left = 90;

        //MID
        public static double power1_mid = 0.3;
        public static double x1_mid = 0;
        public static double y1_mid = 37;
        public static double power2_mid = 0.2;
        public static double x2_mid = 0;
        public static double y2_mid = -7;
        public static double power3_mid = 0.3;
        public static double x3_mid = 0;
        public static double y3_mid = 85;

        //RIGHT
        public static double power1_right = 0.3;
        public static double x1_right = 7;
        public static double y1_right = 37;
        public static double power2_right = 0.2;
        public static double x2_right = 0;
        public static double y2_right = -7;
        public static double power3_right = 0.3;
        public static double x3_right = 0;
        public static double y3_right = 80;

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        streamOpenCV(RobotConstants.blue);

        TeamElementDetectionPipeline.Location elementLocation;

        while (!opModeIsActive()){
            elementLocation = detector.getLocation();
        }

        elementLocation = detector.getLocation();

        waitForStart();

        switch (elementLocation) {
            case LEFT:
                //...
                constantHeading(RobotConstants.power1_left, RobotConstants.x1_left, RobotConstants.y1_left, 0, 0,0);
                constantHeading(RobotConstants.power2_left, RobotConstants.x2_left, RobotConstants.y2_left, 0, 0, 0);
                turnAbsPID(RobotConstants.turn);
                constantHeading(RobotConstants.power3_left, RobotConstants.x3_left, RobotConstants.y3_left, 0, 0, 0);
                break;

            case MID:
                //...
                constantHeading(RobotConstants.power1_mid, RobotConstants.x1_mid, RobotConstants.y1_mid, 0, 0, 0);
                constantHeading(RobotConstants.power2_mid, RobotConstants.x2_mid, RobotConstants.y2_mid, 0, 0, 0);

                turnAbsPID(RobotConstants.turn);
                constantHeading(RobotConstants.power3_mid, RobotConstants.x3_mid, RobotConstants.y3_mid, 0, 0, 0);
                break;

            case RIGHT:
                //...
                constantHeading(RobotConstants.power1_right, RobotConstants.x1_right, RobotConstants.y1_right, 0, 0, 0);
                constantHeading(RobotConstants.power2_right, RobotConstants.x2_right, RobotConstants.y2_right, 0, 0, 0);
                turnAbsPID(RobotConstants.turn);
                constantHeading(RobotConstants.power3_right, RobotConstants.x3_right, RobotConstants.y3_right, 0, 0, 0);
                break;
        }
    }
}