package org.firstinspires.ftc.teamcode.auto.routes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.Methods;

@Autonomous(name="F4 Red Autonomous", group="Autonomous")
public class F4Auto extends Methods.auto {

//    @Config
//    public static class RobotConstants {
//        public static int x=0;
//        public static int y =27;
//        public static double theta=90;
//        public static double movePower = 0.1;
//        public static double kp=0.03;
//        public static double ki=0;
//        public static double kd=0.01;
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        while (!opModeIsActive()) dispatch.updateTelemetry();


        waitForStart();

        constantHeading(0.2, 55, 5, 0, 0, 0);
        runIntake(-0.6, 4);
    }
}
