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

@Autonomous(name="F2 Red Autonomous", group="Autonomous")
public class F2Auto extends LinearOpMode {
    AutoHub dispatch;
    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();
    TelemetryPacket packet;
    FtcDashboard dashboard;

    @Config
    public static class RobotConstants {
        public static int x=0;
        public static int y =27;
        public static double movePower = 0.1;
        public static double kp=0.03;
        public static double ki=0;
        public static double kd=0.01;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);
//        dispatch.initCamera();

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        dispatch.initTelemetry(dashboard, packet);
        dispatch.updateTelemetry();

        while (!opModeIsActive()){
            dispatch.updateTelemetry();
        }
        waitForStart();

        dispatch.constantHeadingV2(RobotConstants.movePower, RobotConstants.x, RobotConstants.y, RobotConstants.kp, RobotConstants.ki, RobotConstants.kd);
//        dispatch.turn(90);
//        dispatch.constantHeading(0.5, 0, 108,  0.03, 0, 0);
//        dispatch.constantHeading(0.5, 0, -108, 0.03, 0, 0);
//        dispatch.turn(90);
//        dispatch.constantHeading(0.5, 0, 27,  0.03, 0, 0);
    }
}
