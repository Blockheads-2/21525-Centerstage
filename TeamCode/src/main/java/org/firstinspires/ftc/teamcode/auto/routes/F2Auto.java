package org.firstinspires.ftc.teamcode.auto.routes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Methods;

@Autonomous(name = "F2 Red Autonomous", group = "Autonomous")
public class F2Auto extends Methods.auto {

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

        constantHeading(0.2, 0, 7, 0, 0, 0);
        constantHeading(0.2, 90, 0, 0, 0, 0);
        runIntake(-0.6, 4);
    }
}
