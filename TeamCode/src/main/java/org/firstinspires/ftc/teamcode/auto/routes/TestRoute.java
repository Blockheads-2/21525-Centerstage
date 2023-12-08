package org.firstinspires.ftc.teamcode.auto.routes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;


@Autonomous(name = "Test Route", group = "Test")
// @Disabled
public class TestRoute extends LinearOpMode {
   AutoHub dispatch;
   HardwareDrive robot;

   @Config
   public static class RobotConstants {
       public static double turn = 90;
   }

   @Override
   public void runOpMode() throws InterruptedException {
       robot = new HardwareDrive();
       dispatch = new AutoHub(this);
//       dispatch.initCamera();

       waitForStart();

//       dispatch.turn(RobotConstants.turn);
//       dispatch.turnPID(RobotConstants.turn, 10);
       dispatch.turnAbsPID(RobotConstants.turn, 10);
//       dispatch.absoluteTurn(180, 0.3);

//       dispatch.variableHeading(0.6,24,24,4);

   }
}
