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

@Autonomous(name="A2 Blue Autonomous", group="Autonomous")
public class A2Auto extends Methods.auto {

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        while (!opModeIsActive()) dispatch.updateTelemetry();


        waitForStart();

        constantHeading(0.2, -55, 5, 0, 0, 0);

        spinIntake(-0.6, 4);
//        turn(RobotConstants.theta);
//        absoluteTurn(-90);
//        dispatch.turnPID(90, 6);
//        dispatch.turnAbsPID(90, 6)
        //        dispatch.turn(90);
//        dispatch.constantHeading(0.5, 0, 108,  0.03, 0, 0);
//        dispatch.constantHeading(0.5, 0, -108, 0.03, 0, 0);
//        dispatch.turn(90);
//        dispatch.constantHeading(0.5, 0, 27,  0.03, 0, 0);
    }
}
