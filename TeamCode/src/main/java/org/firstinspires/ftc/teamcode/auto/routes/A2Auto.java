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


        constantHeading(0.2, 0, 7, 0, 0, 0);
        constantHeading(0.2, -90, 0, 0, 0, 0);
        runIntake(-0.6, 4);
    }
}
