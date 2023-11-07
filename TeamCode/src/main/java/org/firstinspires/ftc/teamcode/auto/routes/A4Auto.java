package org.firstinspires.ftc.teamcode.auto.routes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Methods;

@Autonomous(name="A4 Blue Autonomous", group="Autonomous")
public class A4Auto extends Methods.auto {
    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();
    HardwareDrive robot;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareDrive();

        initRobot(robot);
        initTelemetry(dashboard, packet);

        while (!opModeIsActive()){
            updateTelemetry();
        }

        waitForStart();

        telemetry.addLine("Loop started");
        telemetry.update();

        robotAutoStraightDrivePosition(0.25, 12, 12);
    }

}
