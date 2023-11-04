package org.firstinspires.ftc.teamcode.auto.routes;

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

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareDrive();
        waitForStart();

        telemetry.addLine("Loop started");
        telemetry.update();

        robotAutoStraightDrivePosition(0.25, 12, 12, robot);
    }
}
