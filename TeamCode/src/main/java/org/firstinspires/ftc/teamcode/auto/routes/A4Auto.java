package org.firstinspires.ftc.teamcode.auto.routes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;

@Autonomous(name="A4 Blue Autonomous", group="Autonomous")
public class A4Auto extends LinearOpMode {
    AutoHub dispatch;
    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);

        waitForStart();

        telemetry.addLine("Loop started");
        telemetry.update();
        dispatch.constantHeading(0.75, 0, 12, 0.1, 0, 0);

    }
}
