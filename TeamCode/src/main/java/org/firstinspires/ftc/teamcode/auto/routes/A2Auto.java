package org.firstinspires.ftc.teamcode.auto.routes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;

@Autonomous(name="A2 Blue Autonomous", group="Autonomous")
public class A2Auto extends LinearOpMode {
    AutoHub dispatch;
    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);
//        dispatch.initCamera();

        while (!opModeIsActive());
        waitForStart();

        dispatch.constantHeading(0.5, 0, 27, 7, 0.1, 0, 0);
        dispatch.turn(90);
        dispatch.constantHeading(0.5, 0, 108, 7, 0.1, 0, 0);
        dispatch.constantHeading(0.5, 0, -108, 7, 0.1, 0, 0);
        dispatch.turn(90);
        dispatch.constantHeading(0.5, 0, 27, 7, 0.1, 0, 0);

        telemetry.update();
    }
}
