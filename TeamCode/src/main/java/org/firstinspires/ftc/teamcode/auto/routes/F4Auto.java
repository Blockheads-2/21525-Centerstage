package org.firstinspires.ftc.teamcode.auto.routes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;

@Autonomous(name="F4 Red Autonomous", group="Routes")
public class F4Auto extends LinearOpMode {
    AutoHub dispatch;
    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);
        dispatch.initCamera();

        while (!opModeIsActive());
        waitForStart();

        dispatch.constantHeading(0.5, 0,24, 5,0.1,0,0);
        dispatch.turn(90);
        dispatch.constantHeading(0.5, 0,40, 5,0.1,0,0);
        dispatch.turn(180);
        dispatch.constantHeading(0.5, 0,112, 5,0.1,0,0);
        dispatch.turn(180);
        dispatch.constantHeading(0.5, 0,112, 5,0.1,0,0);
        telemetry.update();
    }
}
