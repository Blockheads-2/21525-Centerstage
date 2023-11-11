package org.firstinspires.ftc.teamcode.teleop;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Methods;

@TeleOp(name = "Base Drive", group = "Beta")
public class BaseDrive extends Methods.teleOp {

    View relativeLayout;

    @Override
    public void init() {
        initRobot();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robotBaseDriveLoop(driveTrainSpeed());
        robotBaseMicroAdjustLoop(driveTrainSpeed());
        robotBaseIntakeLoop();

        UpdateTelemetry();
    }

    @Override
    public void stop() {

    }
}