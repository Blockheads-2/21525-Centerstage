package org.firstinspires.ftc.teamcode.auto.routes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;

@Autonomous(name="A2 Blue Autonomous", group="Autonomous")
public class A2Auto extends LinearOpMode {
    AutoHub dispatch;
    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();

    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);
//        dispatch.initCamera();

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        dispatch.initTelemetry(dashboard, packet);
        dispatch.updateTelemetry();

        while (!opModeIsActive()){
            dispatch.updateTelemetry();
        }
        waitForStart();

        dispatch.constantHeading(0.1, 0, 27,  0.03, 0, 0);
//        dispatch.turn(90);
//        dispatch.constantHeading(0.5, 0, 108,  0.03, 0, 0);
//        dispatch.constantHeading(0.5, 0, -108, 0.03, 0, 0);
//        dispatch.turn(90);
//        dispatch.constantHeading(0.5, 0, 27,  0.03, 0, 0);
    }
}
