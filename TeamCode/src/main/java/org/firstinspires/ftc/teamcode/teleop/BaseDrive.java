package org.firstinspires.ftc.teamcode.teleop;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Methods;

@TeleOp(name="Base Drive", group="Beta")
public class BaseDrive extends LinearOpMode {
    HardwareDrive robot;
    ElapsedTime runtime;
    View relativeLayout;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareDrive(telemetry);
        runtime = new ElapsedTime();

        robot.init(hardwareMap);
//        robot.initCamera();
        runtime.reset();

        waitForStart();

        while (opModeIsActive()) {
            Methods.teleOp.robotDrive(Constants.DEFAULT_SPEED, robot, gamepad1, telemetry);
        }
    }
}
