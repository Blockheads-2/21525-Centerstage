package org.firstinspires.ftc.teamcode.teleop;

import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Methods;

@TeleOp(name="Base Drive FTCLib", group="Beta")
@Disabled
public class BaseDriveFTCLib extends Methods.teleOp {


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
        double strafe = gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        robot.m_drive.driveFieldCentric(strafe, forward, turn, heading);

        UpdateTelemetry();
    }

    @Override
    public void stop() {

    }
}