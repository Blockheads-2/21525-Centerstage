package org.firstinspires.ftc.teamcode.teleop;

import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Methods;

@TeleOp(name="Test Motors", group="Beta")
public class TestMotors extends OpMode {

    HardwareDrive robot = new HardwareDrive();


    View relativeLayout;
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.plane.setPosition(0);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        robot.plane.setPosition(0.3);
    }

    @Override
    public void loop() {
//        robot.lf.setPower(-gamepad1.left_stick_y);
//        robot.rf.setPower(-gamepad1.left_stick_y);
//        robot.lb.setPower(-gamepad1.left_stick_y);
//        robot.rb.setPower(-gamepad1.left_stick_y);
//
//        telemetry.addData("Top Left Encoder Position", robot.lf.getCurrentPosition());
//        telemetry.addData("Top Right Encoder Position", robot.rf.getCurrentPosition());
//        telemetry.addData("Bottom Left Encoder Position", robot.lb.getCurrentPosition());
//        telemetry.addData("Bottom Right Encoder Position", robot.rb.getCurrentPosition());
//
//        telemetry.addData("Top Left Motor Velocity", robot.lf.getVelocity());
//        telemetry.addData("Top Right Motor Velocity", robot.rf.getVelocity());
//        telemetry.addData("Bottom Left Motor Velocity", robot.lb.getVelocity());
//        telemetry.addData("Bottom Right Motor Velocity", robot.rb.getVelocity());

        telemetry.addData("plane servo position", robot.plane.getPosition());

        robot.plane.setPosition(robot.plane.getPosition() - (gamepad1.left_stick_y * 0.1));
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}