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

    public DcMotorEx lf;
    public DcMotorEx  rf;
    public DcMotorEx  rb;
    public DcMotorEx  lb;


    View relativeLayout;
    @Override
    public void init() {
        lf = hardwareMap.get(DcMotorEx.class, "left_front");
        lb = hardwareMap.get(DcMotorEx.class, "left_back");
        rf = hardwareMap.get(DcMotorEx.class, "right_front");
        rb = hardwareMap.get(DcMotorEx.class, "right_back");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        lf.setPower(-gamepad1.left_stick_y);
        rf.setPower(-gamepad1.left_stick_y);
        lb.setPower(-gamepad1.left_stick_y);
        rb.setPower(-gamepad1.left_stick_y);

        telemetry.addData("Top Left Encoder Position", lf.getCurrentPosition());
        telemetry.addData("Top Right Encoder Position", rf.getCurrentPosition());
        telemetry.addData("Bottom Left Encoder Position", lb.getCurrentPosition());
        telemetry.addData("Bottom Right Encoder Position", rb.getCurrentPosition());

        telemetry.addData("Top Left Motor Velocity", lf.getVelocity());
        telemetry.addData("Top Right Motor Velocity", rf.getVelocity());
        telemetry.addData("Bottom Left Motor Velocity", lb.getVelocity());
        telemetry.addData("Bottom Right Motor Velocity", rb.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}