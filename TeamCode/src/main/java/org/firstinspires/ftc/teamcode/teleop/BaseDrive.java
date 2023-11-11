package org.firstinspires.ftc.teamcode.teleop;

import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Methods;

@TeleOp(name="Base Drive", group="Beta")
public class BaseDrive extends Methods.teleOp {


    View relativeLayout;
    @Override
    public void init() {
        init_robot();
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
        robotBaseIntakeLoop();

        UpdateTelemetry();
    }

    void UpdateTelemetry(){
        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", -gamepad1.left_stick_y);
        telemetry.addData("R", gamepad1.right_stick_x);

        telemetry.addData("Top Left Encoder Position", robot.lf.getCurrentPosition());
        telemetry.addData("Top Right Encoder Position", robot.rf.getCurrentPosition());
        telemetry.addData("Bottom Left Encoder Position", robot.lb.getCurrentPosition());
        telemetry.addData("Bottom Right Encoder Position", robot.rb.getCurrentPosition());

        telemetry.addData("Top Left Velocity", robot.lf.getVelocity());
        telemetry.addData("Top Left Acceleration", robot.lf.getVelocity() / runtime.seconds()); //only works if holding down max power at the beginning of the opmode.


        telemetry.addData("Yaw", -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

//        packet.put("X", gamepad1.left_stick_x);
//        packet.put("Y",  -gamepad1.left_stick_y);
//        packet.put("R", gamepad1.right_stick_x);

        packet.put("Top Left Power", robot.lf.getPower());
        packet.put("Top Right Power", robot.rf.getPower());
        packet.put("Bottom Left Power", robot.lb.getPower());
        packet.put("Bottom Right Power", robot.rb.getPower());

        packet.put("Top Left Velocity", robot.lf.getVelocity());
        packet.put("Top Right Velocity", robot.rf.getVelocity());
        packet.put("Bottom Left Velocity", robot.lb.getVelocity());
        packet.put("Bottom Right Velocity", robot.rb.getVelocity());

        dashboard.sendTelemetryPacket(packet);

        //  telemetry.addData("Touch Sensor", robot.digitalTouch.getState());
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}