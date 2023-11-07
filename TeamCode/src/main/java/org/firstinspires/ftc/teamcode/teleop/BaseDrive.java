package org.firstinspires.ftc.teamcode.teleop;

import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Methods;

@TeleOp(name="Base Drive", group="Beta")
public class BaseDrive extends Methods.teleOp {
    HardwareDrive robot = new HardwareDrive();
    ElapsedTime runtime;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    View relativeLayout;
    @Override
    public void init() {
        robot.init(hardwareMap);
        /*robot.initCamera();*/

        runtime = new ElapsedTime();
        runtime.reset();

        robot.imu.resetYaw();

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        robotBaseDriveLoop(driveTrainSpeed(), robot);
        UpdateTelemetry();
        robotBaseIntakeLoop(robot);
    }

    void UpdateTelemetry(){
        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", -gamepad1.left_stick_y);
        telemetry.addData("R", gamepad1.right_stick_x);

        telemetry.addData("Yaw", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

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