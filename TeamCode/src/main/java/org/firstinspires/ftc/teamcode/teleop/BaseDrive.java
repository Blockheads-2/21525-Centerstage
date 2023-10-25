package org.firstinspires.ftc.teamcode.teleop;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Methods;

@TeleOp(name="Base Drive", group="Beta")
public class BaseDrive extends Methods.teleOp {
    HardwareDrive robot = new HardwareDrive();
    ElapsedTime runtime;

    View relativeLayout;
    @Override
    public void init() {
        robot.init(hardwareMap);
        /*robot.initCamera();*/

        runtime = new ElapsedTime();
        runtime.reset();

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
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
    }

    void UpdateTelemetry(){
        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", -gamepad1.left_stick_y);
        telemetry.addData("R", gamepad1.right_stick_x);

        //  telemetry.addData("Touch Sensor", robot.digitalTouch.getState());
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}