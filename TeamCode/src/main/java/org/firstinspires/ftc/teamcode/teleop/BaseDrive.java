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
public class BaseDrive extends OpMode {
    HardwareDrive robot = new HardwareDrive();
    ElapsedTime runtime;

    View relativeLayout;
    @Override
    public void init() {
        robot.init(hardwareMap);
//        robot.initCamera();

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
        UpdatePlayer1();
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }

    void UpdatePlayer1(){
        double drivePower = DriveTrainSpeed();

        DriveTrainBase(drivePower);
        DriveTrainSpeed();
        DriveMicroAdjust(0.4);
    }

    void UpdatePlayer2(){

    }

    void UpdateTelemetry(){

        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", -gamepad1.left_stick_y);
        telemetry.addData("R", gamepad1.right_stick_x);

        //  telemetry.addData("Touch Sensor", robot.digitalTouch.getState());
        telemetry.update();
    }

    void UpdateButton(){

    }

    double lfPower=0;
    double lbPower=0;
    double rfPower=0;
    double rbPower=0;
    void DriveTrainBase(double drivePower){
        double directionX = Math.pow(gamepad1.left_stick_x, 1); //Strafe
        double directionY = -Math.pow(gamepad1.left_stick_y, 1); //Forward
        double directionR = Math.pow(gamepad1.right_stick_x, 1); //Turn

        lfPower = (directionY + directionR + directionX) * drivePower;
        lbPower = (directionY - directionR - directionX) * drivePower;
        rfPower = (directionY + directionR - directionX) * drivePower;
        rbPower = (directionY - directionR + directionX) * drivePower;

        DriveMicroAdjust(0.4);

        robot.lf.setPower(lfPower);
        robot.rf.setPower(lbPower);
        robot.lb.setPower(rfPower);
        robot.rb.setPower(rbPower);
    }

    void DriveMicroAdjust(double power){
        if (gamepad1.dpad_up){
            lfPower = power;
            lbPower = power;
            rfPower = power;
            rbPower = power;
        }
        else if (gamepad1.dpad_down){
            lfPower = -power;
            lbPower = -power;
            rfPower = -power;
            rbPower = -power;
        }
        else if (gamepad1.dpad_right){
            lfPower = power;
            lbPower = -power;
            rfPower = -power;
            rbPower = power;
        }
        else if (gamepad1.dpad_left){
            lfPower = -power;
            lbPower = power;
            rfPower = power;
            rbPower = -power;
        }

        if (gamepad1.left_trigger == 1){
            lfPower = -power;
            lbPower = power;
            rfPower = -power;
            rbPower = power;
        }
        else if (gamepad1.right_trigger == 1){
            lfPower = power;
            lbPower = -power;
            rfPower = power;
            rbPower = -power;
        }
    }

    double DriveTrainSpeed(){
        double drivePower = Constants.DEFAULT_SPEED; //0.75

        if (gamepad1.right_bumper) drivePower = 1;
        else if (gamepad1.left_bumper) drivePower = 0.25;

        return drivePower;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}