package org.firstinspires.ftc.teamcode.teleop;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Base Drive", group="Ready")
//@Disabled
public class BaseDrive extends OpMode {
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive(telemetry);
    private ElapsedTime runtime = new ElapsedTime();

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() {
        try {
            robot.init(hardwareMap);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

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
//        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }
    void UpdatePlayer1(){
        double drivePower = DriveTrainSpeed();

        DriveTrainBase(drivePower);
    }

    void UpdatePlayer2(){

    }

    void UpdateTelemetry(){

        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", -gamepad1.left_stick_y);
        telemetry.addData("R", gamepad1.right_stick_x);

        telemetry.addData("Right trigger", gamepad1.right_trigger);
        telemetry.addData("Right bumper", gamepad1.right_bumper);

        telemetry.addData("right back power", robot.rb.getPower());
        telemetry.addData("right front power", robot.rf.getPower());
        telemetry.update();
    }

    void UpdateButton(){

    }

    double lfPower = 0;
    double lbPower = 0;
    double rfPower = 0;
    double rbPower = 0;

    void DriveTrainBase(double drivePower){
        double directionX = Math.pow(gamepad1.left_stick_x, 1); //Strafe
        double directionY = -Math.pow(gamepad1.left_stick_y, 1); //Forward
        double directionR = Math.pow(gamepad1.right_stick_x, 1); //Turn

        lfPower = (directionY + directionR + directionX) * drivePower;
        lbPower = (directionY - directionR - directionX) * drivePower;
        rfPower = (directionY - directionR - directionX) * drivePower;
        rbPower = (directionY + directionR + directionX) * drivePower;

        DriveMicroAdjust(0.4);

        robot.lf.setPower(lfPower);
        robot.lb.setPower(lbPower);
        robot.rf.setPower(rfPower);
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

        if (gamepad1.left_trigger != 0){
            lfPower = -power;
            lbPower = power;
            rfPower = -power;
            rbPower = power;
        }
        else if (gamepad1.right_trigger != 0){
            lfPower = power;
            lbPower = -power;
            rfPower = power;
            rbPower = -power;
        }
    }

    double DriveTrainSpeed(){
        double drivePower = 0.75;

        if (gamepad1.right_bumper)
            drivePower = 1;
        else if (gamepad1.left_bumper)
            drivePower = 0.25;


        return drivePower;
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
