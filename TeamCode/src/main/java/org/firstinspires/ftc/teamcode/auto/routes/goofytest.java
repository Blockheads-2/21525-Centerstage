package org.firstinspires.ftc.teamcode.auto.routes;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@Autonomous(name="testthinggoof", group="nothing")
public class goofytest extends LinearOpMode {
    ElapsedTime runtime;
    View relativeLayout;
    DcMotorEx  lf;
    DcMotorEx  rf;
    DcMotorEx  rb;
    DcMotorEx  lb;

    @Override
    public void runOpMode() throws InterruptedException {
        runtime = new ElapsedTime();
        runtime.reset();

        lf  = hardwareMap.get(DcMotorEx.class, "left_front");
        lb  = hardwareMap.get(DcMotorEx.class, "left_back");
        rf = hardwareMap.get(DcMotorEx.class, "right_front");
        rb = hardwareMap.get(DcMotorEx.class, "right_back");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            lf.setPower(0.5);
            lb.setPower(0.5);
            rf.setPower(0.5);
            rb.setPower(0.5);
        }
    }
}
