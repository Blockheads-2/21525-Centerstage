package org.firstinspires.ftc.teamcode.common;

import static java.lang.Thread.sleep;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Methods {
    public static class general {
        public static void trySleep(long millis) {
            try {
                sleep(millis);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public abstract static class auto extends LinearOpMode {

    }

    public abstract static class teleOp extends OpMode {
        public void robotBaseDriveLoop(double drivePower, HardwareDrive robot){
            double directionX = 0;
            double directionY = 0;
            double directionR = 0;
            int i1 = 0;

            if (Math.abs(gamepad1.left_stick_x) > 0.2) directionX = Math.pow(gamepad1.left_stick_x, 1);
            if (Math.abs(gamepad1.left_stick_y) > 0.2) directionY = -Math.pow(gamepad1.left_stick_y, 1);
            if (Math.abs(gamepad1.right_stick_x) > 0.2) directionR = -Math.pow(gamepad1.right_stick_x, 1);

            double lfPower = (directionX + directionY + directionR) * drivePower;
            double lbPower = (-directionX + directionY + directionR) * drivePower;
            double rfPower = (-directionX + directionY - directionR) * drivePower;
            double rbPower = (directionX + directionY - directionR) * drivePower;
            List<Double> args = List.of(lfPower, lbPower, rfPower, rbPower);

            double max = Math.max(Math.abs(lfPower), Math.abs(rfPower));
            max = Math.max(max, Math.abs(lbPower));
            max = Math.max(max, Math.abs(rbPower));

            for (double power : args) {
                if (max > 1.0) power /= max;
            }

            for (DcMotorEx chain : List.of(robot.lf, robot.lb, robot.rf, robot.rb)) {
                double arg = args.get(i1);
                chain.setPower(arg);
                i1++;
            }

            telemetry.addData("lf power:", lfPower);
            telemetry.addData("lb power:", lbPower);
            telemetry.addData("rf power:", rfPower);
            telemetry.addData("rb power:", rbPower);
            telemetry.update();
        }
        public double driveTrainSpeed(){
            double drivePower = Constants.DEFAULT_SPEED; //0.75
            if (gamepad1.right_bumper) drivePower = 1;
            else if (gamepad1.left_bumper) drivePower = 0.25;

            return drivePower;
        }
    }
}
