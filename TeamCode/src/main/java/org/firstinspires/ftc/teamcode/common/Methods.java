package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.sun.tools.javac.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Methods {
    public static class teleOp {
        public static void robotDrive(double drivePower, HardwareDrive argRobot, Gamepad gamepad, Telemetry telemetry){
            double directionX = Math.pow(gamepad.left_stick_x, 1);
            double directionY = -Math.pow(gamepad.left_stick_y, 1);
            double directionR = Math.pow(gamepad.right_stick_x, 1);

            double lfPower = (directionX - directionY - directionR) * drivePower;
            double lbPower = (directionX + directionY - directionR) * drivePower;
            double rfPower = (directionX + directionY + directionR) * drivePower;
            double rbPower = (directionX - directionY + directionR) * drivePower;
            List<Double> args = List.of(lfPower, lbPower, rfPower, rbPower);

            double max = Math.max(Math.abs(lfPower), Math.abs(rfPower));
            max = Math.max(max, Math.abs(lbPower));
            max = Math.max(max, Math.abs(rbPower));

            if (max > 1.0) {
                lfPower /= max;
                lbPower /= max;
                rfPower /= max;
                rbPower /= max;
            }

            int i = 0;
            for (DcMotorEx chain : List.of(argRobot.lf, argRobot.lb, argRobot.rf, argRobot.rb)) {
                double arg = args.get(i);
                chain.setPower(arg);
                i++;
            }

            telemetry.addData("lf power:", lfPower);
            telemetry.addData("lb power:", lbPower);
            telemetry.addData("rf power:", rfPower);
            telemetry.addData("rb power:", rbPower);
            telemetry.update();
        }
    }
}
