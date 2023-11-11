package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.common.Constants.CPI;
import static java.lang.Thread.sleep;

import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.math.MathConstHead;

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
        public void updateTelemetry(HardwareDrive robot, TelemetryPacket packet, FtcDashboard dashboard) {
            packet.put("Top Left Power", robot.lf.getPower());
            packet.put("Top Right Power", robot.rf.getPower());
            packet.put("Bottom Left Power", robot.lb.getPower());
            packet.put("Bottom Right Power", robot.rb.getPower());

            packet.put("Top Left Velocity", robot.lf.getVelocity());
            packet.put("Top Right Velocity", robot.rf.getVelocity());
            packet.put("Bottom Left Velocity", robot.lb.getVelocity());
            packet.put("Bottom Right Velocity", robot.rb.getVelocity());

            dashboard.sendTelemetryPacket(packet);
        }
        /**
         * Drive to a position with a specified speed.
         *
         * @param drivePower value between 0 and 1. Default it to 0.5.
         * @param finalX     final x position of the robot relative to where it was before the method (inches)
         * @param finalY     final y position of the robot relative to where it was before the method (inches)
         */
        public void robotAutoStraightDrivePosition(
                double drivePower,
                double finalX,
                double finalY,
                HardwareDrive robot
        ) {

            MathConstHead heading = new MathConstHead();
            heading.setFinalPose(finalX, finalY);

            double distanceToTarget = heading.returnDistance();
            double angleToTarget = heading.returnDistance();

            double positiveAngularChangePosition = Math.cos(angleToTarget) + Math.sin(angleToTarget);
            double negativeAngularChangePosition = Math.cos(angleToTarget) - Math.sin(angleToTarget);

            robot.lf.setTargetPosition((int) (robot.lf.getCurrentPosition() + (positiveAngularChangePosition * CPI * distanceToTarget)));
            robot.lb.setTargetPosition((int) (robot.lb.getCurrentPosition() + (negativeAngularChangePosition * CPI * distanceToTarget)));
            robot.rf.setTargetPosition((int) (robot.rf.getCurrentPosition() + (negativeAngularChangePosition * CPI * distanceToTarget)));
            robot.rb.setTargetPosition((int) (robot.rb.getCurrentPosition() + (positiveAngularChangePosition * CPI * distanceToTarget)));

            for (DcMotorEx motor : List.of(robot.lf, robot.lb, robot.rf, robot.rb)) motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


            while (opModeIsActive()) {
                robot.lf.setVelocity((drivePower * 2700 * positiveAngularChangePosition));
                robot.lb.setVelocity((drivePower * 2700 * negativeAngularChangePosition));
                robot.rf.setVelocity((drivePower * 2700 * negativeAngularChangePosition));
                robot.rb.setVelocity((drivePower * 2700 * positiveAngularChangePosition));

                telemetry.addData("left front velocity", (drivePower * 2700 * positiveAngularChangePosition));
                telemetry.addData("left back velocity", (drivePower * 2700 * negativeAngularChangePosition));
                telemetry.addData("right front velocity", (drivePower * 2700 * negativeAngularChangePosition));
                telemetry.addData("right back velocity", (drivePower * 2700 * positiveAngularChangePosition));
            }
        }
    }

    public abstract static class teleOp extends OpMode {
        protected HardwareDrive robot = new HardwareDrive();
        public ElapsedTime runtime;
        protected FtcDashboard dashboard;
        protected TelemetryPacket packet;

        public void init_robot() {
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

        public void robotBaseDriveLoop(double drivePower) {
            double directionX = 0;
            double directionY = 0;
            double directionR = 0;
            int i1 = 0;

            if (Math.abs(gamepad1.left_stick_x) > 0.2)
                directionX = Math.pow(gamepad1.left_stick_x, 1);
            if (Math.abs(gamepad1.left_stick_y) > 0.2)
                directionY = -Math.pow(gamepad1.left_stick_y, 1);
            if (Math.abs(gamepad1.right_stick_x) > 0.2)
                directionR = Math.pow(gamepad1.right_stick_x, 1);

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
        }

        public double driveTrainSpeed() {
            double drivePower = Constants.DEFAULT_SPEED; //0.75
            if (gamepad1.right_bumper) drivePower = 1;
            else if (gamepad1.right_trigger >= 0.1) drivePower = 0.25;

            return drivePower;
        }

        public void robotBaseIntakeLoop() {
            robot.intake.setPower(gamepad1.left_trigger);
        }

        public void robotBaseMicroAdjustLoop(double drivePower) {
            if (gamepad1.dpad_up) {
                robot.lf.setPower(drivePower / 2);
                robot.lb.setPower(drivePower / 2);
                robot.rf.setPower(drivePower / 2);
                robot.rb.setPower(drivePower / 2);
            } else if (gamepad1.dpad_down) {
                robot.lf.setPower(-drivePower / 2);
                robot.lb.setPower(-drivePower / 2);
                robot.rf.setPower(-drivePower / 2);
                robot.rb.setPower(-drivePower / 2);
            } else if (gamepad1.dpad_right) {
                robot.lf.setPower(-drivePower / 2);
                robot.lb.setPower(drivePower / 2);
                robot.rf.setPower(-drivePower / 2);
                robot.rb.setPower(drivePower / 2);
            } else if (gamepad1.dpad_left) {
                robot.lf.setPower(drivePower / 2);
                robot.lb.setPower(-drivePower / 2);
                robot.rf.setPower(drivePower / 2);
                robot.rb.setPower(-drivePower / 2);
            }

        }

        protected void UpdateTelemetry(){
            telemetry.addData("X", gamepad1.left_stick_x);
            telemetry.addData("Y", -gamepad1.left_stick_y);
            telemetry.addData("R", gamepad1.right_stick_x);

            telemetry.addData("DPAD Up", gamepad1.dpad_up);
            telemetry.addData("DPAD Down", gamepad1.dpad_down);
            telemetry.addData("DPAD Left", gamepad1.dpad_left);
            telemetry.addData("DPAD Right", gamepad1.dpad_right);

            telemetry.addData("Top Left Encoder Position", robot.lf.getCurrentPosition());
            telemetry.addData("Top Right Encoder Position", robot.rf.getCurrentPosition());
            telemetry.addData("Bottom Left Encoder Position", robot.lb.getCurrentPosition());
            telemetry.addData("Bottom Right Encoder Position", robot.rb.getCurrentPosition());

            telemetry.addData("Top Left Velocity", robot.lf.getVelocity());
            telemetry.addData("Top Left Acceleration", robot.lf.getVelocity() / runtime.seconds()); //only works if holding down max power at the beginning of the opmode.


            telemetry.addData("Yaw", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("-Yaw", -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

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
    }
}
