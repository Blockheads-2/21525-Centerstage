package org.firstinspires.ftc.teamcode.common;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.cv.TeamElementDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

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
        protected AutoHub dispatch;
        protected FtcDashboard dashboard;
        protected TelemetryPacket packet;

        protected OpenCvCamera phoneCam;
        private int numFramesWithoutDetection=0;
        final float DECIMATION_HIGH = 3;
        final float DECIMATION_LOW = 2;
        final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
        final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
        protected TeamElementDetectionPipeline detector;
        protected TeamElementDetectionPipeline.Location location = TeamElementDetectionPipeline.Location.NOT_FOUND;

        public void initRobot(){
            dispatch = new AutoHub(this);

            dashboard = FtcDashboard.getInstance();
            packet = new TelemetryPacket();
            telemetry.setMsTransmissionInterval(50);

            dispatch.initTelemetry(dashboard, packet);
            dispatch.updateTelemetry();
        }

        public void initVisionPortal(){
            dispatch.initCamera(telemetry);
        }

        public void streamOpenCV(){
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("Webcam 1","id", hardwareMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            detector = new TeamElementDetectionPipeline(telemetry);
            phoneCam.setPipeline(detector);

            phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
                }

                @Override
                public void onError(int errorCode)
                {
                    telemetry.addLine("Error Opening Camera");
                    telemetry.update();
                }
            });

            location = detector.getLocation();
        }

        public void streamAprilTag(){
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
//            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            java.util.List<AprilTagDetection> myAprilTagDetections;  // list of all detections
            int myAprilTagIdCode;                           // ID code of current detection, in for() loop

// Get a list of AprilTag detections.
            myAprilTagDetections = dispatch.getAprilTagProcessor().getDetections();

            // If there's been a new frame...
            if(myAprilTagDetections != null)
            {
                telemetry.addData("FPS", dispatch.getVisionPortal().getFps());
//                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
//                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if(myAprilTagDetections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        dispatch.getAprilTagProcessor().setDecimation(DECIMATION_LOW);
//                        telemetry.addLine("Must lower decimation on the VisionPortal pipeline");
                    }

                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(myAprilTagDetections.get(0).ftcPose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        dispatch.getAprilTagProcessor().setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : myAprilTagDetections)
                    {
//                        if (detection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
//                            myAprilTagIdCode = detection.id;
//
//                            // Now take action based on this tag's ID code, or store info for later action.
//
//                        }
                        //Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                        telemetry.addData("Camera Streaming?", dispatch.getVisionPortal().getCameraState());
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f inches", detection.ftcPose.x));
                        telemetry.addLine(String.format("Translation Y: %.2f inches", detection.ftcPose.y));
                        telemetry.addLine(String.format("Translation Y: %.2f inches", detection.ftcPose.y-9));
                        telemetry.addLine(String.format("Translation Z: %.2f inches", detection.ftcPose.z));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", detection.ftcPose.yaw));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", detection.ftcPose.pitch));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", detection.ftcPose.roll));
                        telemetry.addLine(String.format("Range (distance to tag center): %.2f inches", detection.ftcPose.range));
                        telemetry.addLine(String.format("Bearing (delta yaw from apriltag center): %.2f degrees", detection.ftcPose.bearing));
                        telemetry.addLine(String.format("Elevation (delta pitch from apriltag center): %.2f degrees", detection.ftcPose.elevation));

                        packet.put("Rotation Yaw: %.2f degrees", detection.ftcPose.yaw);
                        packet.put("Rotation Pitch: %.2f degrees", detection.ftcPose.pitch);
                        packet.put("Rotation Roll: %.2f degrees", detection.ftcPose.roll);
                        packet.put("Range (distance to tag center): %.2f inches", detection.ftcPose.range);
                        packet.put("Bearing (delta yaw from apriltag center): %.2f degrees", detection.ftcPose.bearing);
//                        telemetry.update();
                    }
                }
            }
        }
        public void updateTelemetry() {
            packet.put("Top Left Power", dispatch.robot.lf.getPower());
            packet.put("Top Right Power", dispatch.robot.rf.getPower());
            packet.put("Bottom Left Power", dispatch.robot.lb.getPower());
            packet.put("Bottom Right Power", dispatch.robot.rb.getPower());

            packet.put("Top Left Velocity", dispatch.robot.lf.getVelocity());
            packet.put("Top Right Velocity", dispatch.robot.rf.getVelocity());
            packet.put("Bottom Left Velocity", dispatch.robot.lb.getVelocity());
            packet.put("Bottom Right Velocity", dispatch.robot.rb.getVelocity());
            
            packet.put("Yaw", -dispatch.robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            packet.put("Absolute Angle", dispatch.getAbsoluteAngle());
            packet.put("Get Angle", dispatch.getAngle());


            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
        /**
         * Drive to a position with a specified speed.
         *
         * @param movePower value between 0 and 1. Default it to 0.5.
         * @param x     final x position of the robot relative to where it was before the method (inches)
         * @param y     final y position of the robot relative to where it was before the method (inches)
         * @param kp    proportional constant for the PID loop
         * @param ki    integral constant for the PID loop
         * @param kd    derivative constant for the PID loop
         */
        public void constantHeading(double movePower, double x, double y, double kp, double ki, double kd) {
            dispatch.constantHeadingV2(movePower, x, y, kp, ki, kd);
        }

        /**
         * Drive to a position with a specified speed.
         *
         * @param movePower value between 0 and 1. Default it to 0.5.
         * @param x     final x position of the robot relative to where it was before the method (inches)
         * @param y     final y position of the robot relative to where it was before the method (inches)
         * @param theta final theta position of the robot.
         * @param kp    proportional constant for the PID loop
         * @param ki    integral constant for the PID loop
         * @param kd    derivative constant for the PID loop
         */
        public void constantHeading(double movePower, double x, double y, double theta, double kp, double ki, double kd){
            dispatch.constantHeadingV2(movePower, x, y, theta, kp, ki, kd);
        }

        public void turn(double theta){ //turning relative to its initial point
            dispatch.turnPID(theta, 6);
        }
        public void absoluteTurn(double theta){ //turning relative to field
            dispatch.turnAbsPID(theta, 6);
        }

        public void runIntake(double power, double timeout){
            dispatch.spinIntake(power, timeout);
        }
    }

    public abstract static class teleOp extends OpMode {
        protected HardwareDrive robot = new HardwareDrive();
        public ElapsedTime runtime;
        protected FtcDashboard dashboard;
        protected SpinPID outtakePositioner;
        protected TelemetryPacket packet;
        protected enum possibleLiftStates {
            BOTTOM,
            MIDDLE,
            TOP
        }
        protected possibleLiftStates liftState = possibleLiftStates.BOTTOM;

        Button outake = new Button();

        public void initRobot() {
            robot.init(hardwareMap);

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
            else if (gamepad1.left_bumper) drivePower = 0.25;

            return drivePower;
        }

        public void robotBaseIntakeLoop() {
            if (gamepad1.left_trigger != 0) robot.intake.setPower(gamepad1.left_trigger);
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

        public void robotBaseOuttakeLoop() {
            if (gamepad2.left_stick_y >= 0.2) {
                switch (liftState) {
                    case BOTTOM:
                        liftState = possibleLiftStates.MIDDLE;
                        robot.outtake.setTargetPosition(100);
                        outtakePositioner.setTargets(100, 0.1, 0.1, 0.1);
                        robot.outtake.setPower(outtakePositioner.update(robot.outtake.getCurrentPosition()));
                        robot.outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        break;
                    case MIDDLE:
                        liftState = possibleLiftStates.TOP;
                        robot.outtake.setTargetPosition(200);
                        outtakePositioner.setTargets(200, 0.1, 0.1, 0.1);
                        robot.outtake.setPower(outtakePositioner.update(robot.outtake.getCurrentPosition()));
                        robot.outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        break;
                }
            }
            if (gamepad2.left_stick_y <= -0.2) {
                switch (liftState) {
                    case TOP:
                        liftState = possibleLiftStates.MIDDLE;
                        break;
                    case MIDDLE:
                        liftState = possibleLiftStates.BOTTOM;
                        break;
                }
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

            telemetry.addData("Right trigger", gamepad1.right_trigger);
            telemetry.addData("Left trigger", gamepad1.left_trigger);

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

//        public void UpdateButton(){
//            outake.update(gamepad2.right_bumper);
//        }
    }
}