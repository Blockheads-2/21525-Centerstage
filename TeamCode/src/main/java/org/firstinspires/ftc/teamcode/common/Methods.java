package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.common.Constants.MIN_LIFT_CLICKS;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.auto.cv.TeamElementDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.common.pid.SpinPID;

import java.util.List;
import java.util.concurrent.TimeUnit;

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
        private int numFramesWithoutDetection = 0;
        final float DECIMATION_HIGH = 3;
        final float DECIMATION_LOW = 2;
        final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
        final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
        protected TeamElementDetectionPipeline detector;
        protected TeamElementDetectionPipeline.Location location = TeamElementDetectionPipeline.Location.NOT_FOUND;

        public interface TelemetryFunc {
            void update();
        }

        public void initRobot() {
            dispatch = new AutoHub(this);

            dashboard = FtcDashboard.getInstance();
            packet = new TelemetryPacket();
            telemetry.setMsTransmissionInterval(50);

//            TelemetryFunc updateTele = () -> {
//                UpdateTelemetry();
//            };

            dispatch.initTelemetry(dashboard, packet);

//            UpdateTelemetry();
        }

        public void initVisionPortal() {
            dispatch.initCamera(telemetry);
        }

        public void streamOpenCV(boolean blue) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("Webcam 1", "id", hardwareMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            detector = new TeamElementDetectionPipeline(telemetry);
            detector.blueOrRed(blue);
            phoneCam.setPipeline(detector);

            phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addLine("Error Opening Camera");
                    telemetry.update();
                }
            });

            location = detector.getLocation();
        }

        public void stopOpenCV() {
            phoneCam.stopStreaming();
        }

        public void streamAprilTag() {
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
            if (myAprilTagDetections != null) {
                telemetry.addData("FPS", dispatch.getVisionPortal().getFps());
//                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
//                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if (myAprilTagDetections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        dispatch.getAprilTagProcessor().setDecimation(DECIMATION_LOW);
//                        telemetry.addLine("Must lower decimation on the VisionPortal pipeline");
                    }

                }
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (myAprilTagDetections.get(0).ftcPose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        dispatch.getAprilTagProcessor().setDecimation(DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : myAprilTagDetections) {
                        //Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                        telemetry.addData("Camera Streaming?", dispatch.getVisionPortal().getCameraState());
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f inches", detection.ftcPose.x));
                        telemetry.addLine(String.format("Translation Y: %.2f inches", detection.ftcPose.y));
                        telemetry.addLine(String.format("Translation Y: %.2f inches", detection.ftcPose.y - 9));
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

        public AprilTagDetection streamAprilTag(int desiredTagId) {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
//            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            java.util.List<AprilTagDetection> myAprilTagDetections;  // list of all detections

// Get a list of AprilTag detections.
            myAprilTagDetections = dispatch.getAprilTagProcessor().getDetections();

            // If there's been a new frame...
            if (myAprilTagDetections != null) {
                telemetry.addData("FPS", dispatch.getVisionPortal().getFps());
//                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
//                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if (myAprilTagDetections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        dispatch.getAprilTagProcessor().setDecimation(DECIMATION_LOW);
//                        telemetry.addLine("Must lower decimation on the VisionPortal pipeline");
                    }

                }
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (myAprilTagDetections.get(0).ftcPose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        dispatch.getAprilTagProcessor().setDecimation(DECIMATION_HIGH);
                    }

                    for (AprilTagDetection detection : myAprilTagDetections) {
                        if (detection.metadata != null && detection.id == desiredTagId) {  // This check for non-null Metadata is not needed for reading only ID code.

                            //Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                            telemetry.addData("Camera Streaming?", dispatch.getVisionPortal().getCameraState());
                            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                            telemetry.addLine(String.format("Translation X: %.2f inches", detection.ftcPose.x));
                            telemetry.addLine(String.format("Translation Y: %.2f inches", detection.ftcPose.y));
                            telemetry.addLine(String.format("Translation Y: %.2f inches", detection.ftcPose.y - 9));
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
                            return detection;
                        }

                    }
                }
            }
            return null;
        }


        public void streamTfod() {
            java.util.List<Recognition> currentRecognitions = dispatch.robot.getTfodProcessor().getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y); //units all in pixels.
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                telemetry.addData("- Image Size", "%d x %d", recognition.getImageWidth(), recognition.getImageHeight());
                telemetry.addData("Area", recognition.getHeight() * recognition.getWidth());
                telemetry.addData("maybe distance", 1.0 / (recognition.getHeight() * recognition.getWidth()));

                telemetry.addData("- Angle", recognition.estimateAngleToObject(AngleUnit.DEGREES));

                telemetry.addData("- Distance to Object", recognition.getWidth() / Constants.PIXEL_WIDTH_TO_DISTANCE_FROM_CAMERA);
                telemetry.addData("- Distance to Object (MAYBE)", recognition.getWidth() * Constants.PIXEL_WIDTH_TO_DISTANCE_FROM_CAMERA);

                telemetry.addData("Constant:", Constants.PIXEL_WIDTH_TO_DISTANCE_FROM_CAMERA);

                //todo:
                // look into voltage spikes in intake motor (getCurrent()) to see if motor is intaking stuff


            }   // end for() loop

        }

        public void updateTelemetry() { //always use this method to update telemetry
//            packet.put("Top Left Power", dispatch.robot.lf.getPower());
//            packet.put("Top Right Power", dispatch.robot.rf.getPower());
//            packet.put("Bottom Left Power", dispatch.robot.lb.getPower());
//            packet.put("Bottom Right Power", dispatch.robot.rb.getPower());
//
//            packet.put("Top Left Velocity", dispatch.robot.lf.getVelocity());
//            packet.put("Top Right Velocity", dispatch.robot.rf.getVelocity());
//            packet.put("Bottom Left Velocity", dispatch.robot.lb.getVelocity());
//            packet.put("Bottom Right Velocity", dispatch.robot.rb.getVelocity());

            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }

        /**
         * Drive to a position with a specified speed.
         *
         * @param movePower value between 0 and 1. Default it to 0.5.
         * @param x         final x position of the robot relative to where it was before the method (inches)
         * @param y         final y position of the robot relative to where it was before the method (inches)
         * @param kp        proportional constant for the PID loop
         * @param ki        integral constant for the PID loop
         * @param kd        derivative constant for the PID loop
         */
        public void constantHeading(double movePower, double x, double y, double kp, double ki, double kd) {
            dispatch.constantHeadingV2(movePower, x, y, kp, ki, kd);
        }

        /**
         * Drive to a position with a specified speed.
         *
         * @param movePower value between 0 and 1. Default it to 0.5.
         * @param x         final x position of the robot relative to where it was before the method (inches)
         * @param y         final y position of the robot relative to where it was before the method (inches)
         * @param theta     final theta position of the robot.
         * @param kp        proportional constant for the PID loop
         * @param ki        integral constant for the PID loop
         * @param kd        derivative constant for the PID loop
         */
        public void constantHeading(double movePower, double x, double y, double theta, double kp, double ki, double kd) {
            dispatch.constantHeadingV2(movePower, x, y, theta, kp, ki, kd);
        }

        public void spline(double speed, double x, double y, double timeoutS) {
            dispatch.variableHeading(speed, x, y, timeoutS);
        }

        public void spline(double speed, double x, double y) {
            dispatch.variableHeading(speed, x, y);
        }

        public boolean AprilTagMove(AprilTagDetection tag) {
            return dispatch.AprilTagMove(tag);
        }

        public void turnAbsPID(double theta) { //turning relative to its initial point
            dispatch.turnAbsPID(theta, 6);
        }

        public void runIntake(double power, double timeout) {
//            dispatch.spinIntake(power, timeout);
        }

        public void runOuttake(int clickTarget, double power) {
            while (opModeIsActive() && Math.abs(dispatch.robot.lift.getCurrentPosition() - clickTarget) >= 15) {
                dispatch.robot.lift.setTargetPosition(clickTarget);
                dispatch.robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                dispatch.robot.lift.setPower(power);
            }
            dispatch.robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void UpdateTelemetry() {
            //        packet.put("Top Left Power", robot.lf.getPower());
//        packet.put("Top Right Power", robot.rf.getPower());
//        packet.put("Bottom Left Power", robot.lb.getPower());
//        packet.put("Bottom Right Power", robot.rb.getPower());
//
//        packet.put("Top Left Encoder Position", robot.lf.getCurrentPosition());
//        packet.put("Top Right Encoder Position", robot.rf.getCurrentPosition());
//        packet.put("Bottom Left Encoder Position", robot.lb.getCurrentPosition());
//        packet.put("Bottom Right Encoder Position", robot.rb.getCurrentPosition());
//        packet.put("Yaw", -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//        packet.put("Yaw (Absolute Angle)", getAbsoluteAngle());

            telemetry.addData("Top Left Encoder Position", dispatch.robot.lf.getCurrentPosition());
            telemetry.addData("Top Right Encoder Position", dispatch.robot.rf.getCurrentPosition());
            telemetry.addData("Bottom Left Encoder Position", dispatch.robot.lb.getCurrentPosition());
            telemetry.addData("Bottom Right Encoder Position", dispatch.robot.rb.getCurrentPosition());

            telemetry.addData("Team Element Position:", detector.getLocation());
            telemetry.addData("April Tag Processor On?", dispatch.getVisionPortal().getProcessorEnabled(dispatch.getAprilTagProcessor()));
            telemetry.addData("TFOD Processor On?", dispatch.getVisionPortal().getProcessorEnabled(dispatch.getTfodProcessor()));

            telemetry.update();
//        dashboard.sendTelemetryPacket(packet);
        }
    }

    public abstract static class teleOp extends OpMode {
        protected HardwareDrive robot = new HardwareDrive();
        public ElapsedTime runtime;
        protected FtcDashboard dashboard;
        protected TelemetryPacket packet;

        Constants.LiftState liftState = Constants.LiftState.MIN;
        Constants.lcState lcState = Constants.lcState.release;
        Constants.rcState rcState = Constants.rcState.release;

        Constants.PivotState pivotState = Constants.PivotState.stow;

        /* GAMEPAD2 BUTTONS */
        Button lcButton = new Button();
        Button rcButton = new Button();
        Button pivotButton = new Button();
        Button planeButton = new Button();
        Button pickupButton = new Button();

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

            if (Math.abs(gamepad1.left_stick_x) > 0.25)
                directionX = Math.pow(gamepad1.left_stick_x, 1);
            if (Math.abs(gamepad1.left_stick_y) > 0.25)
                directionY = -Math.pow(gamepad1.left_stick_y, 1);
            if (Math.abs(gamepad1.right_stick_x) > 0.25)
                directionR = Math.pow(gamepad1.right_stick_x, 1);

            double lfPower = (directionX + directionY + directionR) * drivePower;
            double lbPower = (-directionX + directionY + directionR) * drivePower;
            double rfPower = (-directionX + directionY - directionR) * drivePower;
            double rbPower = (directionX + directionY - directionR) * drivePower;

            robot.lf.setPower(lfPower);
            robot.lb.setPower(lbPower);
            robot.rf.setPower(rfPower);
            robot.rb.setPower(rbPower);
        }

        public void robotBaseDriveLoopAndCameraHone(double drivePower) {
            boolean targetFound = false;    // Set to true when an AprilTag target is detected
            double drive = 0;        // Desired forward power/speed (-1 to +1)
            double strafe = 0;        // Desired strafe power/speed (-1 to +1)
            double turn = 0;        // Desired turning power/speed (-1 to +1)

            AprilTagDetection desiredTag = null;

            telemetry.addData("IMU Angle", -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = robot.getAprilTagProcessor().getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    if (detection.id == Constants.DESIRED_ID_BLUE) {
                        desiredTag = detection;
                        targetFound = true;
                        break;
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found (Desired Tag)", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.addData("Yaw - IMU(R)", desiredTag.ftcPose.yaw + robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            } else {
                telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            }

            // If Left Trigger is being pressed hard enough, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.dpad_up && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - Constants.DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = -Range.clip(rangeError * Constants.SPEED_GAIN, -Constants.MAX_AUTO_SPEED, Constants.MAX_AUTO_SPEED); //set negative because our camera is in the back (so it's driving backwards)
                turn = -Range.clip(headingError * Constants.TURN_GAIN, -Constants.MAX_AUTO_TURN, Constants.MAX_AUTO_TURN); //keep turn positive (not sure why but this works)
                strafe = Range.clip(-yawError * Constants.STRAFE_GAIN, -Constants.MAX_AUTO_STRAFE, Constants.MAX_AUTO_STRAFE); //set negative (this works)


                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {
                if (Math.abs(gamepad1.left_stick_x) > 0.25)
                    strafe = Math.pow(gamepad1.left_stick_x, 1) * drivePower;
                if (Math.abs(gamepad1.left_stick_y) > 0.25)
                    drive = -Math.pow(gamepad1.left_stick_y, 1) * drivePower;
                if (Math.abs(gamepad1.right_stick_x) > 0.25)
                    turn = Math.pow(gamepad1.right_stick_x, 1) * drivePower;

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }

            double lfPower = (strafe + drive + turn);
            double rfPower = (-strafe + drive - turn);
            double lbPower = (-strafe + drive + turn);
            double rbPower = (strafe + drive - turn);

            robot.lf.setPower(lfPower);
            robot.lb.setPower(lbPower);
            robot.rf.setPower(rfPower);
            robot.rb.setPower(rbPower);
        }

        public double driveTrainSpeed() {
            double drivePower = Constants.DEFAULT_SPEED; //0.75
            if (gamepad1.right_bumper) drivePower = 1;
            else if (gamepad1.right_trigger >= 0.1) drivePower = 0.25;

            return drivePower;
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


        public void robotBasePixelLoop() {
            if (lcButton.is(Button.State.TAP)) {
                if (lcState == Constants.lcState.release) {
                    lcState = Constants.lcState.hold;
                    robot.lc.setPosition(Constants.LC_HOLD);
                } else if (lcState == Constants.lcState.hold) {
                    lcState = Constants.lcState.release;
                    robot.lc.setPosition(Constants.LC_RELEASE);
                }
            }

            if (rcButton.is(Button.State.TAP)) {
                if (rcState == Constants.rcState.release) {
                    rcState = Constants.rcState.hold;
                    robot.rc.setPosition(Constants.RC_HOLD);
                } else if (rcState == Constants.rcState.hold) {
                    rcState = Constants.rcState.release;
                    robot.rc.setPosition(Constants.RC_RELEASE);
                }
            }

            if (pivotButton.is(Button.State.TAP)) {
                if (pivotState == Constants.PivotState.stow && robot.lift.getCurrentPosition() > MIN_LIFT_CLICKS + 200) {
                    pivotState = Constants.PivotState.deposit;
                    robot.rightPivot.setPosition(Constants.PIVOT_DEPOSIT);
                    robot.leftPivot.setPosition(Constants.PIVOT_DEPOSIT);
                }
                if (pivotState == Constants.PivotState.deposit && robot.lift.getCurrentPosition() > MIN_LIFT_CLICKS) {
                    pivotState = Constants.PivotState.stow;
                    robot.rightPivot.setPosition(Constants.PIVOT_STOW);
                    robot.leftPivot.setPosition(Constants.PIVOT_STOW);
                }
                if (pivotState == Constants.PivotState.pickup && robot.lift.getCurrentPosition() > MIN_LIFT_CLICKS + 200) {
                    pivotState = Constants.PivotState.deposit;
                    robot.rightPivot.setPosition(Constants.PIVOT_DEPOSIT);
                    robot.leftPivot.setPosition(Constants.PIVOT_DEPOSIT);
                }
                if (pivotState == Constants.PivotState.pickup && robot.lift.getCurrentPosition() < MIN_LIFT_CLICKS + 200) {
                    pivotState = Constants.PivotState.stow;
                    robot.rightPivot.setPosition(Constants.PIVOT_STOW);
                    robot.leftPivot.setPosition(Constants.PIVOT_STOW);
                }
            }

            if (pickupButton.is(Button.State.TAP)) {
                if (pivotState == Constants.PivotState.deposit) {
                    // insert lift movement logic here
                    // if the pickup button is toggled then you MUST make the lift go down to the MIN_LIFT_CLICKS position
                    pivotState = Constants.PivotState.pickup;
                    robot.rightPivot.setPosition(Constants.PIVOT_PICKUP);
                    robot.leftPivot.setPosition(Constants.PIVOT_PICKUP);
                }
                if (pivotState == Constants.PivotState.stow) {
                    pivotState = Constants.PivotState.pickup;
                    robot.rightPivot.setPosition(Constants.PIVOT_PICKUP);
                    robot.leftPivot.setPosition(Constants.PIVOT_PICKUP);
                }
            }

            if (pivotState == Constants.PivotState.deposit && robot.lift.getCurrentPosition() < MIN_LIFT_CLICKS + 200) {
                pivotState = Constants.PivotState.stow;
                robot.rightPivot.setPosition(Constants.PIVOT_STOW);
                robot.leftPivot.setPosition(Constants.PIVOT_STOW);
            }

            if (lcState == Constants.lcState.release) robot.lc.setPosition(Constants.LC_RELEASE);
            if (lcState == Constants.lcState.hold) robot.lc.setPosition(Constants.LC_HOLD);

            if (rcState == Constants.rcState.release) robot.rc.setPosition(Constants.RC_RELEASE);
            if (rcState == Constants.rcState.hold) robot.rc.setPosition(Constants.RC_HOLD);

            if (pivotState == Constants.PivotState.deposit) {
                robot.rightPivot.setPosition(Constants.PIVOT_DEPOSIT);
                robot.leftPivot.setPosition(Constants.PIVOT_DEPOSIT);
            }

            if (pivotState == Constants.PivotState.stow) {
                robot.rightPivot.setPosition(Constants.PIVOT_STOW);
                robot.leftPivot.setPosition(Constants.PIVOT_STOW);
            }

            if (pivotState == Constants.PivotState.pickup) {
                // IMPORTANT: see line 591 or so
                // TLDR; implement lift logic here too!!
                robot.rightPivot.setPosition(Constants.PIVOT_PICKUP);
                robot.leftPivot.setPosition(Constants.PIVOT_PICKUP);
            }
        }

        private void setManualExposure(int exposureMS, int gain) throws InterruptedException {
            // Wait for the camera to be open, then use the controls

            if (robot.getVisionPortal() == null) {
                return;
            }

            // Make sure camera is streaming before we try to set the exposure controls
            if (robot.getVisionPortal().getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while ((robot.getVisionPortal().getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }

            // Set camera controls unless we are stopping.
//            if (!isStopRequested())
//            {
            ExposureControl exposureControl = robot.getVisionPortal().getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = robot.getVisionPortal().getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
//            }
        }


        public void planeLaunch() {
            if (planeButton.is(Button.State.DOUBLE_TAP)) {
                robot.plane.setPosition(Constants.RELEASE_PLANE);
            }
        }

        protected void UpdateTelemetry() {
            telemetry.addData("X", gamepad1.left_stick_x);
            telemetry.addData("Y", -gamepad1.left_stick_y);
            telemetry.addData("R", gamepad1.right_stick_x);

            telemetry.addData("DPAD Up", gamepad1.dpad_up);
            telemetry.addData("DPAD Down", gamepad1.dpad_down);
            telemetry.addData("DPAD Left", gamepad1.dpad_left);
            telemetry.addData("DPAD Right", gamepad1.dpad_right);

            telemetry.addData("Top Left Power", robot.lf.getPower());
            telemetry.addData("Bottom Left Power", robot.lb.getPower());
            telemetry.addData("Top Right Power", robot.rf.getPower());
            telemetry.addData("Bottom Right Power", robot.rb.getPower());

            telemetry.addData("Right trigger", gamepad1.right_trigger);
            telemetry.addData("Left trigger", gamepad1.left_trigger);

            telemetry.addData("Top Left Encoder Position", robot.lf.getCurrentPosition());
            telemetry.addData("Top Right Encoder Position", robot.rf.getCurrentPosition());
            telemetry.addData("Bottom Left Encoder Position", robot.lb.getCurrentPosition());
            telemetry.addData("Bottom Right Encoder Position", robot.rb.getCurrentPosition());

            telemetry.addData("Outtake Motor Position", robot.lift.getCurrentPosition());
            telemetry.addData("Plane Servo Position", robot.plane.getPosition());
//            telemetry.addData("Left Claw Servo Position", robot.lcButton.getPosition());
//            telemetry.addData("Left Claw Servo State", lcState);

            telemetry.addData("Right Claw Servo Position", robot.rc.getPosition());
//            telemetry.addData("Right Claw Servo State", rcState);

//            telemetry.addData("Claw Rot Servo State", pivotState);


            telemetry.addData("Top Left Velocity", robot.lf.getVelocity());
            telemetry.addData("Top Left Acceleration", robot.lf.getVelocity() / runtime.seconds()); //only works if holding down max power at the beginning of the opmode.

            telemetry.addData("Yaw", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("-Yaw", -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

//        packet.put("X", gamepad1.left_stick_x);
//        packet.put("Y",  -gamepad1.left_stick_y);
//        packet.put("R", gamepad1.right_stick_x);

            packet.put("Top Left Power", robot.lf.getPower());
            packet.put("Bottom Left Power", robot.lb.getPower());
            packet.put("Top Right Power", robot.rf.getPower());
            packet.put("Bottom Right Power", robot.rb.getPower());

            packet.put("Top Left Current", robot.lf.getCurrent(CurrentUnit.AMPS));
            packet.put("Bottom Left Current", robot.lb.getCurrent(CurrentUnit.AMPS));
            packet.put("Top Right Current", robot.rf.getCurrent(CurrentUnit.AMPS));
            packet.put("Bottom Right Current", robot.rb.getCurrent(CurrentUnit.AMPS));

            packet.put("Top Left Velocity", robot.lf.getVelocity());
            packet.put("Top Right Velocity", robot.rf.getVelocity());
            packet.put("Bottom Left Velocity", robot.lb.getVelocity());
            packet.put("Bottom Right Velocity", robot.rb.getVelocity());


            dashboard.sendTelemetryPacket(packet);

            //  telemetry.addData("Touch Sensor", robot.digitalTouch.getState());
            telemetry.update();
        }

        public void UpdateButton() {
            lcButton.update(gamepad2.left_bumper);
            rcButton.update(gamepad2.right_bumper);
            pickupButton.update(gamepad2.a);
            pivotButton.update(gamepad2.b);
            planeButton.update(gamepad2.y);
        }
    }
}