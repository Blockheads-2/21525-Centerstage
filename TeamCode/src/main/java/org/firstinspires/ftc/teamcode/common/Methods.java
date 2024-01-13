package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.common.Constants.CLAW_ROT_DOWN;
import static org.firstinspires.ftc.teamcode.common.Constants.HIGH_OUTTAKE;
import static org.firstinspires.ftc.teamcode.common.Constants.LOW_OUTTAKE;
import static org.firstinspires.ftc.teamcode.common.Constants.MAX_OUTTAKE_CLICKS;
import static org.firstinspires.ftc.teamcode.common.Constants.MID_OUTTAKE;
import static org.firstinspires.ftc.teamcode.common.Constants.MIN_OUTTAKE_CLICKS;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
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

        public interface TelemetryFunc{
            void update();
        }

        public void initRobot(){
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

        public void initVisionPortal(){
            dispatch.initCamera(telemetry);
        }

        public void streamOpenCV(boolean blue){
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("Webcam 1","id", hardwareMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            detector = new TeamElementDetectionPipeline(telemetry);
            detector.blueOrRed(blue);
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

        public void stopOpenCV(){
            phoneCam.stopStreaming();
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
        public AprilTagDetection streamAprilTag(int desiredTagId){
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
//            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            java.util.List<AprilTagDetection> myAprilTagDetections;  // list of all detections

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
                        if (detection.metadata != null && detection.id == desiredTagId) {  // This check for non-null Metadata is not needed for reading only ID code.

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
                            return detection;
                        }

                    }
                }
            }
            return null;
        }


        public void streamTfod(){
            java.util.List<Recognition> currentRecognitions = dispatch.robot.getTfodProcessor().getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                telemetry.addData(""," ");
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

        public void spline(double speed, double x, double y, double timeoutS){
            dispatch.variableHeading(speed, x, y, timeoutS);
        }

        public void spline(double speed, double x, double y){
            dispatch.variableHeading(speed, x, y);
        }

        public boolean AprilTagMove(AprilTagDetection tag){
            return dispatch.AprilTagMove(tag);
        }

        public void turnAbsPID(double theta){ //turning relative to its initial point
            dispatch.turnAbsPID(theta, 6);
        }

        public void runIntake(double power, double timeout){
//            dispatch.spinIntake(power, timeout);
        }

        public void runOuttake(int clickTarget, double power){
            while (opModeIsActive() && Math.abs(dispatch.robot.outtake.getCurrentPosition() - clickTarget) >= 15){
                dispatch.robot.outtake.setTargetPosition(clickTarget);
                dispatch.robot.outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                dispatch.robot.outtake.setPower(power);
            }
            dispatch.robot.outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void UpdateTelemetry(){
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
        protected SpinPID outtakePositioner;
        protected TelemetryPacket packet;
        protected enum IntakeState {
            LEFT_OPEN,
            LEFT_CLOSED,
            RIGHT_OPEN,
            RIGHT_CLOSED,
            CLAW_ROT_AMBIGUOUS,
            CLAW_ROT_UP,
            CLAW_ROT_DOWN
        }
        protected IntakeState left_claw_state = IntakeState.LEFT_CLOSED;
        protected IntakeState right_claw_state = IntakeState.RIGHT_CLOSED;

        protected IntakeState rot_claw_state = IntakeState.CLAW_ROT_UP;

//        Button bottomOuttake = new Button();
//        Button midOuttake = new Button();
//        Button highOuttake = new Button();
        Button left_claw = new Button();
        Button right_claw = new Button();

        Button rot_claw = new Button();
        Button planeButton = new Button();

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

//            robot.outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void PlayerOne(){
            robotBaseDriveLoop(driveTrainSpeed());
//            robotBaseMicroAdjustLoop(driveTrainSpeed());
        }

        public void PlayerTwo(){
//            robotBaseOuttakeLoop();
            robotBaseIntakeLoop();
            planeLaunch();
        }

        public void robotBaseDriveLoop(double drivePower) {
            double directionX = 0;
            double directionY = 0;
            double directionR = 0;
            int i1 = 0;

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

        public double driveTrainSpeed() {
            double drivePower = Constants.DEFAULT_SPEED; //0.75
            if (gamepad1.right_bumper) drivePower = 1;
            else if (gamepad1.left_bumper) drivePower = 0.25;

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

        public void robotBaseOuttakeLoop() {
//            int clickTarget = robot.outtake.getCurrentPosition();
            int clickTarget = Range.clip(robot.outtake.getCurrentPosition() - (int)(gamepad2.left_stick_y * 300), MIN_OUTTAKE_CLICKS, MAX_OUTTAKE_CLICKS);

//            if (gamepad1.dpad_down){
//                clickTarget = Range.clip(robot.outtake.getCurrentPosition() - 300, MIN_OUTTAKE_CLICKS, MAX_OUTTAKE_CLICKS);
//            } else if (gamepad1.dpad_up){
//                clickTarget = Range.clip(robot.outtake.getCurrentPosition() + 300, MIN_OUTTAKE_CLICKS, MAX_OUTTAKE_CLICKS);
//            }

//            int clickTarget = Range.clip(robot.outtake.getCurrentPosition() - (int)(gamepad2.left_stick_y * 300), MIN_OUTTAKE_CLICKS, MAX_OUTTAKE_CLICKS);
//            if (bottomOuttake.is(Button.State.HELD)) clickTarget = LOW_OUTTAKE;
//            else if (midOuttake.is(Button.State.HELD)) clickTarget = MID_OUTTAKE;
//            else if (highOuttake.is(Button.State.HELD)) clickTarget = HIGH_OUTTAKE;

            robot.outtake.setTargetPosition(clickTarget);
            robot.outtake.setPower(0.8);
        }

        public void robotBaseIntakeLoop() {

            if (gamepad1.right_trigger >= 0.1){
                robot.claw_rot.setPosition(Range.clip(robot.claw_rot.getPosition() - (0.1 * gamepad1.right_trigger), Constants.CLAW_ROT_UP, Constants.CLAW_ROT_DOWN));
                rot_claw_state = IntakeState.CLAW_ROT_AMBIGUOUS;
            } else if (gamepad1.left_trigger >= 0.1){
                robot.claw_rot.setPosition(Range.clip(robot.claw_rot.getPosition() + (0.1 * gamepad1.left_trigger), Constants.CLAW_ROT_UP, Constants.CLAW_ROT_DOWN));
                rot_claw_state = IntakeState.CLAW_ROT_AMBIGUOUS;
            }

            if (left_claw.is(Button.State.TAP)){
//                if (left_claw_state == IntakeState.LEFT_CLOSED) {
//                    robot.left_claw.setPosition(Constants.LEFT_CLAW_RELEASE);
//                    left_claw_state = IntakeState.LEFT_OPEN;
//                }
//                else if (left_claw_state == IntakeState.LEFT_OPEN){
//                    robot.left_claw.setPosition(Constants.LEFT_CLAW_HOLD);
//                    left_claw_state = IntakeState.LEFT_CLOSED;
//                }
            }
            if (right_claw.is(Button.State.TAP)){
                if (right_claw_state == IntakeState.RIGHT_CLOSED && rot_claw_state != IntakeState.CLAW_ROT_UP) { //only open claw if our claw is down/ambiguous. Don't want to open our claw when
                    robot.right_claw.setPosition(Constants.RIGHT_CLAW_RELEASE);
                    right_claw_state = IntakeState.RIGHT_OPEN;
                }
                else if (right_claw_state == IntakeState.RIGHT_OPEN){
                    robot.right_claw.setPosition(Constants.RIGHT_CLAW_HOLD);
                    right_claw_state = IntakeState.RIGHT_CLOSED;
                }
            }
            if (rot_claw.is(Button.State.TAP)){
                if (rot_claw_state == IntakeState.CLAW_ROT_AMBIGUOUS){
                    if (right_claw_state == IntakeState.RIGHT_OPEN){ //we don't want to pull in our claw if the claw is open.
                        robot.right_claw.setPosition(Constants.RIGHT_CLAW_HOLD);
                        right_claw_state = IntakeState.RIGHT_CLOSED;
                    }
                    robot.claw_rot.setPosition(Constants.CLAW_ROT_UP);
                    rot_claw_state = IntakeState.CLAW_ROT_UP;
                }
                else if (rot_claw_state == IntakeState.CLAW_ROT_UP) {
                    robot.claw_rot.setPosition(Constants.CLAW_ROT_DOWN);
                    rot_claw_state = IntakeState.CLAW_ROT_DOWN;
                }
                else if (rot_claw_state == IntakeState.CLAW_ROT_DOWN){
                    if (right_claw_state == IntakeState.RIGHT_OPEN){ //we don't want to pull in our claw if the claw is open.
                        robot.right_claw.setPosition(Constants.RIGHT_CLAW_HOLD);
                        right_claw_state = IntakeState.RIGHT_CLOSED;
                    }
                    robot.claw_rot.setPosition(Constants.CLAW_ROT_UP);
                    rot_claw_state = IntakeState.CLAW_ROT_UP;
                }
            }
        }


        public void planeLaunch(){
            if (planeButton.is(Button.State.DOUBLE_TAP)){
                robot.plane.setPosition(Constants.RELEASE_PLANE);
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

            telemetry.addData("Outtake Motor Position", robot.outtake.getCurrentPosition());
            telemetry.addData("Plane Servo Position", robot.plane.getPosition());
//            telemetry.addData("Left Claw Servo Position", robot.left_claw.getPosition());
//            telemetry.addData("Left Claw Servo State", left_claw_state);

            telemetry.addData("Right Claw Servo Position", robot.right_claw.getPosition());
            telemetry.addData("Right Claw Servo State", right_claw_state);

            telemetry.addData("Claw Rot Servo Position", robot.claw_rot.getPosition());
            telemetry.addData("Claw Rot Servo State", rot_claw_state);


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

        public void UpdateButton(){
//            bottomOuttake.update(gamepad2.a);
//            midOuttake.update(gamepad2.x);
//            highOuttake.update(gamepad2.y);
            left_claw.update(gamepad2.a);
            right_claw.update(gamepad1.y);
            rot_claw.update(gamepad1.x);
            planeButton.update(gamepad1.b);
        }
    }
}