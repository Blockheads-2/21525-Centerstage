/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.common;

import android.util.Size;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.cv.OpenCvProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 */
public class HardwareDrive {

    private final ElapsedTime period = new ElapsedTime();
    public DcMotorEx lf;
    public DcMotorEx rf;
    public DcMotorEx rb;
    public DcMotorEx lb;
    public DcMotorEx intake;
    public DcMotorEx outtake;

    public Motor lf_motor;
    public Motor rf_motor;
    public Motor rb_motor;
    public Motor lb_motor;
    public IMU imu;
    public MecanumDrive m_drive;
    HardwareMap hwMap = null;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private OpenCvProcessor openCV;                 // Used for managing the OpenCV detection process.
    private TfodProcessor tfod;
    private final AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private Telemetry telemetry = null;

    public HardwareDrive() {

    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;
        lf = hwMap.get(DcMotorEx.class, "left_front");
        lb = hwMap.get(DcMotorEx.class, "left_back");
        rf = hwMap.get(DcMotorEx.class, "right_front");
        rb = hwMap.get(DcMotorEx.class, "right_back");
//        intake = hwMap.get(DcMotorEx.class, "intake");
//        outtake = hwMap.get(DcMotorEx.class, "outtake");
        imu = hwMap.get(IMU.class, "imu");

        lf_motor = new Motor(ahwMap, "left_front", Constants.CPR, Constants.RPM); //playing around with ftclib
        lb_motor = new Motor(ahwMap, "left_back", Constants.CPR, Constants.RPM); //playing around with ftclib
        rf_motor = new Motor(ahwMap, "right_front", Constants.CPR, Constants.RPM);
        rb_motor = new Motor(ahwMap, "right_back", Constants.CPR, Constants.RPM);
        m_drive = new MecanumDrive(lf_motor, rf_motor, lb_motor, rb_motor);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
//        intake.setDirection(DcMotorSimple.Direction.REVERSE);
//        outtake.setDirection(DcMotorSimple.Direction.FORWARD);

        resetEncoderPos();
        imu.resetYaw();
    }

    public void resetEncoderPos() {
        for (DcMotorEx motor : List.of(lf, lb, rf, rb)) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    public void initCamera(Telemetry t) {
        initTelemetry(t);

        if (hwMap != null && hwMap.get(WebcamName.class, "Webcam 1") != null) {
            aprilTag = new AprilTagProcessor.Builder() // Create a new AprilTag Processor Builder object.
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()) //sets the AprilTagLibrary to the current season. You can add your own custom AprilTags as well (refer to AprilTag Library under the FIRST FTC Docs)
                .setLensIntrinsics(Constants.fx, Constants.fy, Constants.cx, Constants.cy) //Since C270 1280x720 webcam is not among the resolutions the SDK knows of, we need to provide it w/ the calibration info.
                .setDrawTagID(true) // Default: true, for all detections.
                .setDrawTagOutline(true) // Default: true, when tag size was provided (thus eligible for pose estimation).
                .setDrawAxes(true) // Default: false.
                .setDrawCubeProjection(true) // Default: false.
                .build(); // Create an AprilTagProcessor by calling build()
//
//            openCV = new OpenCvProcessor.Builder()
//                    .build();
//            openCV.setTelemetry(telemetry);

            tfod = new TfodProcessor.Builder() // Create a new TFOD Processor Builder object.
                    .setMaxNumRecognitions(10) // Max. number of recognitions the network will return
                    .setUseObjectTracker(true) // Whether to use the object tracker
                    .setTrackerMaxOverlap((float) 0.2) // Max. % of box overlapped by another box at recognition time
                    .setTrackerMinSize(16) // Min. size of object that the object tracker will track
                    .build(); // Create a TFOD Processor by calling build()

            visionPortal = new VisionPortal.Builder() // Create a new VisionPortal Builder object.
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1")) // Specify the camera to be used for this VisionPortal.
//                .addProcessors(aprilTag, openCV, tfod) // Add the AprilTag Processor to the VisionPortal Builder.
                .addProcessors(aprilTag, tfod) // Add the AprilTag Processor to the VisionPortal Builder.
//                .setCameraResolution(new Size(640, 480)) // Each resolution, for each camera model, needs calibration values for good pose estimation.
                .setCameraResolution(new Size(1280, 720)) // Each resolution, for each camera model, needs calibration values for good pose estimation.
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // MJPEG format uses less bandwidth than the default YUY2.
                .enableLiveView(true) // Enable LiveView (RC preview).  I believe we don't need this because we use the Driver Hub Camera Stream, not an RC phone.
                .setAutoStopLiveView(true) // Automatically stop LiveView (RC preview) when all vision processors are disabled.
                .build(); // Create a VisionPortal by calling build().  The camera starts streaming.

//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                Methods.general.trySleep(20);
//
//            }
//            exposureControl.setExposure((long) Constants.EXPOSURE_MS, TimeUnit.MILLISECONDS);
//            Methods.general.trySleep(20);
//
//            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//            gainControl.setGain(Constants.CAMERA_GAIN);
//            Methods.general.trySleep(20);

            telemetry.addData("Camera ready", visionPortal.getCameraState());
            telemetry.update();
        }
    }

    public void initTelemetry(Telemetry t){
        telemetry = t;
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTag;
    }

    public OpenCvProcessor getOpenCVProcessor() {return openCV;}

    public TfodProcessor getTfodProcessor(){
        return tfod;
    }

    public void pauseStream() {
        visionPortal.stopLiveView(); //temporarily sops the live view (RC preview). This should be fine as we don't use an RC phone.
    }

    public void resumeStream() {
        visionPortal.resumeLiveView();
    }
}

