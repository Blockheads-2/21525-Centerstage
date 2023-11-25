package org.firstinspires.ftc.teamcode.auto.routes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.cv.TeamElementDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Methods;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;

@Autonomous(name="Detect Element", group="Autonomous")
public class DetectElement extends Methods.auto {
    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

//        streamOpenCV();
        initVisionPortal();

        dispatch.robot.getVisionPortal().setProcessorEnabled(dispatch.robot.getAprilTagProcessor(), false);

        while (!opModeIsActive()){
            updateValueDecrease.update(gamepad1.a);
            updateValueIncrease.update(gamepad1.b);

            if (updateValueIncrease.is(Button.State.TAP) && gamepad1.right_bumper==true) {
                detector.changeUpperHue(1);
            } else if (updateValueIncrease.is(Button.State.TAP)){
                detector.changeUpperHue(-1);
            }

            if (updateValueDecrease.is(Button.State.TAP) && gamepad1.right_bumper==true) {
                detector.changeLowerHue(1);
            } else if (updateValueDecrease.is(Button.State.TAP)){
                detector.changeLowerHue(-1);
            }
        }

        waitForStart();

//        phoneCam.stopStreaming();
//        dispatch.robot.initTelemetry(telemetry);
//        initVisionPortal();
        dispatch.robot.getVisionPortal().setProcessorEnabled(dispatch.robot.getAprilTagProcessor(), true);
        dispatch.robot.getVisionPortal().setProcessorEnabled(dispatch.robot.getOpenCVProcessor(), false);

        while (opModeIsActive())
        {
            streamAprilTag();

            updateTelemetry();
            sleep(20);
        }

        phoneCam.stopStreaming();
    }
}
