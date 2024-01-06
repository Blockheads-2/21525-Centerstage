package org.firstinspires.ftc.teamcode.auto.routes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.auto.cv.TeamElementDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.Methods;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="F2 (Red Far; No board-drop-off)", group="Autonomous")
public class F2AutoAvoid extends Methods.auto{
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        streamOpenCV(false);

        TeamElementDetectionPipeline.Location elementLocation;

        while (!opModeIsActive()){
            elementLocation = detector.getLocation();
        }

        elementLocation = detector.getLocation();

        stopOpenCV();

        waitForStart();

        //1 TILE = 24 INCHES

        switch (elementLocation) {
            //Park Left
            case LEFT:
                //Place
                constantHeading(0.2, 19, -34, 0, 0, 0);

                //Move Out of the way
                constantHeading(0.2, -22, 9, 0, 0, 0);

                //Move to middle
                constantHeading(0.2, 0, -41, 0, 0, 0);

                //turn
                turnAbsPID(-90);

                //Move to park
                constantHeading(0.3, 0, 83, 0, 0, 0);
                break;

            case RIGHT:
                //Place
                constantHeading(0.2, 0, -27, 0, 0, 0);

                constantHeading(0.2, -17, 0, 0, 0, 0);

                //Move Out of the way
                constantHeading(0.2, 24, 5, 0, 0, 0);

                //Move to middle
                constantHeading(0.2, 0, -34, 0, 0, 0);

                //turn
                turnAbsPID(-90);

                //Move to park
                constantHeading(0.3, 0, 88, 0, 0, 0);

                break;

            case MID:
                //Place
                constantHeading(0.2, 0, -31, 0, 0, 0);

                //Move Out of the way
                constantHeading(0.2, 20, 7, 0, 0, 0);

                //Move to middle
                constantHeading(0.2, 0, -33, 0, 0, 0);

                //turn
                turnAbsPID(-90);

                //Move to park
                constantHeading(0.3, 0, 100, 0, 0, 0);

                break;
        }
    }
}
