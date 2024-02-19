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

@Autonomous(name="A2 (Blue Far)", group="Autonomous")
public class A2Auto extends Methods.auto{
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        streamOpenCV(true);

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
                constantHeading(0.4, -5, -29,0,0,0);
                //Place
//                spline(0.3, 10, -30, 15); //so this just goes straight...

                turnAbsPID(90);

                //Move Out of the way
                constantHeading(0.2, 0, -10, 0, 0, 0);

//                //Move to middle
//                constantHeading(0.2, 30, 0, 0, 0, 0);
//
//                //Move to park
//                constantHeading(0.3, 5, 98, 0, 0, 0);

//                //Place
//                constantHeading(0.2, 0, -27, 0, 0, 0);
//
//                constantHeading(0.2, -17, 0, 0, 0, 0);
//
//                //Move Out of the way
//                constantHeading(0.2, 24, 6, 0, 0, 0);
//
//                //Move to middle
//                constantHeading(0.2, 0, -34, 0, 0, 0);
//
//                //turn
//                turnAbsPID(-90);
//
//                //Move to park
//                constantHeading(0.3, 0, 87, 0, 0, 0);
//
//                turnAbsPID(90);
//
//                //Move to Place on board
//                constantHeading(0.2, -26, 0, 0, 0, 0);
////
//                runOuttake(Constants.MID_OUTTAKE, 0.6);
//
//                //open claw
//
//                runOuttake(0, 0.5);

                break;

            case RIGHT:
                //Place
                constantHeading(0.2, -21, -31, 0, 0, 0);

                //Move Out of the way
                constantHeading(0.15, 14, 5, 0, 0, 0);

//                //Move to middle
//                constantHeading(0.2, 0, -35, 0, 0, 0);

//                //turn
//                turnAbsPID(90);
//
//                //Move to park
//                constantHeading(0.3, 3, 92, 0, 0, 0);

//                turnAbsPID(90);
//
////                Move to Place on board
//                constantHeading(0.2, -15, 0, 0, 0, 0); //y-value may change based on the length of the claw.
//
//                runOuttake(Constants.MID_OUTTAKE, 0.6);
//
//                //open claw
//
//                runOuttake(0, 0.5);
                break;

            case MID:
                //Place
                constantHeading(0.2, -4, -35, 0, 0, 0);

                //Move Out of the way
                constantHeading(0.15, -20, 5, 0, 0, 0);

//                //Move to middle
//                constantHeading(0.2, 0, -28, 0, 0, 0);
//
//                //turn
//                turnAbsPID(90);
//
//                //Move to park
//                constantHeading(0.3, 0, 105, 0, 0, 0);

//                //Turn 180 degrees
//                turnAbsPID(90);
//
//                //Move to Place on board
//                constantHeading(0.2, -24, 0, 0, 0, 0); //y-value will changed based on the length of the claw
//
//                runOuttake(Constants.MID_OUTTAKE, 0.6);
//
//                //open claw
//
//                runOuttake(0, 0.5);

                break;
        }
    }
}
