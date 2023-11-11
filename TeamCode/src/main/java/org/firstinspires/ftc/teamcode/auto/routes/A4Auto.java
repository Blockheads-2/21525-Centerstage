package org.firstinspires.ftc.teamcode.auto.routes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Methods;

@Autonomous(name="A4 Blue Autonomous", group="Autonomous")
public class A4Auto extends Methods.auto {

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        streamOpenCV();

        waitForStart();

        if (opModeIsActive()){
            switch(detector.getLocation()){
                case LEFT:
                    constantHeading(0.25, 0, 12, 0.03, 0, 0.01);
                    spinIntake(-0.7, 4);
                    break;

                case MID:
                    constantHeading(0.25, 0, 12, 0.03, 0, 0.01);
                    spinIntake(-0.7, 4);
                    break;

                case RIGHT:
                    constantHeading(0.25, 0, 12, 0.03, 0, 0.01);
                    spinIntake(-0.7, 4);
                    break;
            }
        }
        absoluteTurn(-90);
        constantHeading(0.25, 12, 0, 0.03, 0, 0.01);
        spinIntake(-0.7, 4);
    }

}



