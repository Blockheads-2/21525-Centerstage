package org.firstinspires.ftc.teamcode.auto.routes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Methods;

@Autonomous(name="F4 Red Autonomous", group="Routes")
public class F4Auto extends Methods.auto {
    HardwareDrive robot = new HardwareDrive();
    HardwareMap hwMap;
    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hwMap);

        while (!opModeIsActive());
        waitForStart();
            robotAutoStraightDrivePosition(0.5, 0, 29, robot);
            //The only turn we need goes here. It is 270 degrees clockwise.
            robotAutoStraightDrivePosition(0.5, 0, 32, robot);
            robotAutoStraightDrivePosition(0.5, 0, -128, robot);
            robotAutoStraightDrivePosition(0.5, 0, 128, robot);
            robotAutoStraightDrivePosition(0.5, 0, -128, robot);
            robotAutoStraightDrivePosition(0.5, 0, 128, robot);

            telemetry.update();
    }
}
