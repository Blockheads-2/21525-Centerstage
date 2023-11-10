package org.firstinspires.ftc.teamcode.auto.routes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.dispatch.AutoHub;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;

import java.util.ArrayList;
import java.util.List;

public class FTCLibAutoTest extends LinearOpMode {

    AutoHub dispatch;
    Constants constants = new Constants();
    Button updateValueDecrease = new Button();
    Button updateValueIncrease = new Button();
    MecanumDriveOdometry m_odo;
    Pose2d position;

    @Override
    public void runOpMode() throws InterruptedException {
        dispatch = new AutoHub(this);

        dispatch.initCamera();

        Translation2d frontLeftWheelMeters = new Translation2d(0, 0);
        Translation2d frontRightWheelMeters = new Translation2d(0, 0);
        Translation2d rearLeftWheelMeters = new Translation2d(0, 0);
        Translation2d rearRightWheelMeters = new Translation2d(0, 0);


        MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(frontLeftWheelMeters, frontRightWheelMeters, rearLeftWheelMeters, rearRightWheelMeters);
        m_odo = new MecanumDriveOdometry(m_kinematics, new Rotation2d(dispatch.robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)), new Pose2d(0, 0, new Rotation2d()));

        while (!opModeIsActive()) { //checks if play hasn't been pressed (in init stage)

        }

        waitForStart();

        if (opModeIsActive()){
            generateTrajectory();
        }

        //all movement goes here
        telemetry.update();
    }

    public void periodic(){
        // Get my wheel speeds; assume .getRate() has been
        // set up to return velocity of the encoder
        // in meters per second.
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds
                (
                        dispatch.robot.lf_motor.encoder.getRate(), dispatch.robot.rf_motor.encoder.getRate(),
                        dispatch.robot.lb_motor.encoder.getRate(), dispatch.robot.rb_motor.encoder.getRate()
                );
        // Get my gyro angle.
        Rotation2d gyroAngle = new Rotation2d(dispatch.robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Update the pose
        position = m_odo.updateWithTime(dispatch.runtime.seconds(), gyroAngle, wheelSpeeds);
    }

    public void generateTrajectory() {

        // 2018 cross scale auto waypoints.
        Pose2d sideStart = new Pose2d(0,0,
                Rotation2d.fromDegrees(-180));
        Pose2d crossScale = new Pose2d(0,0,
                Rotation2d.fromDegrees(-160));

        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(0,0));
        interiorWaypoints.add(new Translation2d(0,0));

        TrajectoryConfig config = new TrajectoryConfig(Constants.MAX_VELOCITY_DT_MS,12);
//        config.setReversed(true);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config);
    }
}
