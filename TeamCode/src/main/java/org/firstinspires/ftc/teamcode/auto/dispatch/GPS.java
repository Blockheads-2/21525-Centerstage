package org.firstinspires.ftc.teamcode.auto.dispatch;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.sun.tools.javac.util.List;

public class GPS {
    double pos[] = new double[3]; //pos[0] = x, pos[1] = y, pos[2] = theta (heading)
    int clickPos[] = new int[4];
    HardwareDrive robot;

    MecanumDriveOdometry m_odo;
    Pose2d position;

    public GPS(HardwareDrive robot){
        this.robot = robot;

        Translation2d frontLeftWheelMeters = new Translation2d(0, 0); //input correct measurements here
        Translation2d frontRightWheelMeters = new Translation2d(0, 0);
        Translation2d rearLeftWheelMeters = new Translation2d(0, 0);
        Translation2d rearRightWheelMeters = new Translation2d(0, 0);

        MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(frontLeftWheelMeters, frontRightWheelMeters, rearLeftWheelMeters, rearRightWheelMeters);
        m_odo = new MecanumDriveOdometry(m_kinematics, new Rotation2d(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)), new Pose2d(0, 0, new Rotation2d()));

        clickPos[0] = robot.lf.getCurrentPosition();
        clickPos[1] = robot.rf.getCurrentPosition();
        clickPos[2] = robot.lb.getCurrentPosition();
        clickPos[3] = robot.rb.getCurrentPosition();
    }

    public void periodic(double elapsedTime){
        // Get my wheel speeds; assume .getRate() has been
        // set up to return velocity of the encoder
        // in meters per second.
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds
                (
                        robot.lf_motor.encoder.getRate(), robot.rf_motor.encoder.getRate(),
                        robot.lb_motor.encoder.getRate(), robot.rb_motor.encoder.getRate()
                );
        // Get my gyro angle.
        Rotation2d gyroAngle = new Rotation2d(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Update the pose
        position = m_odo.updateWithTime(elapsedTime, gyroAngle, wheelSpeeds);
    }

    public void update(){
        int i = 0;
        for (int click : List.of(robot.lf.getCurrentPosition(), robot.rf.getCurrentPosition(), robot.lb.getCurrentPosition(), robot.rb.getCurrentPosition())){ //not resourcefully efficient to create a new List every loop.
            int delta = click - clickPos[i];
            pos[2] = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            clickPos[i] = click;
            i++;
        }
        i=0;


        clickPos[0] = robot.lf.getCurrentPosition();
        clickPos[1] = robot.rf.getCurrentPosition();
        clickPos[2] = robot.lb.getCurrentPosition();
        clickPos[3] = robot.rb.getCurrentPosition();
    }

    public Pose2d getPose(){
        return position;
    }

    public double[] getPos(){
        return pos;
    }
}
