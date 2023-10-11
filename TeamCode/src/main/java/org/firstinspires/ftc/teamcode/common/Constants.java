package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Constants {
    //Drive Train Constants
    public static float fx = 622.001f;
    public static float fy = 622.001f;
    public static float cx = 319.803f;
    public static float cy = 241.251f;

    public static float LOAD_ON = 0.6f; //assumption
    public static double RPM = 312 * LOAD_ON; //690.  Not very accurate so don't rely on this number.
    public static double RPS = RPM / 60.0; //11.5 ish motor revolutions per second, with load

    public static float WHEEL_D = 0f; //wheel diameter
    public static double WHEEL_C = WHEEL_D * Math.PI; //wheel circumference (inches)

    public static float CPR = 537.7f; //537.7 clicks per revolution
    public static double CPI = CPR / WHEEL_C; //... clicks per inch.

    public static float MAX_VELOCITY_DT = 2700f; // unit is clicks/sec; not sure if this is accurate...

    //Distance Between swerve module and Center
    public static float DISTANCE_BETWEEN_MODULE_AND_CENTER = 3.406f; //3.405512

    public static double DEFAULT_SPEED = 0.75;

    //Camera Constants
    public static float WIDTH = 1280.0f;
    public static float HEIGHT = 720.0f;
    public static double CAMERA_ERROR = 2.0f; //typically off by ~2 inches in the y-direction.  Not sure about the x-direction.

    final double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)

    // apriltag stuff
    public static final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public static final double EXPOSURE_MS = 6;
    public static final int CAMERA_GAIN = 250;
}