package org.firstinspires.ftc.teamcode.common;

public class Constants {
    // apriltag stuff
    public static final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    //Which one?
    public static double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    final double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)

    public static final double EXPOSURE_MS = 6;
    public static final int CAMERA_GAIN = 250;
    //Drive Train Constants
    public static float fx = 622.001f;
    public static float fy = 622.001f;
    public static float cx = 319.803f;
    public static float cy = 241.251f;
    public static float LOAD_ON = 0.6f; //assumption
    public static double RPM = 312 * LOAD_ON; //690.  Not very accurate so don't rely on this number.
    public static double RPS = RPM / 60.0; //11.5 ish motor revolutions per second, with load
    public static double WHEEL_D = 96 / 25.4; //wheel diameter
    public static double WHEEL_C = WHEEL_D * Math.PI; //wheel circumference (inches)
    public static float CPR = 537.7f; //537.7 clicks per revolutionX
    public static double CPI = CPR / WHEEL_C; //... clicks per inch.
    public static float MAX_VELOCITY_DT = 2600f; // unit is clicks/sec; not sure if this is accurate...
    public static double MAX_VELOCITY_DT_MS = (MAX_VELOCITY_DT / CPI) * 0.0254; // unit is meters/sec
    public static float HORIZONTAL_DISTANCE_BETWEEN_WHEEL_AND_CENTER;
    //Distance Between swerve module and Center
    public static float DISTANCE_BETWEEN_MODULE_AND_CENTER = 3.406f; //3.405512
    public static double DEFAULT_SPEED = 1;
    // Camera Constants
    public static float WIDTH = 1280.0f;
    public static float HEIGHT = 720.0f;
    public static double CAMERA_ERROR = 2.0f; //typically off by ~2 inches in the y-direction.  Not sure about the x-direction.
    public static double HSV_HUE_LOW_BLUE = 70;
    public static double HSV_HUE_HIGH_BLUE = 120;
    public static double HSV_SATURATION_LOW_BLUE = 75;
    public static double HSV_SATURATION_HIGH_BLUE = 255;
    public static double HSV_VALUE_LOW_BLUE = 0;
    public static double HSV_VALUE_HIGH_BLUE = 255;

    public static double HSV_HUE_LOW_RED = 0;
    public static double HSV_HUE_HIGH_RED = 70;
    public static double HSV_SATURATION_LOW_RED = 90;
    public static double HSV_SATURATION_HIGH_RED = 255;
    public static double HSV_VALUE_LOW_RED = 0;
    public static double HSV_VALUE_HIGH_RED = 255;
    public static double PIXEL_WIDTH_MM = 84.25;
    public static double PIXEL_WIDTH_TO_DISTANCE_FROM_CAMERA = 640.0 / (Math.tan(Math.toRadians(68.0)) * (PIXEL_WIDTH_MM / 2.0)); //pixels to mm; assumes resolution is 640x480
//<<<<<<< HEAD


    final double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)
//=======
//>>>>>>> f02c6e4db5109c951cbfaae30249002d98273d54

    public class Double {
        protected double v;

        public Double(double inputValue) {
            this.v = inputValue;
        }

        protected double sin() {
            return Math.sin(v);
        }

        protected double cos() {
            return Math.cos(v);
        }
    }
}