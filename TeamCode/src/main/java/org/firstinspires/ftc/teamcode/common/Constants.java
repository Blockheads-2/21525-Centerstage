package org.firstinspires.ftc.teamcode.common;

public class Constants {
    // apriltag stuff
    public static final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    //public static double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    public static final double DESIRED_ID_BLUE = 2; //middle
    public static final double DESIRED_ID_RED = 5; //middle
    public static final double DESIRED_DISTANCE = 8; //  this is how close the camera should get to the target (inches)
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
    public static double DEFAULT_SPEED = 0.75;
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
    public static double HSV_HUE_HIGH_RED = 10;
    public static double HSV_SATURATION_LOW_RED = 100;
    public static double HSV_SATURATION_HIGH_RED = 255;
    public static double HSV_VALUE_LOW_RED = 40;
    public static double HSV_VALUE_HIGH_RED = 255;
    public static double PIXEL_WIDTH_MM = 84.25;
    public static double PIXEL_WIDTH_TO_DISTANCE_FROM_CAMERA = 640.0 / (Math.tan(Math.toRadians(68.0)) * (PIXEL_WIDTH_MM / 2.0)); //pixels to mm; assumes resolution is 640x480

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

    public static final int MAX_LIFT_CLICKS = 1600;
    public static final int STOW_LIFT_CLICKS = 100;
    public static final int MIN_LIFT_CLICKS = 50;
    public static final int LOW_LIFT = 100;
    public static final int MID_LIFT = 1100;
    public static final int HIGH_LIFT = 2250;

    public enum LiftState {
        GROUND,
        DEPOSIT,
        STOW,
        MANUAL
    }



    //Servo
        //Plane
    public static final double HOLD_PLANE = 0.25;
    public static final double RELEASE_PLANE = 0.75;
    public static final double LC_HOLD = 0.42;
    public static final double LC_RELEASE = 0.75;

    public static final double RC_HOLD = 0.42;
    public static final double RC_RELEASE = 0.75;

    public enum rcState {
        hold,
        release
    }

    public enum lcState {
        hold,
        release
    }

    public enum PivotState {
        pickup,
        stow,
        deposit
    }

    public static final double PIVOT_PICKUP = 0;
    public static final double PIVOT_STOW = 0; //same as PIVOT_DEPOSIT.
    public static final double PIVOT_DEPOSIT = 0;

}