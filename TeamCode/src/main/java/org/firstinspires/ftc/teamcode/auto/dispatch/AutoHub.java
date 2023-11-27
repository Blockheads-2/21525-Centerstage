
package org.firstinspires.ftc.teamcode.auto.dispatch;

import static android.os.SystemClock.sleep;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auto.math.MathConstHead;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

import org.firstinspires.ftc.teamcode.common.pid.TurnPIDController;
import org.firstinspires.ftc.teamcode.common.positioning.MathSpline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


public class AutoHub {

    private final LinearOpMode linearOpMode;


// Declare OpMode members.

    public HardwareDrive robot = null;   // Use a Pushbot's hardware
    HardwareMap hardwareMap;
    public ElapsedTime runtime = new ElapsedTime();
    Constants constants = new Constants();
    MathSpline mathSpline = new MathSpline();
    MathConstHead mathConstHead = new MathConstHead();

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    static final double     COUNTS_PER_MOTOR_REV    = Constants.CPR;    // 537.7
    static final double     MAX_VELOCITY_DT         = 2700;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   =  (96.0/25.4);     // For figuring circumference
    static final double     COUNTS_PER_INCH         = Constants.CPI;
    public static boolean over = false;
    public static boolean finishedIntake = false;
    public static boolean checkOver = false;
    public static boolean checkOver2 = false;

    double startRunTime = 0;

    TelemetryPacket packet;
    FtcDashboard dashboard;

    View relativeLayout;

    GPS gps;

    public AutoHub(LinearOpMode plinear){

        linearOpMode = plinear;
        hardwareMap = linearOpMode.hardwareMap;

        robot = new HardwareDrive();

        robot.init(hardwareMap);

//        gps = new GPS(robot);
        runtime.reset();

        // Send telemetry message to signify robot waiting;
        linearOpMode.telemetry.addData("Status", "Resetting Encoders and Camera");
        linearOpMode.telemetry.update();

        // Get a reference to the RelativeLayout so we can later change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearOpMode.telemetry.addData("Status", "Waiting on Camera");
        linearOpMode.telemetry.update();

        //if (robot.colorSensor instanceof SwitchableLight) {
          //  ((SwitchableLight)robot.colorSensor).enableLight(true);
        //}
    }

    public void  initCamera(Telemetry t){
        if (robot != null) robot.initCamera(t);
    }

    public VisionPortal getVisionPortal(){
        return robot.getVisionPortal();
    }

    public AprilTagProcessor getAprilTagProcessor(){
        return robot.getAprilTagProcessor();
    }

    public void initTelemetry(FtcDashboard dashboard, TelemetryPacket packet){
        this.dashboard = dashboard;
        this.packet = packet;
    }

    public void updateTelemetry(){
//        packet.put("Top Left Power", robot.lf.getPower());
//        packet.put("Top Right Power", robot.rf.getPower());
//        packet.put("Bottom Left Power", robot.lb.getPower());
//        packet.put("Bottom Right Power", robot.rb.getPower());
//
//        packet.put("Top Left Encoder Position", robot.lf.getCurrentPosition());
//        packet.put("Top Right Encoder Position", robot.rf.getCurrentPosition());
//        packet.put("Bottom Left Encoder Position", robot.lb.getCurrentPosition());
//        packet.put("Bottom Right Encoder Position", robot.rb.getCurrentPosition());
//        packet.put("Yaw", -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//        packet.put("Yaw (Absolute Angle)", getAbsoluteAngle());

        linearOpMode.telemetry.addData("Top Left Encoder Position", robot.lf.getCurrentPosition());
        linearOpMode.telemetry.addData("Top Right Encoder Position", robot.rf.getCurrentPosition());
        linearOpMode.telemetry.addData("Bottom Left Encoder Position", robot.lb.getCurrentPosition());
        linearOpMode.telemetry.addData("Bottom Right Encoder Position", robot.rb.getCurrentPosition());

        linearOpMode.telemetry.update();
//        dashboard.sendTelemetryPacket(packet);
    }

    //====================================================================================
    //====================================================================================

    //Core Movement
    public void variableHeading(double speed, double xPose, double yPose, double timeoutS) {
        int FleftEncoderTarget;
        int FrightEncoderTarget;
        int BleftEncoderTarget;
        int BrightEncoderTarget;

        double leftDistance;
        double rightDistance;
        double deltaTheta;
        double deltaTime;
        double zeta;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            speed = speed * MAX_VELOCITY_DT;

            mathSpline.setFinalPose(xPose,yPose);

            leftDistance = mathSpline.returnLDistance() * COUNTS_PER_INCH;
            rightDistance = mathSpline.returnRDistance() * COUNTS_PER_INCH;
            deltaTheta = mathSpline.returnTheta();
            deltaTime = leftDistance / (mathSpline.returnLPower() * constants.CPI);
            zeta = deltaTheta/deltaTime;

            double startingAngle = getAbsoluteAngle();
            double targetAngle;

            if ((yPose >= 0 && xPose < 0) || (yPose < 0 && xPose >= 0)){
                FleftEncoderTarget = robot.lf.getCurrentPosition() - (int) leftDistance;
                FrightEncoderTarget = robot.rf.getCurrentPosition() - (int) rightDistance;
                BleftEncoderTarget = robot.lb.getCurrentPosition() - (int) leftDistance;
                BrightEncoderTarget = robot.rb.getCurrentPosition() - (int) rightDistance;
            }
            else {
                FleftEncoderTarget = robot.lf.getCurrentPosition() + (int) leftDistance;
                FrightEncoderTarget = robot.rf.getCurrentPosition() + (int) rightDistance;
                BleftEncoderTarget = robot.lb.getCurrentPosition() + (int) leftDistance;
                BrightEncoderTarget = robot.rb.getCurrentPosition() + (int) rightDistance;
            }

            robot.lf.setTargetPosition(FleftEncoderTarget);
            robot.lb.setTargetPosition(BleftEncoderTarget);
            robot.rf.setTargetPosition(FrightEncoderTarget);
            robot.rb.setTargetPosition(BrightEncoderTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) && robot.lf.isBusy() && robot.rf.isBusy()
                    && robot.lb.isBusy() && robot.rb.isBusy()) {

//                checkButton();
                robot.lf.setVelocity(speed * mathSpline.returnLPower());
                robot.rf.setVelocity(speed * mathSpline.returnRPower());
                robot.lb.setVelocity(speed * mathSpline.returnLPower());
                robot.rb.setVelocity(speed * mathSpline.returnRPower());


            }

            // Stop all motion;
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void variableHeading(double speed, double xPose, double yPose) {
        int FleftEncoderTarget;
        int FrightEncoderTarget;
        int BleftEncoderTarget;
        int BrightEncoderTarget;

        double leftDistance;
        double rightDistance;
        double deltaTheta;
        double deltaTime;
        double zeta;
        double timeoutS;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {

            speed = speed * MAX_VELOCITY_DT;

            mathSpline.setFinalPose(xPose,yPose);

            leftDistance = mathSpline.returnLDistance() * COUNTS_PER_INCH;
            rightDistance = mathSpline.returnRDistance() * COUNTS_PER_INCH;
            deltaTheta = mathSpline.returnTheta();
            deltaTime = leftDistance / (mathSpline.returnLPower() * constants.CPI);
            zeta = deltaTheta/deltaTime;

            double startingAngle = getAbsoluteAngle();
            double targetAngle;

            timeoutS = (mathSpline.returnDistance() * constants.CPI) / speed;

            if ((yPose >= 0 && xPose < 0) || (yPose < 0 && xPose >= 0)){
                FleftEncoderTarget = robot.lf.getCurrentPosition() - (int) leftDistance;
                FrightEncoderTarget = robot.rf.getCurrentPosition() - (int) rightDistance;
                BleftEncoderTarget = robot.lb.getCurrentPosition() - (int) leftDistance;
                BrightEncoderTarget = robot.rb.getCurrentPosition() - (int) rightDistance;
            }
            else {
                FleftEncoderTarget = robot.lf.getCurrentPosition() + (int) leftDistance;
                FrightEncoderTarget = robot.rf.getCurrentPosition() + (int) rightDistance;
                BleftEncoderTarget = robot.lb.getCurrentPosition() + (int) leftDistance;
                BrightEncoderTarget = robot.rb.getCurrentPosition() + (int) rightDistance;
            }

            robot.lf.setTargetPosition(FleftEncoderTarget);
            robot.lb.setTargetPosition(BleftEncoderTarget);
            robot.rf.setTargetPosition(FrightEncoderTarget);
            robot.rb.setTargetPosition(BrightEncoderTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) && robot.lf.isBusy() && robot.rf.isBusy()
                    && robot.lb.isBusy() && robot.rb.isBusy()) {

                targetAngle = startingAngle + zeta * (runtime.milliseconds() + 1);

//                checkButton();
//                detectColor();

                TurnPIDController pidTurn = new TurnPIDController(targetAngle, 0.01, 0, 0.003);

                double angleCorrection = pidTurn.update(getAbsoluteAngle());

                robot.lf.setVelocity(speed * mathSpline.returnLPower());
                robot.rf.setVelocity(speed * mathSpline.returnRPower());
                robot.lb.setVelocity(speed * mathSpline.returnLPower());
                robot.rb.setVelocity(speed * mathSpline.returnRPower());

                linearOpMode.telemetry.addData("Time", timeoutS);
            }

            // Stop all motion;
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void constantHeading(double speed, double xPose, double yPose, double timeoutS, double kP, double kI, double kD) {
        mathConstHead.setFinalPose(xPose,yPose);

        double targetAngle = getAbsoluteAngle();
        TurnPIDController pidTurn = new TurnPIDController(targetAngle, kP, kI, kD);


        double distance = mathConstHead.returnDistance();
        double radianAngle = mathConstHead.returnAngle();

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        double ratioAddPose = Math.cos(radianAngle) + Math.sin(radianAngle);
        double ratioSubPose = Math.cos(radianAngle) - Math.sin(radianAngle);
        double addPose = (ratioAddPose * COUNTS_PER_INCH * distance);
        double subtractPose = (ratioSubPose * COUNTS_PER_INCH * distance);

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = (int) (robot.lf.getCurrentPosition() + addPose);
            newRightFrontTarget = (int) (robot.rf.getCurrentPosition() + subtractPose);
            newLeftBackTarget = (int) (robot.lb.getCurrentPosition() + subtractPose);
            newRightBackTarget = (int) (robot.rb.getCurrentPosition() + addPose);

            robot.lf.setTargetPosition(newLeftFrontTarget);
            robot.rf.setTargetPosition(newRightFrontTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {


                double angleCorrection = pidTurn.update(getAbsoluteAngle());

//                checkButton();
//                detectColor();

                robot.lf.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioAddPose) - (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.rf.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioSubPose) + (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.lb.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioSubPose) - (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.rb.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioAddPose) + (speed * angleCorrection * constants.MAX_VELOCITY_DT));

                // Display it for the driver.
                linearOpMode.telemetry.addData("lf", speed * constants.MAX_VELOCITY_DT * ratioAddPose);
                linearOpMode.telemetry.addData("rf",speed * constants.MAX_VELOCITY_DT * ratioSubPose);
                linearOpMode.telemetry.addData("Left Fromt Velocity: ", robot.lf.getVelocity());
                linearOpMode.telemetry.addData("Right Front Velocity: ", robot.rf.getVelocity());

                linearOpMode.telemetry.addData("Left Back Velocity: ", robot.lb.getVelocity());
                linearOpMode.telemetry.addData("Right Back Velocity: ", robot.rb.getVelocity());
                linearOpMode.telemetry.addData("sub pose", subtractPose);
                linearOpMode.telemetry.addData("add pose", addPose);
                linearOpMode.telemetry.update();
            }

            // Stop all motion;

            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void constantHeadingV2(double movePower, double x, double y, double theta, double kp, double ki, double kd){
        mathConstHead.setFinalPose(x,y);

        updateTelemetry();

        double targetAngle = theta; //want to keep heading constant (current angle)

        TurnPIDController pidTurn = new TurnPIDController(targetAngle, kp, ki, kd);

        double distance = mathConstHead.returnDistance();

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        double timeoutS;

        timeoutS = distance / (movePower * constants.CPI);

        double rightDiagonalRatio = mathConstHead.getRatios()[0];
        double leftDiagonalRatio = mathConstHead.getRatios()[1];

        double rightDiagonalPos = rightDiagonalRatio * Constants.CPI * distance;
        double leftDiagonalPos = leftDiagonalRatio * Constants.CPI  * distance;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = (int) (robot.lf.getCurrentPosition() + leftDiagonalPos);
            newRightFrontTarget = (int) (robot.rf.getCurrentPosition() + rightDiagonalPos);
            newLeftBackTarget = (int) (robot.lb.getCurrentPosition() + rightDiagonalPos);
            newRightBackTarget = (int) (robot.rb.getCurrentPosition() + leftDiagonalPos);

            packet.put("Top Left Encoder Target", newLeftFrontTarget);
            packet.put("Top Right Encoder Target", newRightFrontTarget);
            packet.put("Bottom Left Encoder Target", newLeftBackTarget);
            packet.put("Bottom Right Encoder Target", newRightBackTarget);

            robot.lf.setTargetPosition(newLeftFrontTarget);
            robot.rf.setTargetPosition(newRightFrontTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {

//                checkButton();
//                detectColor();
//                gps.periodic(runtime.seconds());

                double angleCorrection = pidTurn.update(getAbsoluteAngle());

                robot.lf.setVelocity((movePower * constants.MAX_VELOCITY_DT * leftDiagonalRatio) - (movePower * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.rf.setVelocity((movePower * constants.MAX_VELOCITY_DT * rightDiagonalRatio) + (movePower * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.lb.setVelocity((movePower * constants.MAX_VELOCITY_DT * rightDiagonalRatio) - (movePower * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.rb.setVelocity((movePower * constants.MAX_VELOCITY_DT * leftDiagonalRatio) + (movePower * angleCorrection * constants.MAX_VELOCITY_DT));


                // Display it for the driver.
//                linearOpMode.telemetry.addData("Time: ", timeoutS);
//                linearOpMode.telemetry.addData("X", gps.getPose().getTranslation().getX());
//                linearOpMode.telemetry.addData("Y", gps.getPose().getTranslation().getY());
//                linearOpMode.telemetry.addData("R", gps.getPose().getRotation().getDegrees());
//                linearOpMode.telemetry.addData("R (IMU)", -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

//                linearOpMode.telemetry.update();

                updateTelemetry();
            }

            // Stop all motion;
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            updateTelemetry();
        }
    }
    public void constantHeadingV2(double movePower, double x, double y, double kp, double ki, double kd){
        mathConstHead.setFinalPose(x,y);

        updateTelemetry();

        double targetAngle = getAbsoluteAngle(); //want to keep heading constant (current angle)

        TurnPIDController pidTurn = new TurnPIDController(targetAngle, kp, ki, kd);

        double distance = mathConstHead.returnDistance();

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        double timeoutS;

        timeoutS = distance / (movePower * constants.CPI);

        double rightDiagonalRatio = mathConstHead.getRatios()[0];
        double leftDiagonalRatio = mathConstHead.getRatios()[1];

        double rightDiagonalPos = rightDiagonalRatio * Constants.CPI * distance;
        double leftDiagonalPos = leftDiagonalRatio * Constants.CPI  * distance;

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = (int) (robot.lf.getCurrentPosition() + leftDiagonalPos);
            newRightFrontTarget = (int) (robot.rf.getCurrentPosition() + rightDiagonalPos);
            newLeftBackTarget = (int) (robot.lb.getCurrentPosition() + rightDiagonalPos);
            newRightBackTarget = (int) (robot.rb.getCurrentPosition() + leftDiagonalPos);

//            packet.put("Top Left Encoder Target", newLeftFrontTarget);
//            packet.put("Top Right Encoder Target", newRightFrontTarget);
//            packet.put("Bottom Left Encoder Target", newLeftBackTarget);
//            packet.put("Bottom Right Encoder Target", newRightBackTarget);

            robot.lf.setTargetPosition(newLeftFrontTarget);
            robot.rf.setTargetPosition(newRightFrontTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {

//                checkButton();
//                detectColor();
//                gps.periodic(runtime.seconds());

//                double angleCorrection = pidTurn.update(getAbsoluteAngle());

//                robot.lf.setVelocity((movePower * constants.MAX_VELOCITY_DT * leftDiagonalRatio) - (movePower * angleCorrection * constants.MAX_VELOCITY_DT));
//                robot.rf.setVelocity((movePower * constants.MAX_VELOCITY_DT * rightDiagonalRatio) + (movePower * angleCorrection * constants.MAX_VELOCITY_DT));
//                robot.lb.setVelocity((movePower * constants.MAX_VELOCITY_DT * rightDiagonalRatio) - (movePower * angleCorrection * constants.MAX_VELOCITY_DT));
//                robot.rb.setVelocity((movePower * constants.MAX_VELOCITY_DT * leftDiagonalRatio) + (movePower * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.lf.setVelocity((movePower * constants.MAX_VELOCITY_DT * leftDiagonalRatio));
                robot.rf.setVelocity((movePower * constants.MAX_VELOCITY_DT * rightDiagonalRatio));
                robot.lb.setVelocity((movePower * constants.MAX_VELOCITY_DT * rightDiagonalRatio));
                robot.rb.setVelocity((movePower * constants.MAX_VELOCITY_DT * leftDiagonalRatio));

                // Display it for the driver.
//                linearOpMode.telemetry.addData("Time: ", timeoutS);
//                linearOpMode.telemetry.addData("X", gps.getPose().getTranslation().getX());
//                linearOpMode.telemetry.addData("Y", gps.getPose().getTranslation().getY());
//                linearOpMode.telemetry.addData("R", gps.getPose().getRotation().getDegrees());
//                linearOpMode.telemetry.addData("R (IMU)", -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

//                linearOpMode.telemetry.update();

                updateTelemetry();
            }

            // Stop all motion;
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            updateTelemetry();

        }
    }
    public void constantHeading(double speed, double xPose, double yPose, double kP, double kI, double kD) {
        mathConstHead.setFinalPose(xPose,yPose);

        updateTelemetry();

        double targetAngle = getAbsoluteAngle();
        TurnPIDController pidTurn = new TurnPIDController(targetAngle, kP, kI, kD);


        double distance = mathConstHead.returnDistance();
        double radianAngle = mathConstHead.returnAngle();

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        double timeoutS;

        double ratioAddPose = Math.cos(radianAngle) + Math.sin(radianAngle);
        double ratioSubPose = Math.cos(radianAngle) - Math.sin(radianAngle);
        double addPose = (ratioAddPose * COUNTS_PER_INCH * distance);
        double subtractPose = (ratioSubPose * COUNTS_PER_INCH * distance);

        timeoutS = distance / (speed * Constants.CPI);

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = (int) (robot.lf.getCurrentPosition() + addPose);
            newRightFrontTarget = (int) (robot.rf.getCurrentPosition() + subtractPose);
            newLeftBackTarget = (int) (robot.lb.getCurrentPosition() + subtractPose);
            newRightBackTarget = (int) (robot.rb.getCurrentPosition() + addPose);

            robot.lf.setTargetPosition(newLeftFrontTarget);
            robot.rf.setTargetPosition(newRightFrontTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {

//                checkButton();
//                detectColor();

                double angleCorrection = pidTurn.update(getAbsoluteAngle());

                robot.lf.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioAddPose) - (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.rf.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioSubPose) + (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.lb.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioSubPose) - (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.rb.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioAddPose) + (speed * angleCorrection * constants.MAX_VELOCITY_DT));

                // Display it for the driver.
                linearOpMode.telemetry.addData("Time: ", timeoutS);
                linearOpMode.telemetry.addData("lf Target Position:", robot.lf.getTargetPosition());
                linearOpMode.telemetry.addData("rf Target Position:", robot.rf.getTargetPosition());
                linearOpMode.telemetry.addData("lb Target Position:", robot.lb.getTargetPosition());
                linearOpMode.telemetry.addData("rb Target Position:", robot.rb.getTargetPosition());
//                linearOpMode.telemetry.update();

                updateTelemetry();
            }

            // Stop all motion;
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            updateTelemetry();

        }
    }
    public void constantHeading(double speed, double xPose, double yPose, boolean check, double kP, double kI, double kD) {
        mathConstHead.setFinalPose(xPose,yPose);

        double targetAngle = getAbsoluteAngle();
        TurnPIDController pidTurn = new TurnPIDController(targetAngle, kP, kI, kD);


        double distance = mathConstHead.returnDistance();
        double radianAngle = mathConstHead.returnAngle();

        checkOver = false;
        checkOver2 = false;
        over = false;

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        double timeoutS;

        double ratioAddPose = Math.cos(radianAngle) + Math.sin(radianAngle);
        double ratioSubPose = Math.cos(radianAngle) - Math.sin(radianAngle);
        double addPose = (ratioAddPose * COUNTS_PER_INCH * distance);
        double subtractPose = (ratioSubPose * COUNTS_PER_INCH * distance);

        timeoutS = distance / (speed * constants.CPI);

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = (int) (robot.lf.getCurrentPosition() + addPose);
            newRightFrontTarget = (int) (robot.rf.getCurrentPosition() + subtractPose);
            newLeftBackTarget = (int) (robot.lb.getCurrentPosition() + subtractPose);
            newRightBackTarget = (int) (robot.rb.getCurrentPosition() + addPose);

            robot.lf.setTargetPosition(newLeftFrontTarget);
            robot.rf.setTargetPosition(newRightFrontTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) && !over) {

//                checkButton();
//                detectColor();
//                over = detectFloor();

                double angleCorrection = pidTurn.update(getAbsoluteAngle());

                robot.lf.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioAddPose) - (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.rf.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioSubPose) + (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.lb.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioSubPose) - (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.rb.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioAddPose) + (speed * angleCorrection * constants.MAX_VELOCITY_DT));

                //linearOpMode.telemetry.addData("Time",timeoutS);
                linearOpMode.telemetry.update();

            }

            // Stop all motion;
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void constantHeading(double speed, double xPose, double yPose, double kP, double kI, double kD, boolean stop) {
        mathConstHead.setFinalPose(xPose,yPose);

        double targetAngle = getAbsoluteAngle();
        TurnPIDController pidTurn = new TurnPIDController(targetAngle, kP, kI, kD);


        double distance = mathConstHead.returnDistance();
        double radianAngle = mathConstHead.returnAngle();

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        double timeoutS;

        double ratioAddPose = Math.cos(radianAngle) + Math.sin(radianAngle);
        double ratioSubPose = Math.cos(radianAngle) - Math.sin(radianAngle);
        double addPose = (ratioAddPose * COUNTS_PER_INCH * distance);
        double subtractPose = (ratioSubPose * COUNTS_PER_INCH * distance);

        timeoutS = distance / (speed * constants.CPI);

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = (int) (robot.lf.getCurrentPosition() + addPose);
            newRightFrontTarget = (int) (robot.rf.getCurrentPosition() + subtractPose);
            newLeftBackTarget = (int) (robot.lb.getCurrentPosition() + subtractPose);
            newRightBackTarget = (int) (robot.rb.getCurrentPosition() + addPose);

            robot.lf.setTargetPosition(newLeftFrontTarget);
            robot.rf.setTargetPosition(newRightFrontTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) && !over) {

//                checkButton();
//                detectColor();

                double angleCorrection = pidTurn.update(getAbsoluteAngle());

                robot.lf.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioAddPose) - (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.rf.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioSubPose) + (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.lb.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioSubPose) - (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.rb.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioAddPose) + (speed * angleCorrection * constants.MAX_VELOCITY_DT));

                //linearOpMode.telemetry.addData("Time",timeoutS);
                linearOpMode.telemetry.update();

            }

            // Stop all motion;
            if (stop) {
                robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void constVarHead(double speed, double xPose, double yPose, double turnAngle, double timeOut){

        mathConstHead.setFinalPose(xPose,yPose);

        TurnPIDController pidTurn = new TurnPIDController(turnAngle, 0.001,0,0.0003);


        double distance = mathConstHead.returnDistance();
        double radianAngle = mathConstHead.returnAngle();

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        double timeoutS;

        double ratioAddPose = Math.cos(radianAngle) + Math.sin(radianAngle);
        double ratioSubPose = Math.cos(radianAngle) - Math.sin(radianAngle);
        double addPose = (ratioAddPose * COUNTS_PER_INCH * distance);
        double subtractPose = (ratioSubPose * COUNTS_PER_INCH * distance);

        timeoutS = distance / (speed * constants.CPI);

        // Ensure that the opmode is still active
        if (linearOpMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = (int) (robot.lf.getCurrentPosition() + addPose);
            newRightFrontTarget = (int) (robot.rf.getCurrentPosition() + subtractPose);
            newLeftBackTarget = (int) (robot.lb.getCurrentPosition() + subtractPose);
            newRightBackTarget = (int) (robot.rb.getCurrentPosition() + addPose);

            robot.lf.setTargetPosition(newLeftFrontTarget);
            robot.rf.setTargetPosition(newRightFrontTarget);
            robot.lb.setTargetPosition(newLeftBackTarget);
            robot.rb.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (linearOpMode.opModeIsActive() && (runtime.seconds() < timeOut)) {

//                checkButton();
//                detectColor();

                double angleCorrection = pidTurn.update(getAbsoluteAngle());

                robot.lf.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioAddPose) - (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.rf.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioSubPose) + (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.lb.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioSubPose) - (speed * angleCorrection * constants.MAX_VELOCITY_DT));
                robot.rb.setVelocity((speed * constants.MAX_VELOCITY_DT * ratioAddPose) + (speed * angleCorrection * constants.MAX_VELOCITY_DT));

                // Display it for the driver.
                linearOpMode.telemetry.addData("Time: ", timeoutS);
                linearOpMode.telemetry.update();
            }



            // Stop all motion;
            robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }



    //Turn
    public void resetAngle(){
        lastAngles = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }
    public double getAngle() {
        // Get current orientation
        Orientation orientation = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        linearOpMode.telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }
    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (linearOpMode.opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.lf.setPower(-motorPower);
            robot.rf.setPower(motorPower);
            robot.lb.setPower(-motorPower);
            robot.rb.setPower(motorPower);

//            detectColor();
//            checkButton();

            error = degrees - getAngle();
            linearOpMode.telemetry.addData("error", error);
            linearOpMode.telemetry.update();
        }

        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);

    }

    public void absoluteTurn(double theta){
        double currAngle = -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double error = deltaAngle(theta, currAngle);

        while (linearOpMode.opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.lf.setPower(-motorPower);
            robot.rf.setPower(motorPower);
            robot.lb.setPower(-motorPower);
            robot.rb.setPower(motorPower);

//            detectColor();
//            checkButton();

            error = deltaAngle(theta, currAngle);
            linearOpMode.telemetry.addData("error", error);
            linearOpMode.telemetry.update();
        }

        robot.lf.setPower(0);
        robot.rf.setPower(0);
        robot.lb.setPower(0);
        robot.rb.setPower(0);
    }

    public double getAbsoluteAngle() {
//        return robot.imu.getRobotOrientation(
//                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
//        ).firstAngle;
        return -robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public static double deltaAngle(double target, double current){
        double target2 = (target < 0 ? target + 360 : target);
        double current2 = (current < 0 ? current + 360 : current);
        double turnAmount1 = target - current;
        double turnAmount2 = target2 - current2;
        return (Math.abs(turnAmount1) < Math.abs(turnAmount2) ? turnAmount1 : turnAmount2);
    }

    public void turnPID(double degrees,double timeOut) {
        turnMath(-degrees + getAbsoluteAngle(), timeOut);
    }
    public void turnAbsPID(double absDegrees, double timeOut){
        turnMath(-absDegrees, timeOut);
    }
    void turnMath(double targetAngle, double timeoutS) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        linearOpMode.telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        runtime.reset();
        while ((runtime.seconds() < timeoutS) && (Math.abs(targetAngle - getAbsoluteAngle()) > 0.25 || pid.getLastSlope() > 0.15)) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.lf.setPower(-motorPower);
            robot.rf.setPower(motorPower);
            robot.lb.setPower(-motorPower);
            robot.rb.setPower(motorPower);

//            detectColor();

//            checkButton();

            linearOpMode.telemetry.addData("Current Angle", getAbsoluteAngle());
            linearOpMode.telemetry.addData("Target Angle", targetAngle);
            linearOpMode.telemetry.addData("Slope", pid.getLastSlope());
            linearOpMode.telemetry.addData("Power", motorPower);
            linearOpMode.telemetry.update();
        }
        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        constantHeading(1,0,0,0,0,0); //Brakes
    }

    //Peripheral Movements

    public void spinIntake(double power, double timeout){
        runtime.reset();

        while (linearOpMode.opModeIsActive() && runtime.seconds() <= timeout){
            robot.intake.setPower(power);
        }
        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intake.setPower(0);
    }

//    public void spinCarousel(double velocity){
//        robot.duckWheel.setVelocity(velocity);
//    }
//    public void spinCarousel(double velocity, long spinTime){
//        robot.duckWheel.setVelocity(velocity);
//        robot.lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        sleep(spinTime);
//        robot.duckWheel.setVelocity(0);
//    }



//    public void spinIntake(double power){
//        robot.spin.setPower(power);
//    }
//    public void spinIntake(double power, long spinTime){
//        robot.spin.setPower(power);
//
//        // Stop all motion;
//        robot.lf.setPower(0);
//        robot.rf.setPower(0);
//        robot.lb.setPower(0);
//        robot.rb.setPower(0);
//
//        sleep(spinTime);
//        robot.spin.setPower(0);
//    }
//    public void moveElevator(int elevatorPosition){
//        robot.lifter.setTargetPosition(elevatorPosition);
//        robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.lifter.setPower(1);
//    }
    public void breakPoint(){
        while (!linearOpMode.gamepad1.a) {
            sleep(1);
        }
    }

//    public void checkButton(){
//        if (!robot.digitalTouch.getState()) {
//            //Stop
//            robot.lifter.setPower(0);
//
//            //Reset
//            robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            robot.lifter.setTargetPosition(20);
//            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.lifter.setPower(0.1);
//        }
//    }
//    public void detectColor() {
//        // Get the normalized colors from the sensor
//        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
//
//        robot.colorSensor.setGain(3);
//
//        //In
//        if (finishedIntake && ((runtime.milliseconds() - startRunTime) > 500)){
//            spinIntake(0.05);
//        } else if ((((DistanceSensor) robot.colorSensor).getDistance(DistanceUnit.CM) <= 5.5)) {
//            spinIntake(0);
//        }
//        //Empty
//        if ((((DistanceSensor) robot.colorSensor).getDistance(DistanceUnit.CM) > 5.5)){
//            finishedIntake = false;
//        }
//
//    }
//
//
//    public boolean detectFloor() {
//        NormalizedRGBA floorColors = robot.colorFloorSensor.getNormalizedColors();
//        robot.colorFloorSensor.setGain(3);
//        robot.colorFloorSensor2.setGain(3);
//        checkOver = floorColors.alpha >= 0.25 && floorColors.red >= 0.0090 && floorColors.green >= 0.0090 && floorColors.blue >= 0.0090 ;
//
//        NormalizedRGBA floorColors2 = robot.colorFloorSensor2.getNormalizedColors();
//
//        checkOver2 = floorColors2.alpha >= 0.25 && floorColors2.red >= 0.0090 && floorColors2.green >= 0.0090 && floorColors2.blue >= 0.0090 ;
//
//
//        return checkOver || checkOver2;
//    }
}

