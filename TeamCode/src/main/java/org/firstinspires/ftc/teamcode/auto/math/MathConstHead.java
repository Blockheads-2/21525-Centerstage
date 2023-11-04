package org.firstinspires.ftc.teamcode.auto.math;


import org.firstinspires.ftc.teamcode.auto.math.AbsPose;
import org.firstinspires.ftc.teamcode.common.Constants;

public class MathConstHead {
    private double finalXPositionInput = 0; // final x
    private double finalYPositionInput = 0; // final y

    double psiSquared = 0;
    double omegaSquared = 0;
    double psiOverOmega = 0;

    Constants constants = new Constants();
    AbsPose absPose = new AbsPose();


    //Input the Final Position

    /* ----Implement once finish abs pose----
    public void setFinalPose(double xPose, double yPose){
        absPose.setAbsDistance(xPose,yPose);

        psi = absPose.returnDistanceX();
        omega = absPose.returnDistanceY();
    }
     */

    public void setFinalPose(double xPose, double yPose){
        finalXPositionInput = xPose;
        finalYPositionInput = yPose;
    }

    public double returnDistance(){
        // using pythagorean theorem to calculate the distance between the robot and the point
        return Math.sqrt(Math.pow(finalXPositionInput, 2) + Math.pow(finalYPositionInput, 2));
    }

    public double returnAngle(){
        // atan2 returns mPRX in radians where P equals the target point, R equals the robot's
        // position (0, 0) and X equals any applicable point on the x-axis
        return Math.atan2(finalYPositionInput, finalXPositionInput);
    }
}