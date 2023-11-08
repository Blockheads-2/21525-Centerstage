package org.firstinspires.ftc.teamcode.auto.dispatch;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;

import com.sun.tools.javac.util.List;

public class GPS {
    double pos[] = new double[3]; //pos[0] = x, pos[1] = y, pos[2] = theta (heading)
    int clickPos[] = new int[4];
    HardwareDrive robot;

    public GPS(HardwareDrive robot){
        this.robot = robot;

        clickPos[0] = robot.lf.getCurrentPosition();
        clickPos[1] = robot.rf.getCurrentPosition();
        clickPos[2] = robot.lb.getCurrentPosition();
        clickPos[3] = robot.rb.getCurrentPosition();
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

    public double[] getPos(){
        return pos;
    }
}
