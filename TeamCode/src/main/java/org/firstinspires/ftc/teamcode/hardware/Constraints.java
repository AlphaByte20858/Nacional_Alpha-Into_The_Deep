package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constraints {
    public static class ElevatorConstraints{
        public double kp = 0;
        public double ki = 0;
        public double kd = 0;
    }
    public static class DriveBaseConstraints{

    }
    public static class ArmConstraints{
        public final int motorEncoder = 28;
        public final double motorReduction = (5*4*3);
        public final int motorRPM = 6000;

        public final static double kp = 0.004, ki = 0, kd = 0.00001, kf = 0.001;
        public final double ticksInDegree = (motorEncoder * motorReduction)/180;

        //todo: refazer as posições
        public final double highPosition = 0;
        public final double midPosition = 0;
        public final double lowPosition = 0;
    }
    public static class ClawConstraints{

    }
}
