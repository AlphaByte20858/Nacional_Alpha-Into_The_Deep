package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constraints {
    public static class ElevatorConstraints{
        public double kp = 0.045;
        public double ki = 0;
        public double kd = 0.0002;
        public double kf = 0.15;
        public final int highPosition = -1900;
        public final int midPosition = -1000;
        public final int lowPosition = 0;
        public final int basketHigh = -3600;
        public int errorMargin = 10;
    }
    public static class DriveBaseConstraints{

    }
    public static class ArmConstraints{
        public final int motorEncoder = 28;
        public final double motorReduction = (5*4*3);
        public final int motorRPM = 6000;

        public final static double kp = -0.0005, ki = 0, kd = -0.000009, kf = 0.00005;
        public final double ticksInDegree = (motorEncoder * motorReduction)/180;

        //todo: refazer as posições
        public final int highPosition = 30;
        public final int midPosition = -200;
        public final int lowPosition = -390;
        public final int sampleGet = -630;
        public final int samplePut = -180;
        public int errorMargin = 3;
    }
    public static class ClawConstraints{

    }
}
