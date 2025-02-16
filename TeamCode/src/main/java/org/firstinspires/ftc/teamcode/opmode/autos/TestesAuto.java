package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.ArmPIDF;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.opencv.core.Mat;

@Autonomous (name = "Auto Tensa Zangetsu!!1!")
public class TestesAuto extends LinearOpMode {
    IMU imu;

    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MecanumDrive peixinho = new MecanumDrive(hardwareMap, new Pose2d(0,0, Math.toRadians(0)));

        Action splinei, splineii, ajeita, samplei;

        splinei = peixinho.actionBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .splineTo(new Vector2d(28,34), Math.toRadians(0))
                .splineTo(new Vector2d(20, 3), Math.toRadians(-90))
                .waitSeconds(0.4)
                .splineTo(new Vector2d(45, -8), Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(10, -8))
                .splineToConstantHeading(new Vector2d(46, -14), Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(10, -17), Math.toRadians(-180))
                .turnTo(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(28, 34, Math.toRadians(0)), Math.toRadians(0))
                .build();

        waitForStart();
        Actions.runBlocking(
                splinei
        );


    }
}
