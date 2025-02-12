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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.opencv.core.Mat;

@Autonomous (name = "Auto Tensa Zangetsu!!1!")
public class TestesAuto extends LinearOpMode {
    IMU imu;

    public void runOpMode(){

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MecanumDrive peixinho = new MecanumDrive(hardwareMap, new Pose2d(0,0, Math.toRadians(0)));

        Action splinei, splineii, splineiii, linearA;

        splinei = peixinho.actionBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .splineTo(new Vector2d(31,34), Math.toRadians(0))
                .build();

        splineii = peixinho.actionBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .splineTo(new Vector2d(24, 5), Math.toRadians(0))
                .waitSeconds(0.4)
                .build();

        splineiii = peixinho.actionBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(40, -3, Math.toRadians(90)), Math.toRadians(90))
                .build();


        waitForStart();
        Actions.runBlocking(new SequentialAction(
                splinei
        ));
        sleep(500);
        sleep(400);
        Actions.runBlocking(new SequentialAction(
                splineii, splineiii));
    }
}
