package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous (name = "Auto Tensa Zangetsu!!1!")
public class TestesAuto extends LinearOpMode {
    IMU imu;

    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MecanumDrive peixinho = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        Action plusOne, get2, plusTwo, splinei, splineii, ajeita, samplei;


        splinei = peixinho.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .splineTo(new Vector2d(28, 34), Math.toRadians(0))
                .build();

        splineii = peixinho.actionBuilder(new Pose2d(20, 34, Math.toRadians(0)))
                .splineTo(new Vector2d(16, 12), Math.toRadians(-90))
                .build();

        ajeita = peixinho.actionBuilder(new Pose2d(16, 12, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(46, -4, Math.toRadians(-90)), Math.toRadians(-90))
                /*.strafeTo(new Vector2d(46, 3))
                .strafeTo(new Vector2d(46, -8))
                .strafeToConstantHeading(new Vector2d(10, -8))
                .splineToConstantHeading(new Vector2d(46, -12), Math.toRadians(-90))*/
                .build();


        samplei = peixinho.actionBuilder(new Pose2d(43, -6, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(2, -10), Math.toRadians(-90))
                .build();

        plusOne = peixinho.actionBuilder(new Pose2d(2, -10, Math.toRadians(90)))
                .splineTo(new Vector2d(28, 38), Math.toRadians(0))
                .build();

        get2 = peixinho.actionBuilder(new Pose2d(28, 38, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(0, 9, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        plusTwo = peixinho.actionBuilder(new Pose2d(0, 9, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(28, 40, Math.toRadians(0)), Math.toRadians(0))
                .build();

        waitForStart();
        Actions.runBlocking(new SequentialAction(
                splinei
        ));
        sleep(500);
        sleep(400);
        Actions.runBlocking(new SequentialAction(
                splineii,
                ajeita
                ,samplei
        ));
        /*
        Actions.runBlocking(
                plusOne
        );
        sleep(500);
        sleep(400);
        Actions.runBlocking(get2);
        sleep(500);
        Actions.runBlocking(plusTwo);*/
    }

}