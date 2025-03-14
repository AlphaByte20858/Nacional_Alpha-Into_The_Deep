package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.lang.reflect.Array;
import java.util.Arrays;

@Autonomous (name = "Auto Tensa Zangetsu!!1!")
public class TestesAuto extends LinearOpMode {
    IMU imu;

    public void runOpMode() {
        MecanumDrive peixinho = new MecanumDrive(hardwareMap, new Pose2d(0,0, Math.toRadians(0)));
/*
        Action sample1, sample2, sample3, basket1, basket2, basket3, basket4;
        basket1 = peixinho.actionBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(4, 39, Math.toRadians(135)), Math.toRadians(135))
                .build();

        sample1 = peixinho.actionBuilder(new Pose2d(12, 45, Math.toRadians(135)))
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(17, 34, Math.toRadians(0)), Math.toRadians(0))
                .build();

        basket2 = peixinho.actionBuilder(new Pose2d(12, 29, Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(7, 42, Math.toRadians(145)), Math.toRadians(145))
                .build();

        sample2 = peixinho.actionBuilder(new Pose2d(7, 42, Math.toRadians(145)))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(17, 44, Math.toRadians(360)), Math.toRadians(360))
                .build();

        basket3 = peixinho.actionBuilder(new Pose2d(11, 37, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(8, 44, Math.toRadians(145)), Math.toRadians(145))
                .build();

        sample3 = peixinho.actionBuilder(new Pose2d(12, 33, Math.toRadians(145)))
                .splineToLinearHeading(new Pose2d(38, 34, Math.toRadians(90)), Math.toRadians(90))
                .build();

        basket4 = peixinho.actionBuilder(new Pose2d(28, 28, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(8,44, Math.toRadians(145)), Math.toRadians(145))
                .build();

        waitForStart();
        Actions.runBlocking(
                basket1
        );
        sleep(400);
        Actions.runBlocking(sample1);
        sleep(400);
        Actions.runBlocking(basket2);
        sleep(400);
        Actions.runBlocking(sample2);
        sleep(200);
        Actions.runBlocking(basket3);
        sleep(400);
        Actions.runBlocking(sample3);
        sleep(400);
        Actions.runBlocking(basket4);
        sleep(400);*/


        Action get2, plusOne, plusTwo, splinei, splineii, ajeita, samplei; //actions trajetória

        splinei = peixinho.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(30, 10, Math.toRadians(0)), Math.toRadians(0))
                .build();

        splineii = peixinho.actionBuilder(new Pose2d(20, 34, Math.toRadians(0)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(14, -10, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        ajeita = peixinho.actionBuilder(new Pose2d(15, -12, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(50, -24, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        samplei = peixinho.actionBuilder(new Pose2d(50, -25, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(11, -30, Math.toRadians(-180)), Math.toRadians(-180))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(0, -22, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        plusOne = peixinho.actionBuilder(new Pose2d(12, -30, Math.toRadians(-180)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(29, 13, Math.toRadians(0)), Math.toRadians(0))
                .build();

        get2 = peixinho.actionBuilder(new Pose2d(20, 14, Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(0, -15, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        plusTwo = peixinho.actionBuilder(new Pose2d(0, -15, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(29, 18, Math.toRadians(0)), Math.toRadians(0))
                .build();

        waitForStart();
        Actions.runBlocking(
                splinei
        );
        sleep(400);
        Actions.runBlocking(splineii);
        VelConstraint baseVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(75),
                new AngularVelConstraint(Math.PI /2)
        ));
        AccelConstraint baseAcel = new ProfileAccelConstraint(-25, 75);
        sleep(400);
        Actions.runBlocking(ajeita);
        sleep(400);
        Actions.runBlocking(samplei);
        sleep(200);
        Actions.runBlocking(plusOne);
        sleep(400);
        Actions.runBlocking(get2);
        sleep(400);
        Actions.runBlocking(plusTwo);
        sleep(400);

    }

}