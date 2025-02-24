package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous (name = "Auto Tensa Zangetsu!!1!")
public class TestesAuto extends LinearOpMode {
    IMU imu;

    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MecanumDrive peixinho = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        Action basket1, basket2, basket3, sample1, sample2, sample3;

        basket1 = peixinho.actionBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(6, 45, Math.toRadians(135)), Math.toRadians(135))
                .build();

        sample1 = peixinho.actionBuilder(new Pose2d(6, 45, Math.toRadians(135)))
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(16, 34, Math.toRadians(0)), Math.toRadians(0))
                .build();

        basket2 = peixinho.actionBuilder(new Pose2d(16, 34, Math.toRadians(0)))
                .setTangent(Math.toRadians(145))
                .splineToLinearHeading(new Pose2d(6, 45, Math.toRadians(145)), Math.toRadians(145))
                .build();

        sample2 = peixinho.actionBuilder(new Pose2d(16, 45, Math.toRadians(145)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(16, 37, Math.toRadians(0)), Math.toRadians(0))
                        .build();

        basket3 = peixinho.actionBuilder(new Pose2d(16, 37, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(6, 45, Math.toRadians(145)), Math.toRadians(145))
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
        sleep(400);

        Actions.runBlocking(basket3);
    }

}