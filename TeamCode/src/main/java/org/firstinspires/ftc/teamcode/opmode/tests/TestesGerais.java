package org.firstinspires.ftc.teamcode.opmode.tests;

import static org.firstinspires.ftc.teamcode.opmode.tests.PIDfTest.p;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class TestesGerais extends LinearOpMode {
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        ElevatorSubsystem elevador = new ElevatorSubsystem(robot);
        ArmSubsystem arm = new ArmSubsystem(robot);
        ElapsedTime tempo = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MecanumDrive peixinho = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        Action get2, plusOne, plusTwo, splinei, splineii, ajeita, samplei; //actions trajetória
        Action linearHigh, linearMid, linearLow, armHigh, armLow;

        linearHigh = elevador.setHighPosition();
        linearMid = elevador.setMidPosition();
        linearLow = elevador.setLowPosition();
        armHigh = arm.setHighPosition();
        armLow = arm.setLowPosition();

        splinei = peixinho.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(29, 10, Math.toRadians(0)), Math.toRadians(0))
                .build();

        splineii = peixinho.actionBuilder(new Pose2d(20, 34, Math.toRadians(0)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(14, -10, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        ajeita = peixinho.actionBuilder(new Pose2d(16, -12, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(48, -26, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        samplei = peixinho.actionBuilder(new Pose2d(48, -28, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(2, -30, Math.toRadians(-90)), Math.toRadians(0))
                .build();

        plusOne = peixinho.actionBuilder(new Pose2d(2, -31, Math.toRadians(0)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(30, 13, Math.toRadians(0)), Math.toRadians(0))
                .build();

        get2 = peixinho.actionBuilder(new Pose2d(29, 14, Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(0, -15, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        plusTwo = peixinho.actionBuilder(new Pose2d(0, -15, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(29, 18, Math.toRadians(0)), Math.toRadians(0))
                .build();

        arm.init();
        elevador.init();
        robot.clawServo.setPosition(0.23);
//        elevador.pidTarget(0);
        arm.setTarget(0);
        robot.wristServo.setPosition(0);
        waitForStart();
        arm.periodic();
        elevador.periodic();
        Actions.runBlocking(new ParallelAction(splinei, linearMid));
        Actions.runBlocking(linearHigh);
        robot.clawServo.setPosition(0);
        sleep(200);
        Actions.runBlocking(new SequentialAction(new ParallelAction(splineii, linearLow),
                ajeita,
                samplei
        ));
        //segundo sample
        Actions.runBlocking(armLow);
        sleep(400);
        robot.clawServo.setPosition(0.23);
        sleep(100);
        Actions.runBlocking(new ParallelAction(plusOne, armHigh,
                linearMid,
                new InstantAction(() -> {
                    robot.wristServo.setPosition(0.7);
                })
        ));
        robot.wristServo.setPosition(0.7);
        robot.clawServo.setPosition(0);

        sleep(200);
        Actions.runBlocking(linearHigh);
        robot.clawServo.setPosition(0);
        //terceiro sample
        Actions.runBlocking(new SequentialAction(new ParallelAction(
                get2,
                linearLow
        )));
        tempo.reset();
        Actions.runBlocking(armLow);
        //garra ang 6 pos 0.64
        robot.clawServo.setPosition(0.23);
        sleep(100);
        robot.wristServo.setPosition(0);
        Actions.runBlocking((new ParallelAction(plusTwo,
                armHigh,
                linearMid,
                new InstantAction(() -> robot.wristServo.setPosition(0.7)))));
        tempo.reset();
        Actions.runBlocking(linearMid);
        robot.clawServo.setPosition(0);
    }
}