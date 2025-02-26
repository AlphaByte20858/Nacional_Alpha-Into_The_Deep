package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        robot.clawServo.setPosition(0.5);
//        elevador.pidTarget(0);
        arm.setPidTarget(0);
        robot.wristServo.setPosition(0);
        waitForStart();


        arm.periodic();
        elevador.periodic();
        Actions.runBlocking(new SequentialAction(armHigh, splinei,linearHigh));
        robot.clawServo.setPosition(0);
        sleep(200);
        Actions.runBlocking(new SequentialAction(linearLow, splineii,
                ajeita,
                samplei,
                armLow,
                new InstantAction(() -> {robot.clawServo.setPosition(0.5); sleep(300);}
        ),
                armHigh));
        //segundo sample
        sleep(300);
        Actions.runBlocking(new SequentialAction(plusOne,
                new InstantAction(() -> {
                    robot.wristServo.setPosition(0.7);
                }),
                linearHigh));
        sleep(300);
        robot.clawServo.setPosition(0);
        Actions.runBlocking(new SequentialAction(new ParallelAction(
                get2,
                linearLow
        )));
        //terceiro sample
        tempo.reset();
        Actions.runBlocking(armLow);
        //garra ang 6 pos 0.64
        robot.clawServo.setPosition(0.5);
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