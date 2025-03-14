package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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


        Action get2, plusOne, plusTwo, splinei, ajeita, samplei; //actions trajetória
        Action linearHigh, linearLow, armLow;

        arm.init();
        elevador.init();

        arm.periodic();
        elevador.periodic();

        linearHigh = elevador.setHighPosition();
        linearLow = elevador.setLowPosition();
        armLow = arm.setLowPosition();

        splinei = peixinho.actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(30, 10, Math.toRadians(0)), Math.toRadians(0))
                .build();

        ajeita = peixinho.actionBuilder(new Pose2d(20, -34, Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(14, -10, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(50, -24, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        samplei = peixinho.actionBuilder(new Pose2d(50, -27, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(11, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(0, -22, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        plusOne = peixinho.actionBuilder(new Pose2d(12, -30, Math.toRadians(-180)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(30, 13, Math.toRadians(0)), Math.toRadians(0))
                .build();

        get2 = peixinho.actionBuilder(new Pose2d(20, 14, Math.toRadians(0)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(0, -15, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        plusTwo = peixinho.actionBuilder(new Pose2d(0, -15, Math.toRadians(-90)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(29, 18, Math.toRadians(0)), Math.toRadians(0))
                .build();

        robot.clawServo.setPosition(0.5);
//        elevador.pidTarget(0);
        arm.setPidTarget(0);
        robot.wristServo.setPosition(0);
        waitForStart();
        robot.arm.setPower(0.5);
        Actions.runBlocking(new SequentialAction( splinei, linearHigh));
        robot.clawServo.setPosition(0);
        sleep(200);
        Actions.runBlocking(new SequentialAction(linearLow,
                ajeita,
                samplei,
                armLow,
                new SleepAction(0.2),
                new InstantAction(() -> {robot.clawServo.setPosition(0.5);}),
                new SleepAction(0.2)
                ));
        robot.arm.setPower(0.4);

        Actions.runBlocking(new SequentialAction(
                new InstantAction(() -> {robot.wristServo.setPosition(0.67);}),
                plusOne,
                linearHigh
        ));

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
        sleep(400);
        robot.clawServo.setPosition(0.5);
        sleep(200);
        robot.wristServo.setPosition(0);
        robot.arm.setPower(0.4);
        Actions.runBlocking(new SequentialAction(plusTwo));
        robot.wristServo.setPosition(0);
        tempo.reset();
        Actions.runBlocking(linearHigh);
        robot.clawServo.setPosition(0);
    }
}