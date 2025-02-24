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
        Action linearHigh, linearMid, linearLow, armLow, armHigh;

        linearHigh = new InstantAction(() -> {
            elevador.pidTarget(-1900);
        });
        linearMid = new InstantAction(() ->{
            {
                elevador.pidTarget(-1000);
            }});
        linearLow = new InstantAction(() ->{
            elevador.pidTarget(0);
        });
        armHigh = new InstantAction(() -> {
            arm.setTarget(0);
        });
        armLow = new InstantAction(() -> {
            arm.setTarget(-430);
        });
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

        samplei = peixinho.actionBuilder(new Pose2d(48, -27, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(2, -31, Math.toRadians(-90)), Math.toRadians(0))
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
        elevador.pidTarget(0);
        arm.setTarget(0);
        robot.wristServo.setPosition(0);
        waitForStart();
        arm.periodic();
        elevador.periodic();
        Actions.runBlocking(new ParallelAction(splinei, linearMid)
        );
        sleep(200);
        robot.clawServo.setPosition(0);
        Actions.runBlocking(new SequentialAction(
                splineii,
                ajeita,
                samplei
        ));
        //segundo sample
        tempo.reset();
        while (tempo.seconds() < 1) {
            arm.setTarget(-430);
        }
        robot.clawServo.setPosition(0.23);
        sleep(100);
        Actions.runBlocking(new ParallelAction(plusOne,
                new InstantAction(() -> {
                    arm.setTarget(-20);
                }),
                linearMid,
                new InstantAction(() -> {
                    robot.wristServo.setPosition(0.7);
                })
        ));
        tempo.reset();
        while (tempo.seconds() < 2) {
            arm.setTarget(-20);
            elevador.pidTarget(1000);
            robot.wristServo.setPosition(0.7);
        }
        robot.clawServo.setPosition(0);

        sleep(200);
        tempo.reset();
        while (tempo.seconds() < 2) {
            elevador.pidTarget(-1600);
        }
        robot.clawServo.setPosition(0);
        //terceiro sample
        Actions.runBlocking(new SequentialAction(new ParallelAction(
                get2,
                new InstantAction(() -> elevador.pidTarget(0))
        )));
        tempo.reset();
        while (tempo.seconds() < 1) {
            arm.setTarget(-430);
        }
        //garra ang 6 pos 0.64
        robot.clawServo.setPosition(0.23);
        sleep(100);
        robot.wristServo.setPosition(0);
        Actions.runBlocking((new ParallelAction(plusTwo,
                new InstantAction(() -> arm.setTarget(-20)),
                new InstantAction(() -> elevador.pidTarget(-1000)),
                new InstantAction(() -> robot.wristServo.setPosition(0.7)))));
        tempo.reset();
        while (tempo.seconds() < 1) {
            elevador.pidTarget(-2800);
        }
        robot.clawServo.setPosition(0);
    }
}