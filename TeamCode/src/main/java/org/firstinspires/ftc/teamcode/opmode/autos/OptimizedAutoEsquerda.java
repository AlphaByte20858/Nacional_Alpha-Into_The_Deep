package org.firstinspires.ftc.teamcode.opmode.autos;

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

import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.interfaces.OptimizedAutonomous;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "Banguela otimizado")
public class OptimizedAutoEsquerda extends OptimizedAutonomous {
    Action basket1, basket2, basket3, sample1, sample2, sample3, basket4;
    Action linearBasket, linearLow, armGet, armHigh, armPut;
    RobotHardware robot;
    ElevatorSubsystem elevador;
    ArmSubsystem arm;
    public void runOpMode(){
        runInit();
        waitForStart();
        run();
    }
    public void runInit(){

        robot = new RobotHardware(this);
        elevador = new ElevatorSubsystem(robot);
        arm = new ArmSubsystem(robot);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MecanumDrive peixinho = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));


        linearBasket = elevador.setBasketPosition();
        linearLow = elevador.setLowPosition();
        armHigh = arm.setHighPosition();
        armGet = arm.getSample();
        armPut = arm.putSample();

        basket1 = peixinho.actionBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(5, 40, Math.toRadians(135)), Math.toRadians(135))
                .build();

        sample1 = peixinho.actionBuilder(new Pose2d(12, 45, Math.toRadians(135)))
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(16, 34, Math.toRadians(0)), Math.toRadians(0))
                .build();

        basket2 = peixinho.actionBuilder(new Pose2d(16, 29, Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(7, 42, Math.toRadians(145)), Math.toRadians(145))
                .build();

        sample2 = peixinho.actionBuilder(new Pose2d(7, 42, Math.toRadians(145)))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(16, 44, Math.toRadians(360)), Math.toRadians(360))
                .build();

        basket3 = peixinho.actionBuilder(new Pose2d(12, 37, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(8, 44, Math.toRadians(145)), Math.toRadians(145))
                .build();

        sample3 = peixinho.actionBuilder(new Pose2d(12, 33, Math.toRadians(145)))
                .splineToLinearHeading(new Pose2d(37, 35, Math.toRadians(90)), Math.toRadians(90))
                .build();

        basket4 = peixinho.actionBuilder(new Pose2d(28, 28, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(8,44, Math.toRadians(145)), Math.toRadians(145))
                .build();

        arm.init();
        elevador.init();
        robot.clawServo.setPosition(0.6);
    }
    public void run(){

        arm.periodic();
        elevador.periodic();
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        basket1, linearBasket),
                armPut));
        robot.clawServo.setPosition(0);
        sleep(400);
        Actions.runBlocking(new SequentialAction(
                armHigh,

                new ParallelAction(sample1, armHigh),
                linearLow));

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(linearLow, armGet)));
        robot.clawServo.setPosition(0.6);
        sleep(400);
        Actions.runBlocking(new SequentialAction(armHigh,
                new ParallelAction(basket2, linearBasket, armHigh),
                armPut));
        robot.clawServo.setPosition(0);
        sleep(300);

        Actions.runBlocking(new SequentialAction(armHigh,
                new ParallelAction(sample2, linearLow),
                armGet,
                new InstantAction(() -> {robot.clawServo.setPosition(0.6);}),
                new SleepAction(0.4),
                armHigh));
        Actions.runBlocking(new SequentialAction(new ParallelAction(basket3, linearBasket),
                armPut,
                new InstantAction(() -> {robot.clawServo.setPosition(0);})));
        sleep(200);


        Actions.runBlocking(new SequentialAction(
                armHigh,
                new ParallelAction(sample3, armHigh),
                new InstantAction(() -> {robot.wristServo.setPosition(0.37);}),
                linearLow));

        Actions.runBlocking(new SequentialAction(new ParallelAction(linearLow, armGet)));
        sleep(500);
        robot.clawServo.setPosition(0.6);
        sleep(400);
        robot.wristServo.setPosition(0);
        Actions.runBlocking(new SequentialAction(new ParallelAction(basket4, linearBasket, armHigh),
                armPut));
        robot.clawServo.setPosition(0);
    }
}