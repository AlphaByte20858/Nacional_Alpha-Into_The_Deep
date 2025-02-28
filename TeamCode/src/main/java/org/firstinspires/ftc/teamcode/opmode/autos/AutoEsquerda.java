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
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class AutoEsquerda extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(this);
        ElevatorSubsystem elevador = new ElevatorSubsystem(robot);
        ArmSubsystem arm = new ArmSubsystem(robot);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MecanumDrive peixinho = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        Action basket1, basket2, basket3, sample1, sample2, sample3;
        Action linearBasket, linearLow, armGet, armHigh, armPut;

        linearBasket = elevador.setBasketPosition();
        linearLow = elevador.setLowPosition();
        armHigh = arm.setHighPosition();
        armGet = arm.getSample();
        armPut = arm.putSample();
        basket1 = peixinho.actionBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(4, 41, Math.toRadians(135)), Math.toRadians(135))
                .build();

        sample1 = peixinho.actionBuilder(new Pose2d(12, 45, Math.toRadians(135)))
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(16, 35, Math.toRadians(0)), Math.toRadians(0))
                .build();

        basket2 = peixinho.actionBuilder(new Pose2d(16, 34, Math.toRadians(0)))
                .setTangent(Math.toRadians(145))
                .splineToLinearHeading(new Pose2d(6, 44, Math.toRadians(145)), Math.toRadians(145))
                .build();

        sample2 = peixinho.actionBuilder(new Pose2d(16, 45, Math.toRadians(145)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(16, 42, Math.toRadians(0)), Math.toRadians(0))
                .build();

        basket3 = peixinho.actionBuilder(new Pose2d(16, 37, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(6, 44, Math.toRadians(145)), Math.toRadians(145))
                .build();


        arm.init();
        elevador.init();
        robot.clawServo.setPosition(0.4);
        waitForStart();

        robot.Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.periodic();
        elevador.periodic();
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                basket1, linearBasket),
                armPut
                ));
        sleep(100);
        robot.clawServo.setPosition(0);
        sleep(150);
        Actions.runBlocking(new SequentialAction(armHigh,
                new ParallelAction(sample1, linearLow),
                armGet));
        sleep(600);
        robot.clawServo.setPosition(0.4);
        Actions.runBlocking(new SequentialAction(new ParallelAction(basket2, linearBasket, armHigh),
                armPut));

        robot.clawServo.setPosition(0);
        sleep(300);
        Actions.runBlocking(new SequentialAction(armHigh,
                new ParallelAction(sample2, linearLow),
                armGet,
                new InstantAction(() -> {robot.clawServo.setPosition(0.4);}),
                new ParallelAction(armHigh, linearBasket)));
        sleep(400);

        Actions.runBlocking(new SequentialAction(new ParallelAction(basket3, linearBasket),
                armPut,
                new InstantAction(() -> {robot.clawServo.setPosition(0);}),
                armHigh));
    }
}