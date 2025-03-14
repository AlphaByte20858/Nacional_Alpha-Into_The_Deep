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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "AutoReserva", group = "LinearOpMode")
public class AutoReserva extends LinearOpMode {
    @Override
    public void runOpMode() {

        RobotHardware robot = new RobotHardware(this);
        ElevatorSubsystem elevador = new ElevatorSubsystem(robot);
        ArmSubsystem arm = new ArmSubsystem(robot);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MecanumDrive peixinho = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        Action basket1, basket2, basket3, sample1, sample2, sample3, basket4;
        Action linearBasket, linearLow, armGet, armHigh, armPut, armStop;

        linearBasket = elevador.setBasketPosition();
        linearLow = elevador.setLowPosition();
        armHigh = arm.setInitialPos();
        armStop = arm.setZeroPos();
        armGet = arm.getSample();
        armPut = arm.putSample();


        basket1 = peixinho.actionBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(4, 39, Math.toRadians(135)), Math.toRadians(135))
                .build();

        sample1 = peixinho.actionBuilder(new Pose2d(13, 45, Math.toRadians(135)))
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(18, 33, Math.toRadians(0)), Math.toRadians(0))
                .build();

        basket2 = peixinho.actionBuilder(new Pose2d(13, 29, Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(8, 42, Math.toRadians(145)), Math.toRadians(145))
                .build();

        sample2 = peixinho.actionBuilder(new Pose2d(8, 42, Math.toRadians(145)))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(18, 43, Math.toRadians(360)), Math.toRadians(360))
                .build();

        basket3 = peixinho.actionBuilder(new Pose2d(11, 37, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(6, 44, Math.toRadians(145)), Math.toRadians(145))
                .build();

        sample3 = peixinho.actionBuilder(new Pose2d(11, 33, Math.toRadians(145)))
                .splineToLinearHeading(new Pose2d(39, 36, Math.toRadians(90)), Math.toRadians(90))
                .build();

        basket4 = peixinho.actionBuilder(new Pose2d(29, 28, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(6,44, Math.toRadians(145)), Math.toRadians(145))
                .build();

        arm.init();
        elevador.init();
        robot.clawServo.setPosition(0.7);
        waitForStart();

        arm.periodic();
        elevador.periodic();
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        basket1, linearBasket, armHigh),
                armPut));
        robot.clawServo.setPosition(0);
        sleep(400);
        Actions.runBlocking(new SequentialAction(
                armHigh,
                sample1,
                linearLow));
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(linearLow, armGet)));
        robot.clawServo.setPosition(0.7);
        sleep(400);
        Actions.runBlocking(new SequentialAction(armHigh,
                new ParallelAction(basket2, linearBasket),
                armStop,
                armPut));
        robot.clawServo.setPosition(0);
        sleep(300);

        Actions.runBlocking(new SequentialAction(armHigh,
                new ParallelAction(sample2, linearLow),
                armGet,
                new InstantAction(() -> {
                    robot.clawServo.setPosition(0.7);
                }),
                new SleepAction(0.4),
                armHigh,
                new SleepAction(0.5)));
        Actions.runBlocking(new SequentialAction(new ParallelAction(basket3, linearBasket),
                armStop,
                armPut,
                new InstantAction(() -> {
                    robot.clawServo.setPosition(0);
                })));
        sleep(200);


        Actions.runBlocking(new SequentialAction(
                armHigh,
                sample3,
                new InstantAction(() -> {
                    robot.wristServo.setPosition(0.37);
                }),
                linearLow));

        Actions.runBlocking(new SequentialAction(new ParallelAction(linearLow, armGet)));
        sleep(500);
        robot.clawServo.setPosition(0.7);
        sleep(400);
        robot.wristServo.setPosition(0);
        Actions.runBlocking(new SequentialAction(new ParallelAction(basket4, linearBasket, armHigh),
                armStop,
                armPut));
        robot.clawServo.setPosition(0);
    }
}