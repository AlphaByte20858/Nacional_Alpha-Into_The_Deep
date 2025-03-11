package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.interfaces.OptimizedAutonomous;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous (name = "Soluço otimizado", group = "LinearOpMode")
public class OptimizedAutoDireita extends OptimizedAutonomous {

    RobotHardware robot;
    ElevatorSubsystem elevador;
    ArmSubsystem arm;
    ElapsedTime tempo = new ElapsedTime();

    Action get2, plusOne, plusTwo, splinei, splineii, ajeita, samplei; //actions trajetória
    Action linearHigh, linearLow, armHigh, armLow;
    public void runOpMode() {
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


        Action get2, plusOne, plusTwo, splinei, splineii, ajeita, samplei; //actions trajetória
        Action linearHigh, linearLow, armHigh, armLow;

        arm.init();
        elevador.init();

        arm.periodic();
        elevador.periodic();

        linearHigh = elevador.setHighPosition();
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
                .splineToLinearHeading(new Pose2d(29, 13, Math.toRadians(0)), Math.toRadians(0))
                .build();

        get2 = peixinho.actionBuilder(new Pose2d(20, 14, Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(0, -15, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        plusTwo = peixinho.actionBuilder(new Pose2d(0, -15, Math.toRadians(-90)))
                .splineToLinearHeading(new Pose2d(29, 18, Math.toRadians(0)), Math.toRadians(0))
                .build();

        robot.clawServo.setPosition(0.5);
//        elevador.pidTarget(0);
        arm.setPidTarget(0);
        robot.wristServo.setPosition(0);
    }
    public void run(){
        Actions.runBlocking(new SequentialAction(armHigh, splinei, new ParallelAction(linearHigh, armHigh)));
        robot.clawServo.setPosition(0);
        sleep(200);
        Actions.runBlocking(new SequentialAction(linearLow, splineii,
                ajeita,
                samplei,
                armLow,
                new InstantAction(() -> {sleep(200);}),
                new InstantAction(() -> {robot.clawServo.setPosition(0.4);}),
                new InstantAction(() -> {sleep(200);}),
                armHigh
        ));

        Actions.runBlocking(new SequentialAction(armHigh,
                new InstantAction(() -> {robot.wristServo.setPosition(0.7);}),
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
        Actions.runBlocking(new SequentialAction(new ParallelAction(plusTwo,
                armHigh)));
        robot.wristServo.setPosition(0);
        tempo.reset();
        Actions.runBlocking(linearHigh);
        robot.clawServo.setPosition(0);
    }
}