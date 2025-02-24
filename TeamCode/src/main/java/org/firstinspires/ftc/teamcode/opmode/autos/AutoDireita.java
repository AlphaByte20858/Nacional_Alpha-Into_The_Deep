package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
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

import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous (name = "AutoOficial", group = "LinearOpMode")
public class AutoDireita extends LinearOpMode {
    DcMotorEx MEF, MET, MDF, MDT, LSi, LSii, braço;
    Servo garra, yawC;
    IMU imu;


    public void runOpMode(){

        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        braço = hardwareMap.get(DcMotorEx.class, "braço");
        LSi = hardwareMap.get(DcMotorEx.class, "LSi");
        LSii = hardwareMap.get(DcMotorEx.class, "LSii");
        yawC = hardwareMap.get(Servo.class, "yawC");
        garra = hardwareMap.get(Servo.class, "garra");

        MDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MDF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MDT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MET.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MEF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LSii.setDirection(DcMotorSimple.Direction.REVERSE);
        braço.setDirection(DcMotorSimple.Direction.REVERSE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        MecanumDrive peixinho = new MecanumDrive(hardwareMap, new Pose2d(0,0, Math.toRadians(0)));

        Action get2,plusOne, plusTwo, splinei, splineii, ajeita, samplei;

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

        braço.setPower(0.15);
        garra.setPosition(0.15);
        yawC.setPosition(0);
        waitForStart();
        lineares(0.42);
        Actions.runBlocking(new SequentialAction(
                splinei
        ));
        lineares(0.87);
        sleep(500);
        garra.setPosition(0);
        lineares(0);
        Actions.runBlocking(new SequentialAction(
                splineii,
                ajeita
                ,samplei
        ));
        //segundo sample
        braço.setPower(-0.45);
        sleep(635);
        garra.setPosition(0.23);
        sleep(100);
        lineares(0.35);
        braço.setPower(0.3);
        yawC.setPosition(0.64);
        Actions.runBlocking(
                plusOne
        );
        lineares(0.87);
        sleep(500);
        garra.setPosition(0);
        lineares(0);
        //terceiro sample
        Actions.runBlocking(get2);
        lineares(-0.26);
        braço.setPower(-0.45);
        sleep(635);
        //garra ang 6 pos 0.64
        garra.setPosition(0.23);
        sleep(100);
        yawC.setPosition(0);
        lineares(0.32);
        braço.setPower(0.3);
        Actions.runBlocking(plusTwo);
        yawC.setPosition(0.64);
        lineares(0.87);
        sleep(500);
    }

    public void lineares(double valorMotor){
        LSi.setPower(valorMotor);
        LSii.setPower(valorMotor);
    }
}