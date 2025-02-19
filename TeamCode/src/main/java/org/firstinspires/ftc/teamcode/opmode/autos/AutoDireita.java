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
    DcMotorEx MEF, MET, MDF, MDT, LSi, LSii;
    Servo garra, yawC;
    IMU imu;

    RobotHardware Robot = new RobotHardware(this);
    ArmSubsystem Arm = new ArmSubsystem(Robot);

    public void runOpMode(){


        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
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

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Arm.init();

        MecanumDrive peixinho = new MecanumDrive(hardwareMap, new Pose2d(0,0, Math.toRadians(0)));

        Action plusOne, splinei, splineii, ajeita, samplei;


        splinei = peixinho.actionBuilder(new Pose2d(0,0, Math.toRadians(0)))
                .splineTo(new Vector2d(28,34), Math.toRadians(0))
                .build();

        splineii = peixinho.actionBuilder(new Pose2d(20,34, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(12, 8), Math.toRadians(0))
                .build();

        ajeita = peixinho.actionBuilder(new Pose2d(24,3, Math.toRadians(-90)))
                .splineTo(new Vector2d( 46, -6), Math.toRadians(-90))
                /*.strafeTo(new Vector2d(46, 3))
                .strafeTo(new Vector2d(46, -8))
                .strafeToConstantHeading(new Vector2d(10, -8))
                .splineToConstantHeading(new Vector2d(46, -12), Math.toRadians(-90))*/
                .build();


        /*samplei = peixinho.actionBuilder(new Pose2d(46, -12, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(10, -17), Math.toRadians(-90))
                .turnTo(Math.toRadians(90))
                .build();
*/
        plusOne = peixinho.actionBuilder(new Pose2d(10, -17, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(28, 34, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Arm.setTarget(15);
        garra.setPosition(0.15);
        yawC.setPosition(0);
        waitForStart();
        lineares(0.45);
        Actions.runBlocking(new SequentialAction(
                splinei
        ));
        lineares(0.8);
        sleep(500);
        lineares(0);
        sleep(400);
        Actions.runBlocking(new SequentialAction(
                splineii,
                ajeita
                //,samplei
        ));
        Arm.setTarget(260);
        garra.setPosition(0.15);
        lineares(0.25);
        Arm.setTarget(20);
        Actions.runBlocking(
                plusOne
        );
        lineares(0.8);
        sleep(500);
        garra.setPosition(0);
        lineares(0);
        sleep(400);
    }

    public void lineares(double valorMotor){
        LSi.setPower(valorMotor);
        LSii.setPower(valorMotor);
    }
}