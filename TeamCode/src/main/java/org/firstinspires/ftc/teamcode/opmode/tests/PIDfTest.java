package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Constraints;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ElevatorSubsystem;

@Config
@TeleOp
public class PIDfTest extends OpMode {
    private PIDController controller;
    public static double p = 0.002, i = 0, d = 0.0001;
    public static double f =  0.001;
    public static int target;

    private final double ticksInDegree = (28 * 5*4*3) / 180;
    DcMotorEx arm;
    ElevatorSubsystem Elevator;
    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hardwareMap.get(DcMotorEx.class, "braço");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int posElev = arm.getCurrentPosition();
        double pid = controller.calculate(posElev, target);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;

        double power = pid + ff;

        arm.setPower(power);
//        lineares(power);

        telemetry.addData("pos", posElev);
        telemetry.addData("target", target);
        telemetry.update();

    }/*
    public void lineares(double valor){
        LSi.setPower(valor);
        LSii.setPower(valor);
    }*/
}