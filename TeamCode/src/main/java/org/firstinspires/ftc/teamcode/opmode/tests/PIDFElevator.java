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


@Config
@TeleOp
public class PIDFElevator extends OpMode {
    private PIDController controller;


    public double kp = 0.045;
    public double ki = 0;
    public double kd = 0.0002;
    public double kf = 0.15;

    DcMotorEx LSi, LSii;
    public static int target;

    public double ticksInDegree = 537.7/ 360;
    @Override

    public void init(){
        controller = new PIDController(kp, ki, kd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LSi = hardwareMap.get(DcMotorEx.class, "LSi");
        LSii = hardwareMap.get(DcMotorEx.class, "LSii");

        LSi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSii.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LSii.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LSi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LSii.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        controller.setPID(kp, ki, kd);
        int posElev = (LSi.getCurrentPosition() + LSii.getCurrentPosition() / 10);
        double pid = controller.calculate(posElev, target);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * kf;

        double power = pid + ff;

        lineares(power);
    }
    public void lineares(double power){
        LSi.setPower(power);
        LSii.setPower(power);
    }
}