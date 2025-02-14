package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.robot.Robot;

@Config
@TeleOp (name = "Braço PIDF")
public class ArmPIDF extends OpMode {
    private PIDController controller;
    public static double p = 0.004, i = 0, d = 0.00001, f = 0.001;

    public static int target = 0;

    private final double ticks_in_degree = (28 * 5 * 4 * 3)/180;

    private Robot_Hardware robot = new Robot_Hardware();

    public void init(){
        robot.init(hardwareMap);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.braço = hardwareMap.get(DcMotorEx.class, "braço");

        robot.braço.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.braço.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        robot.braço.setPower((setTarget(target)));
        if (gamepad1.a){
            robot.braço.setPower(setTarget(-300));
        }
        else if (gamepad1.b){
            robot.braço.setPower(setTarget(0));
        }

        telemetry.addData("posição", robot.braço.getCurrentPosition());
        telemetry.addData("target", target);
        telemetry.update();
    }
    public double setTarget(double targetVal){
        controller.setPID(p, i, d);
        int braçoPos = robot.braço.getCurrentPosition();
        double pid = controller.calculate(braçoPos, targetVal);
        double ff = Math.cos(Math.toRadians(targetVal / ticks_in_degree)) * f;

        double força = pid + ff;

        return (força);
    }
}