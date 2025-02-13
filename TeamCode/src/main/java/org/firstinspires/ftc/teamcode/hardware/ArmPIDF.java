package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp (name = "Braço PIDF")
public class ArmPIDF extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0, f = 0;

    public static int target = 0;

    private final double ticks_in_degree = (1120 * 5 * 4 * 3)/360;

    private DcMotorEx braço;

    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        braço = hardwareMap.get(DcMotorEx.class, "braço");

        braço.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        braço.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int braçoPos = braço.getCurrentPosition();
        double pid = controller.calculate(braçoPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double força = pid + ff;

        braço.setPower(força);

        telemetry.addData("posição", braçoPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}