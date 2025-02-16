package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Tele")
public class MotorSample extends OpMode {
    DcMotorEx MDT, MDF, MET, MEF;
    ;
    double axial, lateral, yaw;

    public void init() {
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");

        MDF.setDirection(DcMotorSimple.Direction.REVERSE);
        MDT.setDirection(DcMotorSimple.Direction.REVERSE);
        MET.setDirection(DcMotorSimple.Direction.FORWARD);
        MEF.setDirection(DcMotorSimple.Direction.FORWARD);

        MDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        MDF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MDT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MET.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MEF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   }

    public void loop() {
        axial = (gamepad1.right_trigger - gamepad1.left_trigger) * 0.8;
        lateral = gamepad1.left_stick_x * 0.8;
        yaw = gamepad1.right_stick_x * 0.6;


        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw = Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);
        double motorEsquerdoFf = (axial + lateral + yaw / denominador);
        double motorDireitoFf = (axial - lateral - yaw / denominador);
        double motorEsquerdoTf = (axial - lateral + yaw / denominador);
        double motorDireitoTf = (axial + lateral - yaw / denominador);

        if (gamepad1.right_bumper) {
            MotorsPower(motorEsquerdoFf * 0.8, motorDireitoFf * 0.8, motorEsquerdoTf * 0.8, motorDireitoTf * 0.8);
        } else {
            MotorsPower(motorEsquerdoFf, motorDireitoFf, motorEsquerdoTf, motorDireitoTf);
        }

        telemetry.getCaptionValueSeparator();
        if (gamepad1.a) {
            MDF.setPower(1);
            telemetry.addLine("Direita Frente ligado!");
        }
        else if (gamepad1.b) {
            MDT.setPower(1);
            telemetry.addLine("Direita Tras ligado!");
        }
        else if (gamepad1.x) {
            MET.setPower(1);
            telemetry.addLine("Esquerda Tras ligado!");
        }
        else if (gamepad1.y) {
            MEF.setPower(1);
            telemetry.addLine("Esquerda Frente ligado!");
        }
        else {
            telemetry.addLine("nada ativo");
        }
    }

    public void MotorsPower(double p1, double p2, double p3, double p4) {
        MEF.setPower(p1);
        MDF.setPower(p2);
        MET.setPower(p3);
        MDT.setPower(p4);
    }
}