package org.firstinspires.ftc.teamcode.opmode.testes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class TestesGerais extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotorEx braço;

        braço = hardwareMap.get(DcMotorEx.class, "braço");
        double power = 0;
        braço.setPower(power);

        telemetry.addData("força", power);
    }

}