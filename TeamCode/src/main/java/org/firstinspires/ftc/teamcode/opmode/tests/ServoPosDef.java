package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ServoPos")
public class ServoPosDef extends OpMode {
    Servo servo;
    double pos;
    ElapsedTime timing = new ElapsedTime();
    public void init() {
        //TODO: só alterar de teste para teste o deviceName
        servo = hardwareMap.get(Servo.class, "yawC");
        servo.setPosition(0);
        pos = 0;
    }

    public void loop() {
        if (timing.seconds() > 0.3) {
            if (gamepad1.right_bumper) {
                pos = pos + 0.1;
                servo.setPosition(pos);
            }

            else if (gamepad1.left_bumper) {
                pos = pos - 0.1;
                servo.setPosition(pos);
            }

            else if (gamepad1.a){
                pos = pos + 0.01;
                servo.setPosition(pos);
            }
            else if (gamepad1.b){
                pos = pos - 0.01;
                servo.setPosition(pos);
            }
        timing.reset();
        }
        telemetry.addData("posição servo:", servo.getPosition());
    }
}
