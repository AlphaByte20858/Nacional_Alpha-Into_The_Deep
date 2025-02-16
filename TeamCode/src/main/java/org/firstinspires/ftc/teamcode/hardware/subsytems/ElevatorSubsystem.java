package org.firstinspires.ftc.teamcode.hardware.subsytems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;

@Config
@TeleOp(name = "ElevatorPID")
public class ElevatorSubsystem {
    /// definindo os motores do Elevador ///
    RobotHardware robot = new RobotHardware();
    int encoderPosition;
    double pidPower = 0;
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;

    public static int target = 0;

    public void init() {
        controller = new PIDController(p,i,d);
        /// variáveis para o PID


    }
    public void periodic(){
        encoderPosition = -((robot.LSi.getCurrentPosition() + robot.LSii.getCurrentPosition())/10);
        robot.LSi.setPower(target);
        robot.LSii.setPower(target);
        pidPower = controller.calculate(encoderPosition,target);
        robot.LSi.setPower(pidPower);
        robot.LSii.setPower(pidPower);
    }

    public void manualControl(float upButton, float downButton) {
        robot.LSi.setPower(upButton - downButton);
        robot.LSii.setPower(upButton - downButton);
    }

    public void pidManualControl(float upButton, float downButton){
        double position;
        int lastPosition = 0;
        robot.LSi.setPower(upButton - downButton * 0.7);
        robot.LSii.setPower(upButton - downButton * 0.7);
        encoderPosition = (robot.LSi.getCurrentPosition() + robot.LSii.getCurrentPosition());
        if (robot.LSi.getPower() == 0 && robot.LSii.getPower() == 0){
            pidTarget(lastPosition);
        }
        else{
            lastPosition = encoderPosition;
        }
    }
    public void pidTarget(int target){
        encoderPosition = (robot.LSi.getCurrentPosition() + robot.LSii.getCurrentPosition());
        pidPower = controller.calculate(encoderPosition,target);
        robot.LSi.setPower(pidPower);
        robot.LSii.setPower(pidPower);
    }

}
