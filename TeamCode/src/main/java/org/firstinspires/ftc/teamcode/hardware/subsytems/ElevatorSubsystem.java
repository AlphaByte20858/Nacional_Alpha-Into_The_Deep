package org.firstinspires.ftc.teamcode.hardware.subsytems;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;

@Config
@TeleOp(name = "ElevatorPID")
public class ElevatorSubsystem implements SubsystemBase {
    /// definindo os motores do Elevador ///
    RobotHardware Robot;
    int encoderPosition;
    double pidPower = 0;
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;

    public static int target = 0;

    public ElevatorSubsystem(RobotHardware robot){
        this.Robot = robot;
    }
    public void init() {
        controller = new PIDController(p,i,d);
        /// variáveis para o PID


    }
    public void periodic(){
        encoderPosition = -((Robot.LSi.getCurrentPosition() + Robot.LSii.getCurrentPosition())/10);
    }

    public void manualControl(float upButton, float downButton) {
        Robot.LSi.setPower(upButton - downButton);
        Robot.LSii.setPower(upButton - downButton);
    }

    public void pidManualControl(float upButton, float downButton){
        double position;
        int lastPosition = 0;
        Robot.LSi.setPower(upButton - downButton * 0.7);
        Robot.LSii.setPower(upButton - downButton * 0.7);
        encoderPosition = (Robot.LSi.getCurrentPosition() + Robot.LSii.getCurrentPosition());
        if (Robot.LSi.getPower() == 0 && Robot.LSii.getPower() == 0){
            pidTarget(lastPosition);
        }
        else{
            lastPosition = encoderPosition;
        }
    }
    public void pidTarget(int target){
        encoderPosition = (Robot.LSi.getCurrentPosition() + Robot.LSii.getCurrentPosition());
        pidPower = controller.calculate(encoderPosition,target);
        Robot.LSi.setPower(pidPower);
        Robot.LSii.setPower(pidPower);
    }

}
