package org.firstinspires.ftc.teamcode.hardware.subsytems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;

@Config
public class ElevatorSubsystem implements SubsystemBase {
    /// definindo os motores do Elevador ///
    RobotHardware Robot;
    int encoderPosition;
    private PIDController controller;
    public static double p = 0.045, i = 0, d = 0.0002;
    public static double f = 0.15;
    public static int target;

    private final double ticksInDegree = 537.7/ 360;
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
        controller.setPID(p, i, d);
        int posElev = Robot.LSi.getCurrentPosition();
        double pid = controller.calculate(posElev, target);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;

        double power = pid + ff;

        Robot.LSi.setPower(power);
        Robot.LSii.setPower(power);
    }
}
