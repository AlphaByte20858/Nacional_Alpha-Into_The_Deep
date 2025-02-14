package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.Constraints.ElevatorConstraints;

@Config
@TeleOp(name = "ElevatorPID")
public class ElevatorSubsystem extends OpMode {
    /// definindo os motores do Elevador ///
    Robot_Hardware robot = new Robot_Hardware();
    int encoderPosition;
    double pidPower;
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;

    public static int target = 0;
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(p,i,d);
        /// variáveis para o PID


    }
    public void loop(){
        encoderPosition = -((robot.LSi.getCurrentPosition() + robot.LSii.getCurrentPosition())/10);
        robot.LSi.setPower(target);
        robot.LSii.setPower(target);
        pidPower = controller.calculate(encoderPosition,target);
        robot.LSi.setPower(pidPower);
        robot.LSii.setPower(pidPower);  
        telemetry.addData("posição", encoderPosition);
        telemetry.addData("target", target);
        telemetry.update();
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
