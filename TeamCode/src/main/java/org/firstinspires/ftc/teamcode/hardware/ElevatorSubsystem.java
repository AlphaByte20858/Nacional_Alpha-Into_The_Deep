package org.firstinspires.ftc.teamcode.hardware;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.Constraints.ElevatorConstraints;

@Disabled
public class ElevatorSubsystem extends SubsystemBase {
    /// definindo os motores do Elevador ///
    DcMotor rightMotor,leftMotor;
    ElevatorConstraints linear = new ElevatorConstraints();
    PIDController controller;
    int encoderPosition;
    double pidPower;


    /// criando o construtor do sistema de Elevator///
    public ElevatorSubsystem(){
        this.rightMotor = Robot_Hardware.getInstance().LSi;
        this.leftMotor = Robot_Hardware.getInstance().LSii;

        this.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /// variáveis para o PID
        controller = new PIDController(linear.kp,linear.ki,linear.kd);
        encoderPosition = Integer.parseInt(String.valueOf((rightMotor.getCurrentPosition() + leftMotor.getCurrentPosition())/2));
    }

    public void manualControl(float upButton, float downButton) {
        rightMotor.setPower(upButton - downButton);
        leftMotor.setPower(upButton - downButton);
    }

    public void pidManualControl(float upButton, float downButton){
        double position;
        int lastPosition = 0;
        rightMotor.setPower(upButton - downButton * 0.7);
        leftMotor.setPower(upButton - downButton * 0.7);
        encoderPosition = Integer.parseInt(String.valueOf((rightMotor.getCurrentPosition() + leftMotor.getCurrentPosition())/2));
        if (rightMotor.getPower() == 0 && leftMotor.getPower() == 0){
            pidTarget(lastPosition);
        }
        else{
            lastPosition = encoderPosition;
        }
    }
    public void pidTarget(int target){
        encoderPosition = Integer.parseInt(String.valueOf((rightMotor.getCurrentPosition() + leftMotor.getCurrentPosition())/2));
        pidPower = controller.calculate(encoderPosition,target);
        rightMotor.setPower(pidPower);
        leftMotor.setPower(pidPower);
    }

}
