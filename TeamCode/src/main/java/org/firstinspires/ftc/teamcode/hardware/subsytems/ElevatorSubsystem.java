package org.firstinspires.ftc.teamcode.hardware.subsytems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.hardware.Constraints;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;

@Config
public class ElevatorSubsystem implements SubsystemBase {
    /// definindo os motores do Elevador ///
    RobotHardware Robot;
    int encoderPosition;
    private PIDController controller;
    public static double p = 0.035, i = 0, d = 0.0002;
    public static double f = 0.15;
    public int target;
    double pid,ff;
    double error;
    Constraints.ElevatorConstraints consts = new Constraints.ElevatorConstraints();

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
            setPidTarget(lastPosition);
        }
        else{
            lastPosition = encoderPosition;
        }
    }
    public void setPidTarget(int valueTarget){
        this.target = valueTarget;
        controller.setPID(p, i, d);
        int elevatorPosition = (Robot.LSi.getCurrentPosition() + Robot.LSii.getCurrentPosition())/2;
        pid = controller.calculate(elevatorPosition, valueTarget);
        ff = Math.cos(Math.toRadians(this.target / ticksInDegree)) * f;

        double power = pid + ff;

        setPower(power);
    }
    public void setStop(){


        ff = Math.cos(Math.toRadians(target /   ticksInDegree)) * f;


        double power = ff;


        setPower(power);

    }

    public void setPower(double power){
        Robot.LSi.setPower(power);
        Robot.LSii.setPower(power);
    }

    public double getEncoderValue(){
        return Robot.LSi.getCurrentPosition();
    }

    public Action setHighPosition(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                error = getEncoderValue() - consts.highPosition;
                if (Math.abs(error) > consts.errorMargin){
                    setPidTarget(consts.highPosition);
                }
                else{
                    return false;
                }
                return true;

            }
        };
    }
    public Action setMidPosition(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                error = getEncoderValue() - consts.midPosition;
                if (Math.abs(error) > consts.errorMargin){
                    setPidTarget(consts.midPosition);
                }
                else{
                    setStop();
                    return false;
                }
                return true;
            }
        };
    }
    public Action setLowPosition(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                error = getEncoderValue() - consts.lowPosition;
                if (Math.abs(error) > consts.errorMargin){
                    setPidTarget(consts.lowPosition);
                }
                else{
                    Robot.LSii.setPower(0);
                    Robot.LSi.setPower(0);
                    return false;
                }
                return true;
            }
        };
    }
    public Action setPitLowPosition(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                error = getEncoderValue() - 500;
                if (Math.abs(error) > consts.errorMargin){
                    setPidTarget(consts.lowPosition);
                }
                else{
                    Robot.LSii.setPower(0);
                    Robot.LSi.setPower(0);
                    return false;
                }
                return true;
            }
        };
    }
    public Action setBasketPosition(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                error = getEncoderValue() - consts.basketHigh;
                if (Math.abs(error) > consts.errorMargin){
                    setPidTarget(consts.basketHigh);
                }
                else{
                    setStop();
                    return false;
                }
                return true;

            }
        };
    }
}