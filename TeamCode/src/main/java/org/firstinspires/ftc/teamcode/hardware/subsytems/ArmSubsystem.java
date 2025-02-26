package org.firstinspires.ftc.teamcode.hardware.subsytems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Constraints;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;

@Config
public class ArmSubsystem implements SubsystemBase {

    private RobotHardware Robot;

    private PIDController controller;
    Constraints.ArmConstraints consts = new Constraints.ArmConstraints();
    static int target = 0;
    int armPosition;
    double error;
    public ArmSubsystem(RobotHardware robot){
        this.Robot = robot;
    }

    public void init(){
        controller = new PIDController(consts.kp, consts.ki, consts.kd);

        Robot.Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void periodic() {
        armPosition = Robot.Arm.getCurrentPosition();
    }
    public void manualControl(boolean UpBotton, boolean DownBotton){
        if (UpBotton){
            Robot.Arm.setPower(0.14);
        }
        else if (DownBotton){
            Robot.Arm.setPower(-0.14);
        }
        else {
            Robot.Arm.setPower(0);
        }
    }
    public void setPidTarget(int targetVal){
        controller.setPID(consts.kp, consts.ki, consts.kd);
        target = targetVal;
        double pid = controller.calculate(armPosition, targetVal);
        double ff = Math.cos(Math.toRadians(targetVal / consts.ticksInDegree)) * consts.kf;

        double power = pid + ff;
        Robot.Arm.setPower(power);
    }
    public void setStop(){

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
                    setStop();
                    return false;
                }
                return true;
            }
        };
    }
    public double getEncoderValue(){
        return Robot.Arm.getCurrentPosition();
    };
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
    public Action setLowPosition() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                error = getEncoderValue() - consts.lowPosition;
                if (Math.abs(error) > consts.errorMargin) {
                    setPidTarget(consts.lowPosition);
                } else {
                    setStop();
                    return false;
                }
                return true;
            }
        };
    }
}