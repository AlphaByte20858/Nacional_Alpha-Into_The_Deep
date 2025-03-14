package org.firstinspires.ftc.teamcode.hardware.subsytems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Constraints;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;

@Config
public class ArmSubsystem implements SubsystemBase {

    private RobotHardware Robot;

    private PIDController controller;
    Constraints.ArmConstraints consts = new Constraints.ArmConstraints();
    public int target;
    DcMotorEx arm, armEncoder;
ElapsedTime timer = new ElapsedTime();
    int armPosition;
    double pid,ff;
    double error;
    public ArmSubsystem(RobotHardware robot){
        this.Robot = robot;
    }

    public void init(){
        controller = new PIDController(consts.kp, consts.ki, consts.kd);

    }

    public void periodic() {
        armPosition = Robot.arm.getCurrentPosition();
    }
    public void manualControl(boolean UpBotton, boolean DownBotton){
        if (UpBotton){
            Robot.arm.setPower(0.14);
        }
        else if (DownBotton){
            Robot.arm.setPower(-0.14);
        }
        else {
            Robot.arm.setPower(0);
        }
    }

    public void setPidTarget(int targetVal){
        this.target = targetVal;
        controller.setPID(consts.kp, consts.ki, consts.kd);
        int positionArm = Robot.armEncoder.getCurrentPosition();
        double pid = controller.calculate(positionArm, targetVal);
        double ff = Math.cos(Math.toRadians(targetVal / consts.ticksInDegree)) * consts.kf;

        double power = pid + ff;
        Robot.arm.setPower(power);
    }

    public void setStop(){

        ff = Math.cos(Math.toRadians(target / consts.ticksInDegree)) * consts.kf;


        double power = ff;

        Robot.arm.setPower(power);

    }
    public double getEncoderValue(){
        return Robot.armEncoder.getCurrentPosition();
    };
    public double getPowerValue(){
        return Robot.arm.getPower();
    }

    public Action setHighPosition(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                error = getEncoderValue() - consts.highPosition;
                if ((Math.abs(error) > consts.errorMargin)){
                    setPidTarget(consts.highPosition);
                    telemetryPacket.addLine("Está ");
                }
                else{
                    setStop();
                    return false;
                }
                return true;
            }
        };
    }
    public Action setHighPositionI(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                error = getEncoderValue() - consts.highPositionI;
                if ((Math.abs(error) > consts.errorMargin)){
                    setPidTarget(consts.highPositionI);
                    telemetryPacket.addLine("Está ");
                }
                else{
                    setStop();
                    return false;
                }
                return true;
            }
        };
    }
    public Action setHighPositionII(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                error = getEncoderValue() - consts.highPositionII;
                if ((Math.abs(error) > consts.errorMargin)){
                    setPidTarget(consts.highPositionII);
                    telemetryPacket.addLine("Está ");
                }
                else{
                    setStop();
                    return false;
                }
                return true;
            }
        };
    }
    public Action setInitialPos(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Robot.arm.setPower(0.5);
                return false;
            }
        };
    }

    public Action setZeroPos(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Robot.arm.setPower(0);
                return false;
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
    public Action setMidPosition(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                error = getEncoderValue() - consts.midPosition;
                if (Math.abs(error) > consts.errorMargin){
                    setPidTarget(consts.midPosition);
                }
                else{
                    return false;
                }
                return true;
            }
        };
    }
    public Action getSample(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                error = getEncoderValue() - consts.sampleGet;
                if (Math.abs(error) > consts.errorMargin){
                    setPidTarget(consts.sampleGet);
                    telemetryPacket.addLine("Position" + getEncoderValue());
                }
                else{
                    setStop();
                    return false;
                }
                return true;
            }
        };
    }
    public Action putSample(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                error = getEncoderValue() - consts.samplePut;
                if (Math.abs(error) > consts.errorMargin){
                    setPidTarget(consts.samplePut);
                    telemetryPacket.addLine("Position" + getEncoderValue());
                }
                else{
                    setStop();
                    return false;
                }
                return true;
            }
        };
    }

    public Action clipChamber(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                error = getEncoderValue() - consts.clipPosition;
                if (Math.abs(error) > consts.errorMargin){
                    setPidTarget(consts.clipPosition);
                    telemetryPacket.addLine("Position" + getEncoderValue());
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