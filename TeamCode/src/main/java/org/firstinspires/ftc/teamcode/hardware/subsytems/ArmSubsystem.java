package org.firstinspires.ftc.teamcode.hardware.subsytems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Constraints;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;

@Disabled
public class ArmSubsystem implements SubsystemBase {

    private RobotHardware Robot;

    private PIDController controller;
    Constraints.ArmConstraints constraints = new Constraints.ArmConstraints();
    static int target = 0;
    int armPosition;

    public ArmSubsystem(RobotHardware robot){
        this.Robot = robot;
    }

    public void init(){
        controller = new PIDController(constraints.kp, constraints.ki, constraints.kd);

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
    public void setTarget(double targetVal){
        controller.setPID(constraints.kp, constraints.ki, constraints.kd);
        double pid = controller.calculate(armPosition, targetVal);
        double ff = Math.cos(Math.toRadians(targetVal / constraints.ticksInDegree)) * constraints.kf;

        double power = pid + ff;
        Robot.Arm.setPower(power);
    }

    public Action setHighPosition(){
        return new InstantAction(() -> {
            setTarget(constraints.highPosition);
        });
    }
    public Action setMidPosition(){
        return new InstantAction(() -> {
            setTarget(constraints.midPosition);
        });
    }
    public Action setLowPosition(){
        return new InstantAction(() -> {
            setTarget(constraints.lowPosition);
        });
    }
}