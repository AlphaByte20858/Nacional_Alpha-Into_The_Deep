package org.firstinspires.ftc.teamcode.hardware.subsytems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Constraints;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;

@Disabled
public class ArmSubsystem{
    private PIDController controller;
    Constraints.ArmConstraints constraints = new Constraints.ArmConstraints();
    private RobotHardware robot = new RobotHardware();
    static int target = 0;

    public void init(){
        controller = new PIDController(constraints.kp, constraints.ki, constraints.kd);

        robot.Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void periodic() {

    }
    public double setTarget(double targetVal){
        controller.setPID(constraints.kp, constraints.ki, constraints.kd);
        int braçoPos = robot.Arm.getCurrentPosition();
        double pid = controller.calculate(braçoPos, targetVal);
        double ff = Math.cos(Math.toRadians(targetVal / constraints.ticksInDegree)) * constraints.kf;

        double força = pid + ff;

        return (força);
    }

    public Action setHighPosition(){
        return new InstantAction(() -> {
            setTarget(constraints.highPosition);
        });
    }
}