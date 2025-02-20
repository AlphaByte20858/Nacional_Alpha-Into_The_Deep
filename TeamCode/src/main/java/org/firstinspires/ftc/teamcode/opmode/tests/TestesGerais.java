package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ElevatorSubsystem;

@TeleOp
public class TestesGerais extends OpMode {
    RobotHardware Robot = new RobotHardware(this);
    ElevatorSubsystem Elevator;
    @Override
    public void init(){
        Elevator = new ElevatorSubsystem(Robot);
        Elevator.init();
    }

    @Override
    public void loop() {
        Elevator.manualControl(gamepad2.right_trigger, gamepad2.left_trigger);
    }
}