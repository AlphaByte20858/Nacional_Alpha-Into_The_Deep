package org.firstinspires.ftc.teamcode.opmode.teleops;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.interfaces.OptimizedOpMode;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotTelemetry;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ClawSubystem;
import org.firstinspires.ftc.teamcode.hardware.subsytems.DriveBaseSubsytem;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ElevatorSubsystem;

@TeleOp(name = "Astrid")
public class TeleopWithOOP extends OptimizedOpMode {

    RobotHardware Robot;
    ArmSubsystem Arm;
    ElevatorSubsystem Elevator;
    ClawSubystem Claw;
    DriveBaseSubsytem DriveBase;
    RobotTelemetry robotTelemetry;

    public void init(){

        Robot = new RobotHardware(this);
        Arm = new ArmSubsystem(Robot);
        Elevator = new ElevatorSubsystem(Robot);
        Claw = new ClawSubystem(Robot);
        DriveBase = new DriveBaseSubsytem(Robot);
        robotTelemetry = new RobotTelemetry(Robot);
        Arm.init();
        Elevator.init();
        Claw.init();
        DriveBase.init();
        robotTelemetry.init();
    }
    public void loop(){
        Arm.periodic();
        Elevator.periodic();
        Claw.periodic();
        DriveBase.periodic();
        keybinds();
        robotTelemetry.periodic();
    }

    public void keybinds(){
        gamepad1Keybinds();
        gamepad2Keybinds();
    }


    public void gamepad1Keybinds(){
        DriveBase.robotGyroMove(gamepad1.right_trigger-gamepad1.left_trigger,gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    public void gamepad2Keybinds() {
        Elevator.manualControl(gamepad2.right_trigger, gamepad2.left_trigger);
        Arm.manualControl(gamepad2.right_bumper, gamepad2.left_bumper);
        if (gamepad2.dpad_up){
            Elevator.setPidTarget(-1900);
        }

    }

}
