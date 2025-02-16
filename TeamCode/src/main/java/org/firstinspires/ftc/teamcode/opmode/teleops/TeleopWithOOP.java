package org.firstinspires.ftc.teamcode.opmode.teleops;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotTelemetry;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ClawSubystem;
import org.firstinspires.ftc.teamcode.hardware.subsytems.DriveBaseSubsytem;
import org.firstinspires.ftc.teamcode.hardware.subsytems.ElevatorSubsystem;

public class TeleopWithOOP extends OpMode {

    RobotHardware Robot = new RobotHardware();
    ArmSubsystem Arm = new ArmSubsystem();
    ElevatorSubsystem Elevator = new ElevatorSubsystem();
    ClawSubystem Claw = new ClawSubystem();
    DriveBaseSubsytem DriveBase = new DriveBaseSubsytem();
    RobotTelemetry robotTelemetry = new RobotTelemetry();

    public void init(){
        Robot.init(hardwareMap);
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
        if (gamepad1.a){
            Actions.runBlocking(Arm.setHighPosition());
        }
    }
    public void gamepad2Keybinds(){

    }

}
