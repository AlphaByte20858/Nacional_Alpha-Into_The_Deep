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

@TeleOp
public class TeleopWithOOP extends OptimizedOpMode {

    RobotHardware Robot = new RobotHardware(this);
    ArmSubsystem Arm = new ArmSubsystem(Robot);
    ElevatorSubsystem Elevator = new ElevatorSubsystem(Robot);
    ClawSubystem Claw = new ClawSubystem(Robot);
    DriveBaseSubsytem DriveBase = new DriveBaseSubsytem(Robot);
    RobotTelemetry robotTelemetry = new RobotTelemetry(Robot);

    public void init(){
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
        if (gamepad1.a) {
            Actions.runBlocking(Arm.setHighPosition());
        }
    }

    public void gamepad2Keybinds() {

    }

}
