package org.firstinspires.ftc.teamcode.hardware.robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
public class RobotHardware {


    OpMode opMode;
    public DcMotorEx MDT, MDF, MET, MEF, LSi, LSii, Arm, armEncoder;
    double axial, lateral, yaw,angle;
    public Servo wristServo, clawServo; //Define o nome dos servos no sistema
    public IMU imu;


    public RobotHardware(OpMode opMode) {
        MET = opMode.hardwareMap.get(DcMotorEx.class, "MET");
        MDT = opMode.hardwareMap.get(DcMotorEx.class, "MDT");
        MDF = opMode.hardwareMap.get(DcMotorEx.class, "MDF");
        MEF = opMode.hardwareMap.get(DcMotorEx.class, "MEF");

        MDF.setDirection(DcMotorSimple.Direction.FORWARD);
        MDT.setDirection(DcMotorSimple.Direction.FORWARD);
        MDT.setDirection(DcMotorSimple.Direction.FORWARD);
        MET.setDirection(DcMotorSimple.Direction.REVERSE);
        MEF.setDirection(DcMotorSimple.Direction.REVERSE);

        MDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MDF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MDT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MET.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MEF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LSi = opMode.hardwareMap.get(DcMotorEx.class, "LSi");
        LSii = opMode.hardwareMap.get(DcMotorEx.class, "LSii");
        LSi.setDirection(DcMotorSimple.Direction.REVERSE);
        LSi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSii.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LSi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LSii.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LSi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSii.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSii.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wristServo = opMode.hardwareMap.get(Servo.class, "yawC");
        clawServo = opMode.hardwareMap.get(Servo.class, "garra");
        wristServo.setPosition(0);
        clawServo.setPosition(0);
        Arm = opMode.hardwareMap.get(DcMotorEx.class, "braço");
        armEncoder = opMode.hardwareMap.get(DcMotorEx.class, "armEncoder");
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);
        armEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //define o meio de "freio" para os motores
        imu = opMode.hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
    }
}