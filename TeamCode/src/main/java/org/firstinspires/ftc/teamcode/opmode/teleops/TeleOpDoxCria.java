package org.firstinspires.ftc.teamcode.opmode.teleops;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.subsytems.DriveBaseSubsytem;

@TeleOp(name = "molhadinhos")
public class TeleOpDoxCria extends OpMode {
    DcMotorEx MDT, MDF, MET, MEF, LSi, LSii, braço;
    double angle, axial, lateral, yaw;
    Servo yawC, garra; //Define o nome dos servos no sistema
    boolean yawG, raw, motionType;
    ElapsedTime f = new ElapsedTime(); //define contador de tempo para as funções
    ElapsedTime tempo = new ElapsedTime(); // define  o contador do tempo decorrido para o PID

    IMU imu;
    boolean isOrientedTrue = false;

    double yawBase = 0, yawChanged = 0.37;

    public void init() {
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        LSi = hardwareMap.get(DcMotorEx.class, "LSi");
        LSii = hardwareMap.get(DcMotorEx.class, "LSii");
        yawC = hardwareMap.get(Servo.class, "yawC");
        garra = hardwareMap.get(Servo.class, "garra");
        braço = hardwareMap.get(DcMotorEx.class, "braço");
        imu = hardwareMap.get(IMU.class, "imu");


        MDF.setDirection(DcMotorSimple.Direction.FORWARD);
        MDT.setDirection(DcMotorSimple.Direction.FORWARD);
        MDT.setDirection(DcMotorSimple.Direction.FORWARD);
        MET.setDirection(DcMotorSimple.Direction.REVERSE);
        MEF.setDirection(DcMotorSimple.Direction.REVERSE);

        MDF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MET.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MEF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LSi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSii.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        braço.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LSi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LSii.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        braço.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MDF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MDT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MET.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MEF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Orientação do Control Hub/Expansion Hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Inicializa o giroscópio
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        LSii.setDirection(DcMotorSimple.Direction.REVERSE);
        braço.setDirection(DcMotorSimple.Direction.REVERSE);
        yawC.setPosition(0);
        garra.setPosition(0.5);
        motionType = true;
        yawG = false;
        raw = true;

        LSi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSii.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        braço.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //define o meio de "freio" para os motores
        LSi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LSii.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        braço.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LSi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSii.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSii.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop(){
        if (gamepad1.left_bumper && motionType){
            motionType = false;
        }
        else if (gamepad1.left_bumper && !motionType){
            motionType = true;
        }
        movi();
        linear(); //função do sistema linear
        arm(); //função do braço
        Crvos(); // função dos servos
    }
    public void movi(){
        telemetry.addData("Robot + field centric", motionType);
        if (motionType){
            defaultMove();
        }
        else{
            centricMove();
        }
    }

    public void defaultMove(){
        axial   = (gamepad1.right_trigger - gamepad1.left_trigger)* 0.8;
        lateral = gamepad1.left_stick_x * 0.8;
        yaw     =  gamepad1.right_stick_x * 0.6;


        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw= Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);
        double motorEsquerdoFf = (axial + lateral + yaw / denominador);
        double motorDireitoFf = (axial - lateral - yaw / denominador);
        double motorEsquerdoTf = (axial - lateral + yaw / denominador);
        double motorDireitoTf = (axial + lateral - yaw / denominador);

        if(gamepad1.right_bumper){
            MotorsPower(motorEsquerdoFf * 0.8, motorDireitoFf * 0.8, motorEsquerdoTf * 0.8, motorDireitoTf * 0.8);
        }
        else {
            MotorsPower(motorEsquerdoFf, motorDireitoFf, motorEsquerdoTf, motorDireitoTf);
        }
    }
    public void centricMove(){
        robotGyroMove(gamepad1.right_trigger-gamepad1.left_trigger,gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
    public void robotGyroMove(float axialPower, float lateralPower, float yawPower){
        axial = axialPower * 0.8;
        lateral = lateralPower;
        yaw = yawPower *0.7;

        if((yaw!=0) && (!isOrientedTrue)) {
            isOrientedTrue= true;
            imu.resetYaw();
        }
        if (isOrientedTrue){
            fieldOriented(axial,lateral);
        }
        if((yaw==0) && (isOrientedTrue)) {
            isOrientedTrue= false;
        }


        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw= Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);
        double motorEsquerdoFf = ((axial + lateral + yaw) / denominador);
        double motorDireitoFf = ((axial - lateral - yaw) / denominador);
        double motorEsquerdoTf = ((axial - lateral + yaw) / denominador);
        double motorDireitoTf = ((axial + lateral - yaw) / denominador);

        MotorsPower(motorEsquerdoFf, motorDireitoFf, motorEsquerdoTf, motorDireitoTf);
    }
    private void fieldOriented(double driveP, double turnP) {
        angle = gyroCalculate();
        axial = driveP * Math.cos(angle) - turnP * Math.sin(angle);
        lateral = driveP * Math.sin(angle) + turnP * Math.cos(angle);
    }

    private double gyroCalculate() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }

    public void MotorsPower(double p1, double p2, double p3,double p4){
        MEF.setPower(p1);
        MDF.setPower(p2);
        MET.setPower(p3);
        MDT.setPower(p4);
    }

    public void Crvos(){
        //Angulação da garra
        if (gamepad2.y && f.seconds() >= 0.3){
            if (!yawG){
                yawC.setPosition(yawChanged);
                yawG = true;
            }
            else if (yawG){
                yawC.setPosition(yawBase);
                yawG = false;
                yawChanged = 0.37;
            }
            f.reset();
        }

        //Abrir/fechar a garra
        if (gamepad2.x && f.seconds() >= 0.5){
            if (raw == true){
                garra.setPosition(0.7);
                raw = false;
            }
            else if (!raw) {
                garra.setPosition(0.0);
                raw = true;
            }
            f.reset();
        }
    }

    public void linear(){
        //funções do linear
        LSi.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        LSii.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        telemetry.addData("Linear", LSi.getCurrentPosition());
    }

    int posAlvo;
    public void arm(){
        //define a força do braço
        if (gamepad2.right_bumper){
            /*
            posAlvo = 40;
            braço.setTargetPosition(posAlvo);
            braço.setPower(0.6);
            braço.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            REATIVAR DEPOIS DE VER AS POSIÇÕES
            */
            braço.setPower(0.24);
        }
        else if (gamepad2.left_bumper){
            braço.setPower(-0.24);
        }
        else {
            braço.setPower(0);
        }
        telemetry.addData("braço", braço.getCurrentPosition());
    }

}
