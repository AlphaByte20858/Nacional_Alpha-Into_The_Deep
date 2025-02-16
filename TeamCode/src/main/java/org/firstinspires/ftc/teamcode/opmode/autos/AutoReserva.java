package org.firstinspires.ftc.teamcode.opmode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoReserva", group = "LinearOpMode")
public class AutoReserva extends LinearOpMode {
    public DcMotorEx MEF, MET, MDF, MDT, LSi, LSii, braço;
    public Servo garra;
 /*   BNO055IMU imu;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffs1 = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains1 = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffs2 = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains2 = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients pidCoeffs3 = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains3 = new PIDCoefficients(0, 0, 0);


    double integral = 0;
    double error = 0;
    double currvel = 0;
    double derivate = 0;
    double deltaError = 0;
    private double lastError = 0;

    double integral1 = 0;
    double error1 = 0;
    double currvel1 = 0;
    double derivate1 = 0;
    double deltaError1 = 0;
    private double lastError1 = 0;

    double integral2 = 0;
    double error2 = 0;
    double currvel2 = 0;
    double derivate2 = 0;
    double deltaError2 = 0;
    private double lastError2 = 0;

    double integral3 = 0;
    double error3 = 0;
    double currvel3 = 0;
    double derivate3 = 0;
    double deltaError3 = 0;
    private double lastError3 = 0;
*/

    ElapsedTime tempo = new ElapsedTime();


    @Override
    public void runOpMode() {
        LSi = hardwareMap.get(DcMotorEx.class, "LSi");
        LSii = hardwareMap.get(DcMotorEx.class, "LSii");
        braço = hardwareMap.get(DcMotorEx.class, "braço");
        garra = hardwareMap.get(Servo.class, "garra");

        MEF = hardwareMap.get(DcMotorEx.class, "MEF");
        MDF = hardwareMap.get(DcMotorEx.class, "MDF");
        MET = hardwareMap.get(DcMotorEx.class, "MET");
        MDT = hardwareMap.get(DcMotorEx.class, "MDT");

        braço.setDirection(DcMotorSimple.Direction.REVERSE);
        LSi.setDirection(DcMotorSimple.Direction.REVERSE);

        LSi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LSii.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        modemoto(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        modemoto(DcMotorEx.RunMode.RUN_USING_ENCODER);

        MEF.setDirection(DcMotorEx.Direction.REVERSE);
        MDF.setDirection(DcMotorEx.Direction.FORWARD);
        MET.setDirection(DcMotorEx.Direction.REVERSE);
        MDT.setDirection(DcMotorEx.Direction.FORWARD);

        garra.setPosition(0.15);

        resetRuntime();
        waitForStart();

        if (opModeIsActive()) {
            allMotorsPower(-0.1, 0.1, 0.1,-0.1);
            esperar(1);
            allMotorsPower(0,0,0,0);
            esperar(2);
            allMotorsPower(-0.05,-0.05,-0.05,-0.05);
            esperar(1);
            braço.setPower(0.7);
            allMotorsPower(0,0,0,0);
            esperar(2);
            braço.setPower(0);
            linearPower(0.73);
            esperar(1);
            linearPower(0);
            esperar(2);
            braço.setPower(-0.3);
            esperar(1);
            braço.setPower(0);
            linearPower(-0.3);
            esperar(1);
           /* allMotorsPower(-0.5, -0.5, -0.5, -0.5);
            esperar(1);
            allMotorsPower(0, 0, 0, 0);
            sleep(3000);
            linearPower(-0.2);
            esperar(1);*/
            garra.setPosition(0);
            esperar(1);
            linearPower(0);


        }
    }

    public void esperar(double temp){
        tempo.reset();
        while (tempo.seconds() < temp){

        }
    }
    public void allMotorsPower(double paMEF, double paMDF, double paMET, double paMDT){
        MEF.setPower(paMEF);
        MDF.setPower(paMDF);
        MET.setPower(paMET);
        MDT.setPower(paMDT);
    }
    public void linearPower(double paLS){
        LSi.setPower(paLS);
        LSii.setPower(paLS);
    }

    public void modemoto(DcMotorEx.RunMode mode){
        MDF.setMode(mode);
        MDT.setMode(mode);
        MEF.setMode(mode);
        MET.setMode(mode);
    }
}