package org.firstinspires.ftc.teamcode.hardware.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.interfaces.SubsystemBase;
import org.firstinspires.ftc.teamcode.hardware.robot.RobotHardware;


@Config
public class DriveBaseSubsytem implements SubsystemBase {
    private RobotHardware Robot;
    double angle,axial,lateral,yaw;
    boolean isOrientedTrue = false;

    public DriveBaseSubsytem(RobotHardware Robot){
        this.Robot = Robot;
    }
    public void init(){

    }

    public void periodic() {

    }
    public void manualControl(float axial, float lateral, float yaw){

        double absaxial = Math.abs(axial);
        double abslateral = Math.abs(lateral);
        double absyaw= Math.abs(yaw);
        double denominador = Math.max(absaxial + abslateral + absyaw, 1);
        double motorEsquerdoFf = (axial + lateral + yaw / denominador);
        double motorDireitoFf = (axial - lateral - yaw / denominador);
        double motorEsquerdoTf = (axial - lateral + yaw / denominador);
        double motorDireitoTf = (axial + lateral - yaw / denominador);

        setMotorsPower(motorEsquerdoFf, motorDireitoFf, motorEsquerdoTf, motorDireitoTf);
    }

    public void robotGyroMove(float axialPower, float lateralPower, float yawPower){
        axial = axialPower * 0.8;
        lateral = lateralPower;
        yaw = yawPower *0.7;

        if((yaw!=0) && (!isOrientedTrue)) {
            isOrientedTrue= true;
            Robot.imu.resetYaw();
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

        setMotorsPower(motorEsquerdoFf, motorDireitoFf, motorEsquerdoTf, motorDireitoTf);
    }
    private void setMotorsPower(double MEFpower,double MDFpower,double METpower,double MDTpower){
        Robot.MEF.setPower(MEFpower);
        Robot.MDF.setPower(MDFpower);
        Robot.MET.setPower(METpower);
        Robot.MDT.setPower(MDTpower);
    }
    private void fieldOriented(double driveP, double turnP) {
        angle = gyroCalculate();
        axial = driveP * Math.cos(angle) - turnP * Math.sin(angle);
        lateral = driveP * Math.sin(angle) + turnP * Math.cos(angle);
    }

    // Função que retorna a orientação do robô em graus
    private double gyroCalculate() {
        YawPitchRollAngles orientation = Robot.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }
}
