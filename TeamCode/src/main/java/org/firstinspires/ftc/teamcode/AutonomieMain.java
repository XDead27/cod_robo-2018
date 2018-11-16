package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;

public abstract class AutonomieMain extends LinearOpMode {
    //motoare
    protected DcMotor Motor_FL = null;
    protected DcMotor Motor_FR = null;
    protected DcMotor Motor_BL = null;
    protected DcMotor Motor_BR = null;

    //motoare mecanisme
    private DcMotor Motor_Glisiera_Fata = null;
    private DcMotor Motor_Glisiera_Ridicat = null;
    private DcMotor Motor_Rotire_Glisiera_Fata = null;
    private DcMotor Motor_Captare = null;

    //constante
    protected static final int TICS_PER_CM = 67;
    protected static final double deadzone = 0.1;

    protected static double power_Motor_FLBR = 0;
    protected static double power_Motor_FRBL = 0;
    protected static double power_Motor_LEFT = 0;
    protected static double power_Motor_RIGHT = 0;

    //senzori
    protected OpticalDistanceSensor ODS1 = null;
    protected ModernRoboticsAnalogOpticalDistanceSensor ODS2 = null;
    protected ModernRoboticsI2cRangeSensor Range1 = null;
    protected ModernRoboticsI2cRangeSensor Range2 = null;
    protected GyroSensor gyro = null;

    enum DIRECTION{
        FORWARD,
        REVERSE,
        LEFT,
        RIGHT
    }

    protected abstract void runOp();

    protected abstract void endOp();

    protected void initOp(){
        //hardware mapping
        Motor_FL = hardwareMap.dcMotor.get("Motor_FL");
        Motor_FR = hardwareMap.dcMotor.get("Motor_FR");
        Motor_BL = hardwareMap.dcMotor.get("Motor_BL");
        Motor_BR = hardwareMap.dcMotor.get("Motor_BR");

        //setare directii
        Motor_BL.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor_FL.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_BR.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_FR.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_Captare.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_Glisiera_Fata.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_Glisiera_Ridicat.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor_Rotire_Glisiera_Fata.setDirection(DcMotorSimple.Direction.REVERSE);


        //setare mod
        Motor_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initializare putere
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
        Motor_Captare.setPower(0);
        Motor_Glisiera_Ridicat.setPower(0);
        Motor_Glisiera_Fata.setPower(0);
        Motor_Rotire_Glisiera_Fata.setPower(0);
    }

    protected void setMotorsDirectionFLBR(DIRECTION dir, double speed){
        double FLBR = 0, FRBL = 0;
        switch (dir){
            case FORWARD:
                FLBR = speed;
                FRBL = speed;
                break;

            case REVERSE:
                FLBR = -speed;
                FRBL = -speed;
                break;

            case RIGHT:
                FLBR = -speed;
                FRBL = speed;
                break;

            case LEFT:
                FLBR = speed;
                FRBL = -speed;
                break;
        }

        setMotorsPowerFLBR(FLBR, FRBL);
    }

    protected void setMotorsPowerFLBR(double FLBR, double FRBL){
        Motor_FL.setPower(FLBR);
        Motor_BR.setPower(FLBR);
        Motor_FR.setPower(FRBL);
        Motor_BL.setPower(FRBL);

    }

    protected void setMotorsPowersRotate(double Left, double Right){
        Motor_BL.setPower(Left);
        Motor_FL.setPower(Left);
        Motor_BR.setPower(Right);
        Motor_FR.setPower(Right);
    }

    protected void stopMotors(){
        Motor_FL.setPower(0);
        Motor_BR.setPower(0);
        Motor_FR.setPower(0);
        Motor_BL.setPower(0);

    }
}
