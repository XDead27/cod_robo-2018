package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class AutonomieMain extends LinearOpMode {
    //motoare
    protected DcMotor Motor_FL = null;
    protected DcMotor Motor_FR = null;
    protected DcMotor Motor_BL = null;
    protected DcMotor Motor_BR = null;
    protected DcMotor Motor_Glisiera = null;
    protected DcMotor Motor_Ridicat = null;

    //servo
    protected CRServo Servo_Palete_1 = null;
    protected CRServo Servo_Palete_2 = null;

    //constante
    protected static final int TICS_PER_CM = 67;
    protected static final double deadzone = 0.1;

    protected static double power_Motor_FLBR = 0;
    protected static double power_Motor_FRBL = 0;
    protected static double power_Motor_LEFT = 0;
    protected static double power_Motor_RIGHT = 0;


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
        Motor_Glisiera = hardwareMap.dcMotor.get("Motor_Glisiera");
        Motor_Ridicat = hardwareMap.dcMotor.get("Motor_Ridicat");
        Servo_Palete_1 = hardwareMap.crservo.get("Servo_1");
        Servo_Palete_2 = hardwareMap.crservo.get("Servo_2");

        //setare directii
        Motor_BL.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor_FL.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_BR.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_FR.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_Glisiera.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor_Ridicat.setDirection(DcMotorSimple.Direction.FORWARD);
        Servo_Palete_1.setDirection(DcMotorSimple.Direction.FORWARD);
        Servo_Palete_2.setDirection(DcMotorSimple.Direction.REVERSE);


        //setare mod
        Motor_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_Glisiera.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_Ridicat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initializare putere
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
        Motor_Ridicat.setPower(0);
        Motor_Glisiera.setPower(0);
    }

    protected void setMovementFLBR(DIRECTION dir, double speed){
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
                FLBR = speed;
                FRBL = -speed;
                break;

            case LEFT:
                FLBR = -speed;
                FRBL = speed;
                break;
        }

       setMotorPowersFLBR(FLBR, FRBL);
    }

    protected void setMotorPowersFLBR(double FLBR, double FRBL){
        Motor_FL.setPower(FLBR);
        Motor_BR.setPower(FLBR);
        Motor_FR.setPower(FRBL);
        Motor_BL.setPower(FRBL);
    }

    protected void setMotorsPowerTank(double RightSpeed, double LeftSpeed){
        Motor_FL.setPower(LeftSpeed);
        Motor_BR.setPower(RightSpeed);
        Motor_FR.setPower(RightSpeed);
        Motor_BL.setPower(LeftSpeed);
    }

    protected void stopMotors(){
        Motor_FL.setPower(0);
        Motor_BR.setPower(0);
        Motor_FR.setPower(0);
        Motor_BL.setPower(0);
    }
}
