package org.firstinspires.ftc.teamcode;

import android.support.annotation.IntRange;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp (name = "Driver_Test", group = "Driver")

public class DriverTest extends LinearOpMode {
    //motoare
    private DcMotor Motor_Glisiera = null;
    private DcMotor Motor_FL = null;
    private DcMotor Motor_FR = null;
    private DcMotor Motor_BL = null;
    private DcMotor Motor_BR = null;
    private DcMotor Motor_Ridicat = null;

    //servo
    private CRServo Servo_Palete_1 = null;
    private CRServo Servo_Palete_2 = null;

    //constante
    static final int TICS_PER_CM = 67;
    static final double deadzone = 0.1;

    static double power_Motor_FLBR = 0;
    static double power_Motor_FRBL = 0;
    static double power_Motor_LEFT = 0;
    static double power_Motor_RIGHT = 0;

    @Override
    public void runOpMode()
    {
        initialise();

        waitForStart();

        while(opModeIsActive())
        {
            //gamepad 1

            if ( Math.abs(gamepad1.left_stick_x) > deadzone || Math.abs(gamepad1.left_stick_y) > deadzone || Math.abs(gamepad1.right_stick_x) > deadzone)
                calculateWheelsPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            else
                calculateWheelsPower(0,0,0);


            if(gamepad1.x){
                Servo_Palete_1.setPower(0.8);
                Servo_Palete_2.setPower(0.8);
            }
            if(gamepad1.y){
                Servo_Palete_1.setPower(0.5);
                Servo_Palete_2.setPower(0.5);
            }

            //gamepad 2

            if(gamepad2.a) {
                Motor_Glisiera.setPower(0.7);


            }else if(gamepad2.b) {
                Motor_Glisiera.setPower(-0.7);

            }else{
                Motor_Glisiera.setPower(0);
            }


            if(gamepad2.left_trigger > deadzone){
                Motor_Ridicat.setPower(Motor_Ridicat.getCurrentPosition() > -29.432*TICS_PER_CM ? gamepad2.left_trigger/3 : 0);
            }else if(gamepad2.right_trigger > deadzone){
                Motor_Ridicat.setPower(Motor_Ridicat.getCurrentPosition() < 0 ? -gamepad2.right_trigger/3 : 0);
            }else{
                Motor_Ridicat.setPower(0);
            }

            telemetry.addData("Left X", gamepad1.left_stick_x);
            telemetry.addData("Left Y", gamepad1.left_stick_y);
            telemetry.addData( "Motor_Glisiera: ", Motor_Glisiera.getCurrentPosition());
            telemetry.addData("Motor Ridicat:", Motor_Ridicat.getCurrentPosition());
            telemetry.update();
        }
    }

    private void initialise()
    {
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
        Motor_Glisiera.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor_BL.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor_FL.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_BR.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_FR.setDirection(DcMotorSimple.Direction.REVERSE);
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

    //calculeaza puterile rotilor in functie de stick si apoi aplica rotilor puterea
    private void calculateWheelsPower ( double left_X, double left_Y, double RotationChange )
    {
        power_Motor_FLBR = left_Y - left_X;
        power_Motor_FRBL = left_Y + left_X;
        power_Motor_LEFT = RotationChange;
        power_Motor_RIGHT = -RotationChange;
        setWheelsPower();
    }

    //aplica rotilor puterea
    private void setWheelsPower()
    {
        Motor_FL.setPower(power_Motor_FLBR + power_Motor_LEFT);
        Motor_FR.setPower(power_Motor_FRBL + power_Motor_RIGHT);
        Motor_BL.setPower(power_Motor_FRBL + power_Motor_LEFT);
        Motor_BR.setPower(power_Motor_FLBR + power_Motor_RIGHT);
    }
}