package org.firstinspires.ftc.teamcode;

public class Autonomie_Patrat extends AutonomieMain {

    static final double MIN_DIST_BEFORE_WALL = 15;
    static final double FORWARD_SPEED = 0.8;
    static final double ERROR_ALLOWED = 10;

    @Override
    public void runOpMode() {
        initOp();

        waitForStart();

        runOp();

        endOp();
    }

    @Override
    protected void runOp() {

        //roteste pana cand e perpendicular cu peretele din dreapta
        double MinDistanceToWall = Range1.rawUltrasonic();

        setMotorsPowersRotate(0.5, -0.5);
        while(MinDistanceToWall > Range1.rawUltrasonic() || MinDistanceToWall > Range2.rawUltrasonic()){
            idle();
        }

        stopMotors();

        //merge in fata pana langa perete si se roteste inspre crater
        setMotorsDirectionFLBR(DIRECTION.FORWARD, FORWARD_SPEED);
        while(Range1.cmUltrasonic() > MIN_DIST_BEFORE_WALL){
            if(Range1.rawUltrasonic() > Range2.rawUltrasonic()){
                setMotorsPowersRotate(FORWARD_SPEED + 0.1, FORWARD_SPEED);
            }else if(Range1.rawUltrasonic() < Range2.rawUltrasonic()){
                setMotorsPowersRotate(FORWARD_SPEED, FORWARD_SPEED + 0.1);
            }else{
                setMotorsDirectionFLBR(DIRECTION.FORWARD, FORWARD_SPEED);
            }
        }
        stopMotors();

        setMotorsPowersRotate(0.5, -0.5);
        while(Math.abs(Range1.rawUltrasonic() - Range2.rawUltrasonic()) > ERROR_ALLOWED || Range1.cmUltrasonic() < MIN_DIST_BEFORE_WALL + 30){
            idle();
        }
        stopMotors();

        //se lipeste de perete
        ODS1.enableLed(true);
        setMotorsDirectionFLBR(DIRECTION.LEFT,0.5);
        while(ODS1.getLightDetected() < 0.5){
            idle();
        }
        stopMotors();

        //se duce pana in crater
        setMotorsDirectionFLBR(DIRECTION.FORWARD, FORWARD_SPEED);
        while(Range1.rawUltrasonic() > 0 && Range2.rawUltrasonic() > 0){
            idle();
        }

        stopMotors();
    }

    @Override
    protected void endOp() {
        ODS1.enableLed(false);
    }
}
