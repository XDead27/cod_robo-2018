package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Autonomie_Crater extends AutonomieMain {

    @Override
    public void runOpMode() {
        initOp();

        waitForStart();

        runOp();

        endOp();

    }

    @Override
    protected void runOp() {
        setMotorsDirectionFLBR(DIRECTION.FORWARD, 0.9);
        while(Range1.rawUltrasonic() > 0 && Range2.rawUltrasonic() > 0){
            idle();
        }

        stopMotors();
    }

    @Override
    protected void endOp() {

    }
}
