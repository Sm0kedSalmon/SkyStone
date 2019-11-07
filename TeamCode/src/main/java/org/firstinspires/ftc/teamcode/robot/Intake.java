package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotor intakeLeft;
    public DcMotor intakeRight;

    public static final double INTAKE_POWER = 0.25;

    public Intake(HardwareMap ahwMap){
        intakeLeft = ahwMap.get(DcMotor.class, "IntakeLeft");
        intakeRight = ahwMap.get(DcMotor.class, "IntakeRight");

        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void intakeOn(){
        intakeLeft.setPower(INTAKE_POWER);
        intakeRight.setPower(INTAKE_POWER);
    }
    public void intakeReverse(){
        intakeLeft.setPower(-INTAKE_POWER);
        intakeRight.setPower(-INTAKE_POWER);
    }
    public void intakeOff(){
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }
}
