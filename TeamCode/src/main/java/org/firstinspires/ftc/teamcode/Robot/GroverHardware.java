package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;



public class GroverHardware {

    HardwareMap hwMap =  null;
    Drivetrain dt = null;
    public GroverHardware(){

    }

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;
        dt = new Drivetrain(hwMap);
    }




}
