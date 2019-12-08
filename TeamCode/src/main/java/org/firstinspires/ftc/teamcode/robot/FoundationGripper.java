package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationGripper {
    public Servo gripperF;
    public Servo gripperB;

    public FoundationGripper(HardwareMap ahwMap){
        gripperF = ahwMap.get(Servo.class, "gripper_front");
        gripperB = ahwMap.get(Servo.class, "gripper_back");

        gripperF.setPosition(0);
        gripperB.setPosition(0);
    }

    public void grab(){
        gripperF.setPosition(1);
        gripperB.setPosition(1);
    }

    public void reset(){
        gripperF.setPosition(0);
        gripperB.setPosition(0);
    }
}
