package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//Class that has all of the hardware of the robot, as well as some misc. methods.
public class GroverHardware {

    HardwareMap hwMap =  null;

    public Drivetrain dt = null;
    public Intake intake = null;
    public Lift lift = null;
    public FoundationGripper gripper = null;

    public BNO055IMU imu;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public ElapsedTime time = new ElapsedTime();

    public GroverHardware(){

    }

    //Initializes hardware in each OpMode
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        dt = new Drivetrain(hwMap);
        intake = new Intake(hwMap);
        lift = new Lift(hwMap);
        gripper = new FoundationGripper(hwMap);

        //Set up IMU parameters
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }

    public void initNoGyro(HardwareMap ahwMap){
        hwMap = ahwMap;

        dt = new Drivetrain(hwMap);
        intake = new Intake(hwMap);
        lift = new Lift(hwMap);
        gripper = new FoundationGripper(hwMap);
    }

    //Returns the robot heading in degrees
    public double getHeading(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void gyroTurnPID(int degrees){
        dt.disableEncoders();
        dt.turnToAnglePID.reset();
        time.reset();
        while(getHeading() != degrees  && time.seconds() < 2) {
            double c = dt.gyroPIDCorrection(getHeading(), degrees, dt.turnToAnglePID);
            dt.setMotorPower(-c, c, -c, c);
            packet.put("Error", dt.turnToAnglePID.getTarget() - dt.turnToAnglePID.getCurrent());
            dashboard.sendTelemetryPacket(packet);
        }
        dt.setMotorPower(0,0,0,0);
        dt.resetEncoders();
    }



}
