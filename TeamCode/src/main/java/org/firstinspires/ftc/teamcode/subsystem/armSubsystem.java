package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config

public class armSubsystem {

    private final ServoImplEx arm;
    private final AnalogInput encoderArm;
    public static double armPickup = 0.7; //Arm pos to clear linkage
    public static double armPickupSpcimen = .71; //Grab Spec from wall

    public static double armSample = .4; //Score Sample
    public static double armSpecimen = 0.54 ; //Score Specimen

    public static double armPickup2 = 0.76; //Arm pos to grab from bucket

    public double armTargetPosition = armPickup2;

    public static double park = .62;
    public static double armHang = .01;
    public static double armAscent = .01;

    public armSubsystem(HardwareMap hardwareMap) {
        arm = hardwareMap.get(ServoImplEx.class, "arm");
        arm.setPwmRange(new PwmControl.PwmRange(505, 2495));
        encoderArm = hardwareMap.get(AnalogInput.class, "eArm");

    }

    public void init() {
        arm.setPosition(armPickup);
    }

    public void armPickup() {
        armTargetPosition = armPickup;
    }

    public void armPickup2(){
        armTargetPosition = armPickup2;
    }
    public void armPickupSpeicmen() {
        armTargetPosition= armPickupSpcimen;
    }
    public void armPark() {
        armTargetPosition= park;
    }
    public void armHang () {
        armTargetPosition= armHang;
    }


    public void armSample(){
        armTargetPosition = armSample;
    }
    public void armSpecimen(){
        armTargetPosition = armSpecimen;
    }

    public double getArmPosition() {
        return armTargetPosition;
    }
    public double getArmEncoderPosition() {
        return encoderArm.getVoltage() / 3.3 * 360;
    }
    public void update() {
        arm.setPosition(armTargetPosition);
    }

}

