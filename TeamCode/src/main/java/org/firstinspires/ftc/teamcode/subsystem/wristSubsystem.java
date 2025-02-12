package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config

public class wristSubsystem {

    public final ServoImplEx rw, lw;
    private final AnalogInput encoderRw, encoderLw;

    public static double rwPickup = .33;
    public static double lwPickup = .37;
    public static double rwIn = .3;
    public static double lwIn = .34;
    public static double rwPickupSpeicmen = .47;
    public static double lwPickupSpeicmen = .5;
    public static double rwScoreSpeicmen = .63;
    public static double lwScoreSpeicmen = .67;


    public static double rwScore = .46;
    public static double lwScore = .5;

    public static double rwOut = .30;
    public static double lwOut = .34;


    public double rwTargetPosition =.33;
    public double lwTargetPosition = .37;

    public wristSubsystem(HardwareMap hardwareMap) {
        rw = hardwareMap.get(ServoImplEx.class, "rw");
        rw.setPwmRange(new PwmControl.PwmRange(505, 2495));
        encoderRw = hardwareMap.get(AnalogInput.class, "eRw");
        lw = hardwareMap.get(ServoImplEx.class, "lw");
        lw.setPwmRange(new PwmControl.PwmRange(505, 2495));
        encoderLw = hardwareMap.get(AnalogInput.class, "eLw");

    }

    public void init() {
        rw.setPosition(rwPickup);
        lw.setPosition(lwPickup);
    }

    public void wristPickup() {
        rwTargetPosition = rwPickup;
        lwTargetPosition = lwPickup;
    }
    public void wristIn() {
        rwTargetPosition = rwIn;
        lwTargetPosition= lwIn;
    }
    public void wristPickupSpeicmen(){
        rwTargetPosition = rwPickupSpeicmen;
        lwTargetPosition = lwPickupSpeicmen;
    }
    public void wristScoreSpeicmen(){
        rwTargetPosition = rwScoreSpeicmen;
        lwTargetPosition = lwScoreSpeicmen;
    }
    public void wristScore() {
        rwTargetPosition = rwScore;
        lwTargetPosition = lwScore;
    }
    public void wristOut() {
        rwTargetPosition = rwOut;
        lwTargetPosition = lwOut;
    }

    public double getRwTargetPosition() {
        return rwTargetPosition;
    }
    public double getlwTargetPosition() {
        return lwTargetPosition;
    }
    public double getRwEncoderPosition() {
        return encoderRw.getVoltage() / 3.3 * 360;
    }
    public double getLwEncoderPosition() {
        return encoderLw.getVoltage() / 3.3 * 360;
    }
    public void update() {
        rw.setPosition(rwTargetPosition);
        lw.setPosition(lwTargetPosition);
    }

}

