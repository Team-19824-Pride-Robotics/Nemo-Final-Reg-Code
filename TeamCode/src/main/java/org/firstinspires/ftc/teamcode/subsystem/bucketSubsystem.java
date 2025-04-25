package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config

public class bucketSubsystem {

    private final ServoImplEx bucket;
    private final AnalogInput encoderBucket;
    private final AnalogInput encoderCover;
    private final Servo cover;

    public static double bucketDown = 0.75;
    public static double bucketUp = 0.65
            ;

    public static double bucketEject = .05;
    public static double bucketSampleOut = .3;

    public static double coverOpen = 0.99;
    public static double coverClose = 0.5;
    public double bucketTargetPosition = bucketUp ;
    public double coverTargetPosition = coverOpen;
    public bucketSubsystem(HardwareMap hardwareMap) {
        bucket = hardwareMap.get(ServoImplEx.class, "bucket");
        bucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
        encoderBucket = hardwareMap.get(AnalogInput.class, "eBucket");
        encoderCover = hardwareMap.get(AnalogInput.class, "eEncoder");
        cover = hardwareMap.get(Servo.class, "cover");

    }

    public void init() {
        bucket.setPosition(bucketUp);
        cover.setPosition(coverOpen);
    }
    public void bucketDown() {
        bucketTargetPosition = bucketDown;
    }

    public void bucketUp(){
        bucketTargetPosition = bucketUp;
    }

    public void bucketEjec() {
        bucketTargetPosition = bucketEject;
    }
    public void sampleOut() {
        bucketTargetPosition = bucketSampleOut;
    }
    public void coverOpen() {
        coverTargetPosition = coverOpen;
    }

    public void coverClose() {
        coverTargetPosition = coverClose;
    }

    public double getBucketPosition() {
        return bucketTargetPosition;
    }
    public double getBucketEncoderPosition() {
        return encoderBucket.getVoltage() / 3.3 * 360;
    }
    public void update() {
        bucket.setPosition(bucketTargetPosition);
        cover.setPosition(coverTargetPosition);
    }

    public double getCoverEncoderPosition() {
    return encoderCover.getVoltage() / 3.3 * 360;
}

}

