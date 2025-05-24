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

    public static double bucketDown = 0.16;
    public static double bucketUp = 0.1;

    public static double bucketEject = .4;
    public static double bucketAlmostDown = .14;

    public static double coverOpen = 0.7;
    public static double coverClose = 0.39;
    public double bucketTargetPosition = bucketUp;
    public double coverTargetPosition = coverOpen;

    public bucketSubsystem(HardwareMap hardwareMap) {
        bucket = hardwareMap.get(ServoImplEx.class, "bucket");
        bucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
        encoderBucket = hardwareMap.get(AnalogInput.class, "eBucket");
        encoderCover = hardwareMap.get(AnalogInput.class, "eCover ");
        cover = hardwareMap.get(Servo.class, "cover");

    }

    public void init() {
        bucket.setPosition(bucketUp);
        cover.setPosition(coverOpen);
    }

    public void bucketDown() {
        bucketTargetPosition = bucketDown;
    }

    public void bucketUp() {
        bucketTargetPosition = bucketUp;
    }

    public void bucketEjec() {
        bucketTargetPosition = bucketEject;
    }

    public void bucketAlmostDown() {
        bucketTargetPosition = bucketAlmostDown;
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

    public double getCoverPosition() {
        return cover.getPosition();
    }
}


