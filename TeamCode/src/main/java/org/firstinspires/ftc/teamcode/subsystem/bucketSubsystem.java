package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config

public class bucketSubsystem {

    private final ServoImplEx bucket;
    private final AnalogInput encoderBucket;

    public static double bucketDown = 0.37;
    public static double bucketUp = .3;

    public double bucketTargetPosition =.31 ;

    public bucketSubsystem(HardwareMap hardwareMap) {
        bucket = hardwareMap.get(ServoImplEx.class, "bucket");
        bucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
        encoderBucket = hardwareMap.get(AnalogInput.class, "eBucket");

    }

    public void init() {
        bucket.setPosition(bucketUp);
    }

    public void bucketDown() {
        bucketTargetPosition = bucketDown;
    }

    public void bucketUp(){
        bucketTargetPosition = bucketUp;
    }

    public double getBucketPosition() {
        return bucketTargetPosition;
    }
    public double getBucketEncoderPosition() {
        return encoderBucket.getVoltage() / 3.3 * 360;
    }
    public void update() {
        bucket.setPosition(bucketTargetPosition);
    }

}

