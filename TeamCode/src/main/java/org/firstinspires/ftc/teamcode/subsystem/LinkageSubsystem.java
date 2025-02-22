package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public class LinkageSubsystem {
    private final ServoImplEx ll, rl;
    private final AnalogInput encoderRl, encoderLl;

    public static final double SERVO1_MIN = 0.035, SERVO1_MAX = 0.245; //RIGHT
    public static final double SERVO2_MIN = 0.06, SERVO2_MAX = 0.43; //LEFT
    public static final double SERVO_FULL_MIN = 0.01, SERVO_FULL_MAX = 0.99;
    public static double rlHangIn = .02, llHangIn = .06;
    public static double rlHangIn2 = .02, llHangIn2 = .06;

    public static double rlSampleOut = .245, llSampleOut = .43;



    private boolean stickControlEnabled = false;
    private double stickControlMin = SERVO1_MIN;

    public LinkageSubsystem(HardwareMap hardwareMap) {
        rl = initServo(hardwareMap, "rl", SERVO1_MIN);
        ll = initServo(hardwareMap, "ll", SERVO2_MIN);
        encoderRl = hardwareMap.get(AnalogInput.class, "eRl");
        encoderLl = hardwareMap.get(AnalogInput.class, "eLl");
    }

    private ServoImplEx initServo(HardwareMap hardwareMap, String name, double initialPosition) {
        ServoImplEx servo = (ServoImplEx) hardwareMap.get(Servo.class, name);
        servo.setPwmRange(new PwmControl.PwmRange(505, 2495));
        servo.setPosition(initialPosition);
        return servo;
    }

    public void init() {
        rl.setPosition(SERVO1_MIN);
        ll.setPosition(SERVO2_MIN);
    }

    public void enableStickControl(double min, double position) {
        stickControlEnabled = true;
        stickControlMin = min;
        setServoPositions(position);
    }

    public void disableStickControl() {
        stickControlEnabled = false;
        setServoPositions(SERVO1_MIN);
    }
    public void hangIn(){
        rl.setPosition(rlHangIn);
        ll.setPosition(llHangIn);
    }
    public void hangIn2(){
        rl.setPosition(rlHangIn2);
        ll.setPosition(llHangIn2);
    }
    public void sampleOut(){
        rl.setPosition(rlSampleOut);
        ll.setPosition(llSampleOut);
    }

    public boolean isStickControlEnabled() {
        return stickControlEnabled;
    }

    public double getStickControlMin() {
        return stickControlMin;
    }

    public double getServo1Position() {
        return rl.getPosition();
    }

    public double getServo2Position() {
        return ll.getPosition();
    }
    public double getEncoderRlPosition() {
        return encoderRl.getVoltage() / 3.3 * 360;
    }
    public double getEncoderLlPosition() {
        return encoderLl.getVoltage() / 3.3 * 360;
    }

    public void updateServoPositions(double stickY) {
        double servo1Position = map(stickY, 0, 1, stickControlMin, SERVO1_MAX);
        setServoPositions(servo1Position);
    }

    private double map(double value, double inMin, double inMax, double outMin, double outMax) {
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    private double mapServo2(double servo1Position) {
        return map(servo1Position, SERVO1_MIN, SERVO1_MAX, SERVO2_MIN, SERVO2_MAX);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void setServoPositions(double servo1Position) {
        rl.setPosition(clamp(servo1Position, SERVO_FULL_MIN, SERVO_FULL_MAX));
        ll.setPosition(clamp(mapServo2(servo1Position), SERVO_FULL_MIN, SERVO_FULL_MAX));
    }
}

