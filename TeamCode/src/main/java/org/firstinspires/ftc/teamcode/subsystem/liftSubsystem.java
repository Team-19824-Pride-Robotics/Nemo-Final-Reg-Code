package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
public class liftSubsystem {

    private final DcMotorEx lift1, lift2;
    private final PIDController controller;

    public static final double P = 0.005, I = 0, D = 0;
    //lift max 2980
    public static double pickup = 0;
    public static double bucketLow = 800;
    public static double bucketHigh = 2850;
    public static double barLow = 700;
    public static double barHigh = 1000;
    public static double score = 200;

    private double target = 0;

    public liftSubsystem(HardwareMap hardwareMap) {
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        controller = new PIDController(P, I, D);
    }

    public void init() {
        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift1.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void pickup() {
        target = pickup;
    }
    public void bucketLow() {
        target = bucketLow;
    }
    public void bucketHigh() {
        target = bucketHigh;
    }
    public void score() {
        target = target - score;
    }
    public void barLow() {
        target = barLow;
    }
    public void barHigh() {
        target = barHigh;
    }
    public double getTarget() {
        return target;
    }

    public int getLift1Position() {
        return lift1.getCurrentPosition();
    }

    public int getLift2Position() {
        return lift2.getCurrentPosition();
    }

    public void update() {
        double pid1 = controller.calculate(lift1.getCurrentPosition(), target);
        double pid2 = controller.calculate(lift2.getCurrentPosition(), target);

        lift1.setPower(pid1);
        lift2.setPower(pid2);
    }
}

