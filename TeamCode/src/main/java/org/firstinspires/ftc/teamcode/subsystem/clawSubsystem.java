package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class clawSubsystem {

    private final Servo claw;
    public static double clawOpen = 0.8;
    public static double clawClose = .98;

    public double clawTargetPosition = .8;

    public clawSubsystem(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
    }

    public void init() {
        claw.setPosition(clawOpen);
    }

    public void clawOpen() {
        clawTargetPosition= clawOpen;
    }

    public void clawClose(){
        clawTargetPosition = clawClose;
    }
    public double getClawPosition() {
        return clawTargetPosition;
    }
    public void update() {
        claw.setPosition(clawTargetPosition);
    }

}

