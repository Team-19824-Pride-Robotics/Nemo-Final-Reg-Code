package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class intakeSubsystem {

    private final DcMotorEx intake;
    public static double intakeIn;
    public static double intakeOut;

    public double intakePower = 0;

    public intakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");

    }

    public void init() {
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intakeSetPower() {
        intakePower = intakeIn + intakeOut;
    }

    public void intakeSetIdle(){
        intakePower = 0;
    }
    public double getIntakeIn() {
        return intakeIn;
    }
    public double getIntakeOut() {
        return intakeOut;
    }
    public double getIntakePower() {
        return intakePower;
    }
    public void update() {
        intake.setPower(intakePower);
    }

}

