package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@Config
@TeleOp(name="ledTest")
public class ledTest extends OpMode {
    public ServoImplEx led;

    public static double pwm = 0;


    @Override
    public void init() {

        led = hardwareMap.get(ServoImplEx.class, "led");
        led.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    @Override
    public void loop() {

        led.setPosition(pwm);

    }
}