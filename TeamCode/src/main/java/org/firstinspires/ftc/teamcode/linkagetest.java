package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name="linkageTest",  group="Test")
public class linkagetest extends OpMode {

    //pid
    ServoImplEx s1;
    ServoImplEx s2;

    public static double rl1 = .09;
    public static double rl2 = .27;

    public static double ll1 = .14;
    public static double ll2 = .45;

    // Servo ranges
    private final double SERVO1_MIN = 0.09;
    private final double SERVO1_MAX = 0.27;
    private final double SERVO2_MIN = 0.14;
    private final double SERVO2_MAX = 0.45;

    // Full range limits
    private final double SERVO_FULL_MIN = 0.01;
    private final double SERVO_FULL_MAX = 0.99;

    // State variables
    private boolean stickControlEnabled = false;
    private double stickControlMin = SERVO1_MIN; // Dynamic minimum for stick control

    public static double stickControlMin1 = .13;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s1 = (ServoImplEx) hardwareMap.get(Servo.class, "rl");
        s1.setPwmRange(new PwmControl.PwmRange(505, 2495));
        s2 = (ServoImplEx) hardwareMap.get(Servo.class, "ll");
        s2.setPwmRange(new PwmControl.PwmRange(505, 2495));

        s1.setPosition(SERVO1_MIN);
        s2.setPosition(SERVO2_MIN);


    }

    @Override
    public void loop() {
/*
        if (gamepad1.b) {
            // Enable stick control and set Servo 1 to 0.2
            stickControlEnabled = true;
            stickControlMin = 0.13; // Set new minimum for stick control
            setServoPositions(0.2);
        } else if (gamepad1.x) {
            // Disable stick control and reset to minimum positions
            stickControlEnabled = false;
            setServoPositions(SERVO1_MIN);
        }

        if (stickControlEnabled) {
            // Use X stick to control the servo range
            double stickY = -gamepad1.left_stick_y;

            // Ignore negative X values (clamp to 0)
            if (stickY < 0) {
                stickY = 0;
            }

            // Map stick X to the dynamic range
            double servo1Position = map(stickY, 0, 1, stickControlMin, SERVO1_MAX);

            // Calculate Servo 2 position
            double servo2Position = mapServo2(servo1Position);

            // Clamp positions and set servos
            servo1Position = clamp(servo1Position, SERVO_FULL_MIN, SERVO_FULL_MAX);
            servo2Position = clamp(servo2Position, SERVO_FULL_MIN, SERVO_FULL_MAX);

            // Set positions
            s1.setPosition(servo1Position);
            s2.setPosition(servo2Position); */



        if (gamepad1.b) {
            s2.setPosition(ll1);
            s1.setPosition(rl1);
        }
        if (gamepad1.x) {
            s2.setPosition(ll2);
            s1.setPosition(rl2);
        }



            telemetry.addData("Run time", getRuntime());
            telemetry.addData("1", "org/firstinspires/ftc/teamcode/test");
            telemetry.addData("Stick Control Enabled", stickControlEnabled);
            telemetry.addData("Stick Control Min", stickControlMin);
            telemetry.addData("Servo 1 Position", s1.getPosition());
            telemetry.addData("Servo 2 Position", s2.getPosition());
            telemetry.update();
        }
    }

