package org.firstinspires.ftc.teamcode.test;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="colorSensor",  group="Test")
public class colorSensorTest extends OpMode {

    ColorSensor colorSensor;
    public double red;
    public double green;
    public double blue;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


    }

    @Override
    public void loop() {

        red= colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();



        telemetry.addData("Run time", getRuntime());
        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);
        telemetry.update();
    }
}