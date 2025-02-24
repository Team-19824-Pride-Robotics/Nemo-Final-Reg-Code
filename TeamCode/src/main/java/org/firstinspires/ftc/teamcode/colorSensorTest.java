package org.firstinspires.ftc.teamcode.test;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name="colorSensor",  group="Test")
public class colorSensorTest extends OpMode {

    ColorSensor colorSensor;
    public double red;
    public double green;
    public double blue;

    public static double redMin =300;
    public static double redMax =340;

    public static double greenMin =500;
    public static double greenMax =540;
    public static double blueMin =370;
    public static double blueMax =410;

    private DigitalChannel red2;
    private DigitalChannel green2;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


        red2 = hardwareMap.get(DigitalChannel.class, "red2");
        green2 = hardwareMap.get(DigitalChannel.class, "green2");
    }

    @Override
    public void loop() {

        red= colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();
        if ( (red>300 && red < 340)&& (green>500 && green <540) && (blue>370&& blue<410)){
            green2.setState(false);
            red2.setState(false);
        }
        else {
            green2.setState(true);
            red2.setState(false);
        }


        telemetry.addData("Run time", getRuntime());
        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);
        telemetry.update();
    }
}