package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.armSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.bucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.clawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.liftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.wristSubsystem;

import java.util.List;

@Config
@TeleOp(name="Inspection")
public class Inspection extends OpMode {

    private Servo claw;
    private ServoImplEx arm;
    private ServoImplEx lw;
    private ServoImplEx rw;
    private ServoImplEx rl;
    private ServoImplEx ll;


    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;



    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); //MANUAL
        }

        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(ServoImplEx.class, "arm");
        arm.setPwmRange(new PwmControl.PwmRange(505, 2495));
        lw = hardwareMap.get(ServoImplEx.class, "lw");
        lw.setPwmRange(new PwmControl.PwmRange(505, 2495));
        rw = hardwareMap.get(ServoImplEx.class, "rw");
        rw.setPwmRange(new PwmControl.PwmRange(505, 2495));
        rl = hardwareMap.get(ServoImplEx.class, "rl");
        rl.setPwmRange(new PwmControl.PwmRange(505, 2495));
        ll = hardwareMap.get(ServoImplEx.class, "ll");
        ll.setPwmRange(new PwmControl.PwmRange(505, 2495));

        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
    }

    @Override
    public void loop() {

      /*  for (LynxModule hub : allHubs){
            hub.clearBulkCache();
        } */

     //   distance = distanceSensor.getDistance(DistanceUnit.MM);
        if (gamepad1.a){
            claw.setPosition(.92);
            arm.setPosition(.84);
            lw.setPosition(.5);
            rw.setPosition(.46);
            rl.setPosition(.27);
            ll.setPosition(.45);
        }

        telemetry.addData("Run time", getRuntime());


        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
        telemetry.update();
    }
}