package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
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
@TeleOp(name="ascentTest")
public class ascentTest extends OpMode {
    private liftSubsystem lift;
    private LinkageSubsystem linkage;




    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = new liftSubsystem(hardwareMap);
        lift.init();
        linkage = new LinkageSubsystem(hardwareMap);
        linkage.init();


    }

    @Override
    public void loop() {


        if (gamepad1.y) {
            lift.ascent2();
            linkage.hangIn();
        }
        if (gamepad1.a) {
            lift.ascent2Up();
        }
        if (gamepad1.x) {
            lift.ascent2Down();
            linkage.hangIn2();
        }



        lift.update();

        telemetry.addData("targetpos", lift.getTarget());
        telemetry.addData("lift1 pos", lift.getLift1Position());
        telemetry.addData("lift2 pos", lift.getLift2Position());
        telemetry.addData("lift1 power", lift.getLift1Power());
        telemetry.addData("lift2 power", lift.getLift2Power());
        telemetry.addData("Run time", getRuntime());


        telemetry.update();
    }
}