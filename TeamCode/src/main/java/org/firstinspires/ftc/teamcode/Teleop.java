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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.armSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.bucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.clawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.liftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.wristSubsystem;

import java.util.List;

@Config
@TeleOp(name="A_Teleop")
public class Teleop extends OpMode {
    private LinkageSubsystem linkage;
    private liftSubsystem lift;
    private intakeSubsystem intake;
    private bucketSubsystem bucket;
    private clawSubsystem claw;
    private armSubsystem arm;
    private wristSubsystem wrist;
    boolean dpad_up =false ;
    boolean dpad_down =false ;


    boolean pickup = true;
    boolean pickup2 =  false;
    boolean liftPickup = true;
    boolean intaking = false;
    boolean specPos = false;

    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;

    //sensor
    RevBlinkinLedDriver LED;
    RevBlinkinLedDriver.BlinkinPattern pattern;
  /*  DistanceSensor distanceSensor;
    public double distance; */
    ColorSensor colorSensor;
    public double red;
    public double green;
    public double blue;

    DcMotorEx FR;
    DcMotorEx FL;
    DcMotorEx BR;
    DcMotorEx BL;
    public static double drive_speed_M = 1;
    boolean ascent3 = false;
    public static double liftpos = 1000;

    public static double linkagePos = .18;
    public static double linkageMin = .13;


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        linkage = new LinkageSubsystem(hardwareMap);
        linkage.init();
        lift = new liftSubsystem(hardwareMap);
        lift.init();
        intake = new intakeSubsystem(hardwareMap);
        intake.init();
        bucket = new bucketSubsystem(hardwareMap);
        bucket.init();
        claw = new clawSubsystem(hardwareMap);
        claw.init();
        arm = new armSubsystem(hardwareMap);
        arm.init();
        wrist = new wristSubsystem(hardwareMap);
        wrist.init();

        FR = hardwareMap.get(DcMotorEx.class, "rightFront");
        FL = hardwareMap.get(DcMotorEx.class, "leftFront");
        BR = hardwareMap.get(DcMotorEx.class, "rightBack");
        BL = hardwareMap.get(DcMotorEx.class, "leftBack");
        BR.setDirection(DcMotorEx.Direction.REVERSE);
        FR.setDirection(DcMotorEx.Direction.REVERSE);

        LED = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        LED.setPattern(pattern);

        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs){
            hub.clearBulkCache();
        }

     //   distance = distanceSensor.getDistance(DistanceUnit.MM);
        red= colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();
        //////////////////////////
        /// Gamepad 1 controls ///
        //////////////////////////
        double d_power = .5 - .4 * gamepad1.left_trigger + (.5 * gamepad1.right_trigger);
        double drive = gamepad1.left_stick_y * drive_speed_M;
        double rotate = -gamepad1.right_stick_x * drive_speed_M;

        BL.setPower(drive + rotate);
        FL.setPower(drive + rotate);
        BR.setPower(drive - rotate);
        FR.setPower(drive - rotate);

        if (gamepad1.dpad_up) {
            BL.setPower(-d_power);
            FL.setPower(-d_power);
            BR.setPower(-d_power);
            FR.setPower(-d_power);
        }
        else if (gamepad1.dpad_down) {
            BL.setPower(d_power);
            FL.setPower(d_power);
            BR.setPower(d_power);
            FR.setPower(d_power);
        }
        else if (gamepad1.dpad_left) {
            BL.setPower(-d_power);
            FL.setPower(d_power);
            BR.setPower(d_power);
            FR.setPower(-d_power);
        }
        else if (gamepad1.dpad_right) {
            BL.setPower(d_power);
            FL.setPower(-d_power);
            BR.setPower(-d_power);
            FR.setPower(d_power);
        }
        else if (gamepad2.dpad_right) {
            BL.setPower(0.5);
            FL.setPower(0.5);
            BR.setPower(-0.5);
            FR.setPower(-0.5);
        }
        else if (gamepad2.dpad_left) {
            BL.setPower(-0.5);
            FL.setPower(-0.5);
            BR.setPower(0.5);
            FR.setPower(0.5);
        }

        //specimen control
        if (gamepad1.x){

            lift.barHigh();
            specPos = true;
        }
        if (lift.getLift1Position() > 400 && specPos) {
            arm.armSpecimen();
            wrist.wristScoreSpeicmen();
            specPos = false;
        }
        if (gamepad1.y){
            lift.score();
        }
        if (gamepad1.a){
            arm.armPickupSpeicmen();
            lift.pickup();
            wrist.wristPickupSpeicmen();
        }

        if (gamepad1.b){
            lift.barLow();
        }
        //claw control
        if (gamepad1.left_bumper||gamepad2.left_bumper) {
            claw.clawOpen();
        }
        if (gamepad1.right_bumper||gamepad2.right_bumper) {
            claw.clawClose();
        }
        /*
        if (gamepad1.back){
            arm.armPark();
            wrist.wristScore();
            lift.pickup();
        }
        if (gamepad1.start) {
            lift.ascent2();
        }
        if (gamepad1.back) {
            lift.ascent2Up();
        }
        if (gamepad1.y) {
            lift.ascent2();
        }
        if (gamepad1.a) {
            lift.ascent2Up();
        }
        if (gamepad1.x) {
            lift.ascent3();
        }
        if (gamepad1.b) {
            lift.ascent3();
            arm.armHang();
            ascent3 = true;
        }

        if (lift.getLift1Position() > liftpos && ascent3){
            lift.ascent3Up();
        }
*/

        //////////////////////////
        /// Gamepad 2 controls ///
        //////////////////////////

        //linkage control
        if (gamepad2.start) {
            linkage.enableStickControl(linkageMin, linkagePos);
            arm.armPickup();
            liftPickup = false;
            intaking = true;
        } else if (gamepad2.back) {
            linkage.disableStickControl();
            bucket.bucketUp();
            pickup2= true;
            intaking = false;
        }
        if (linkage.getEncoderRlPosition() <= 38 && pickup2) {
            arm.armPickup2();
            wrist.wristIn();
        }
        if(arm.getArmEncoderPosition()<=120 && pickup2){
            wrist.wristPickup();
            pickup2= false;
        }
        if (linkage.isStickControlEnabled()) {
            double stickY = Math.max(-gamepad2.left_stick_y, 0);
            linkage.updateServoPositions(stickY);
        }

        //lift control

        if (gamepad2.a) {
            wrist.wristPickup();
            arm.armPickup();
            pickup = true;
            liftPickup = true;
        }
        if (arm.getArmPosition() <= 120&& pickup) {
            lift.pickup();
            pickup = false;
        }
        if (lift.getLift1Position() <= 10 && liftPickup) {
            arm.armPickup2();
            liftPickup = false;
        }
        if (gamepad2.y) {
            dpad_up = true;
            lift.bucketHigh();
            arm.armSample();
            wrist.wristOut();
        }

        if (gamepad2.b) {
            dpad_down =true;
            lift.bucketLow();
            arm.armSample();
            wrist.wristOut();
        }

        if ((arm.getArmEncoderPosition() >= 200 && dpad_up) || (arm.getArmEncoderPosition() >= 200 && dpad_down)) {
            wrist.wristScore();
            pickup = false;
            liftPickup = false;
            dpad_down = false;
            dpad_up = false;
        }

        //intake control
        if (gamepad2.left_trigger > .1 || gamepad2.right_trigger > .1) {
            intakeSubsystem.intakeIn = Math.pow(gamepad2.right_trigger, 3);
            intakeSubsystem.intakeOut = Math.pow(-gamepad2.left_trigger, 3);
            intake.intakeSetPower();
        } else {
            intake.intakeSetIdle();
        }

        //bucket control
        if (gamepad2.dpad_up) {
            bucket.bucketUp();
        }
        if (gamepad2.dpad_down) {
            bucket.bucketDown();
        }


        lift.update();
        intake.update();
        bucket.update();
        claw.update();
        arm.update();
        wrist.update();

        //led
        if (intaking){
            if ((red>=405&& red<=1631)&&(green>=235&&green<=900)&&(blue>=117&&blue<=524)) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            }
            // Check for Blue Block
            else if ((green>=214&&green<=826)&&(blue>=500&&blue<=1828)&&(red<455)) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            }
            // Check for Yellow Block
            else if ((red>=690&&red<=2380)&&(green>=925&&green<=3100)&&(blue>=200&&blue<=790)) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            }
            else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
            }
        }
      /*  if (!intaking){
            if (distance <=250&& distance>25 ){
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            }
            else if(distance<=25){
                pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
            }
            else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
            }
        } */

        LED.setPattern(pattern);

        telemetry.addData("Run time", getRuntime());
        //linkage
        telemetry.addData("Stick Control Enabled", linkage.isStickControlEnabled());
        telemetry.addData("Stick Control Min", linkage.getStickControlMin());
        telemetry.addData("RL Position", linkage.getServo1Position());
        telemetry.addData("LL Position", linkage.getServo2Position());
        telemetry.addData("Encoder RL Position", linkage.getEncoderRlPosition());
        telemetry.addData("Encoder LL Position", linkage.getEncoderLlPosition());
        //lift
        telemetry.addData("Lift Target", lift.getTarget());
        telemetry.addData("Lift Position 1", lift.getLift1Position());
        telemetry.addData("Lift Position 2", lift.getLift2Position());
        //intake
        telemetry.addData("intakePower", intake.getIntakePower());
        telemetry.addData("intakePowerin", intake.getIntakeIn());
        telemetry.addData("intakePowerout", intake.getIntakeOut());
        //bucket
        telemetry.addData("bucketTarget", bucket.getBucketPosition());
        telemetry.addData("bucketEncoder", bucket.getBucketEncoderPosition());
        //claw
        telemetry.addData("clawPosition", claw.getClawPosition());
        //arm
        telemetry.addData("armTarget", arm.getArmPosition());
        telemetry.addData("armEncoder", arm.getArmEncoderPosition());
        //wrist
        telemetry.addData("rwTarget", wrist.getRwTargetPosition());
        telemetry.addData("rwEncoder", wrist.getRwEncoderPosition());



        telemetry.addData ("pickup", pickup);

       /* telemetry.addData("Lift Position 1", lift.getLift1Position());
        telemetry.addData("Lift Position 2", lift.getLift2Position());
        telemetry.addData("Encoder RL Position", linkage.getEncoderRlPosition());
        telemetry.addData("Encoder LL Position", linkage.getEncoderLlPosition());
        telemetry.addData("bucketEncoder", bucket.getBucketEncoderPosition());
        telemetry.addData("armEncoder", arm.getArmEncoderPosition());
        telemetry.addData("lwEncoder", wrist.getLwEncoderPosition());
        telemetry.addData("rwEncoder", wrist.getRwEncoderPosition());*/

        /*telemetry.addData("lift1 setPower", lift.getLift1SetPower());
        telemetry.addData("lift1 power",lift.getLift1Power());
        telemetry.addData("lift2 setPower", lift.getLift2SetPower());
        telemetry.addData("lift2 power",lift.getLift2Power());*/

        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
        telemetry.update();
    }
}