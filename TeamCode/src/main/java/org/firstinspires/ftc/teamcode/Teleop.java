package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
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
@TeleOp(name="A_Teleop")
public class Teleop extends OpMode {
    public LynxModule CONTROL_HUB, EXPANSION_HUB;
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

  /*  DistanceSensor distanceSensor;
    public double distance; */
   //ColorSensor colorSensor;

    public double red;
    public double green;
    public double blue;


    public double d_power;

    DcMotorEx FR;
    DcMotorEx FL;
    DcMotorEx BR;
    DcMotorEx BL;
    public static double drive_speed_M = 1;
    boolean liftWait = false;
    public static double liftpos = 1000;

    public static double linkagePos = .1;
    public static double linkageMin = .1;
    public double rotateMod = 1;

    public boolean spec = false;
    public boolean startPressedLast = false;
    public boolean ascent = false;
    public boolean backPressedLast = false;
    public boolean bucketDown = false;
    public boolean hold = false;


    //led
//    private DigitalChannel red1;
//    private DigitalChannel green1;
//    private DigitalChannel red2;
//    private DigitalChannel green2;





    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        if(allHubs.get(0).isParent() && LynxConstants.isEmbeddedSerialNumber(allHubs.get(0).getSerialNumber())) {
            CONTROL_HUB = allHubs.get(0);
            EXPANSION_HUB = allHubs.get(1);
        } else {
            CONTROL_HUB = allHubs.get(1);
            EXPANSION_HUB = allHubs.get(0);
        }

        linkage = new LinkageSubsystem(hardwareMap);
        //linkage.init();
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

        //colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


//        red1 = hardwareMap.get(DigitalChannel.class, "red1");
//        green1 = hardwareMap.get(DigitalChannel.class, "green1");
//        red2 = hardwareMap.get(DigitalChannel.class, "red2");
//        green2 = hardwareMap.get(DigitalChannel.class, "green2");
//
//        red1.setMode(DigitalChannel.Mode.OUTPUT);
//        green1.setMode(DigitalChannel.Mode.OUTPUT);
//        red2.setMode(DigitalChannel.Mode.OUTPUT);
//        green2.setMode(DigitalChannel.Mode.OUTPUT);

        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
    }

    @Override
    public void loop() {

            for (LynxModule hub : allHubs) {
                if (hub.getDeviceName().equals("Servo Hub 3")) return;
                CONTROL_HUB.clearBulkCache();
                EXPANSION_HUB.clearBulkCache();
            }

        for (LynxModule hub : allHubs){
            hub.clearBulkCache();
        }
        // colorsensor

//        red= colorSensor.red();
//        green = colorSensor.green();
//        blue = colorSensor.blue();

//         ,

        if (gamepad1.start && !startPressedLast) {
            spec = !spec;
        }
        startPressedLast = gamepad1.start;

        if (gamepad1.back && !backPressedLast) {
            ascent = !ascent;
        }
        backPressedLast = gamepad1.back;

        //////////////////////////
        /// Gamepad 1 controls ///
        //////////////////////////
        if (spec) {
            d_power = 1 - .4 * gamepad1.right_trigger + (.5 * gamepad1.left_trigger);
        }
        if (!spec) {
            d_power = .5 - .4 * gamepad1.left_trigger + (.5 * gamepad1.right_trigger);
        }
        if(intaking){
            rotateMod=0.3;
        } else{
            rotateMod=1;
        }
        double drive = gamepad2.left_stick_y * drive_speed_M;
        double driveX = -gamepad2.left_stick_x * drive_speed_M;
        double rotate = -gamepad2.right_stick_x * drive_speed_M * rotateMod;

        BL.setPower(drive - driveX + rotate);
        FL.setPower(drive + driveX + rotate);
        BR.setPower(drive + driveX - rotate);
        FR.setPower(drive - driveX - rotate);

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

    if (ascent){
        if (gamepad1.y) {
            lift.ascent2();
            linkage.hangIn2();

        }
        if (gamepad1.a) {
            lift.ascent2Up();
            linkage.hangIn2();
        }
        if (gamepad1.x) {
            lift.ascent2Down();
            linkage.hangIn2();
            arm.armHang();
            wrist.wristScore();
        }
    }
    if (!ascent) {
        //specimen control
        if (gamepad1.x) {

            lift.barHigh();
            specPos = true;
        }
        if (lift.getLift1Position() > 400 && specPos) {
            arm.armSpecimen();
            wrist.wristScoreSpeicmen();
            specPos = false;
        }
        if (gamepad1.y) {
            lift.score();
        }
        if (gamepad1.a) {
            arm.armPickupSpeicmen();
            lift.pickup();
            wrist.wristPickupSpeicmen();
        }

        if (gamepad1.b) {
            lift.barLow();
        }
    }



    //claw control
    if (gamepad1.left_bumper || gamepad2.left_bumper) {
        claw.clawOpen();
    }
    if (gamepad1.right_bumper || gamepad2.right_bumper) {
        claw.clawClose();
    }



        //////////////////////////
        /// Gamepad 2 controls ///
        //////////////////////////

        //linkage control
        if (gamepad2.start) {
            linkage.enableStickControl(linkageMin, linkagePos);
            arm.armPickup();
            wrist.wristPickup();
            bucket.coverClose();
            liftPickup = false;
            intaking = true;
        }
        if (gamepad2.back) {
            linkage.disableStickControl();
            bucket.bucketUp();
            pickup2= true;
            intaking = false;
            bucketDown = false;
            hold=false;
        }
        if (linkage.getEncoderRlPosition() <= 5 && pickup2) {
            arm.armPickup2();
            wrist.wristPickup2();
            pickup2= false;
        }
     /*   if(arm.getArmEncoderPosition()<=70 && pickup2){
            wrist.wristPickup();
            pickup2= false;
        } */
        if (linkage.isStickControlEnabled() && !hold) {
            double stickY = Math.max(-gamepad2.right_stick_y, 0);
            linkage.updateServoPositions(stickY);
        }

        //lift control

        if (gamepad2.a) {
            wrist.wristPickup2();
            arm.armPickup2();
            pickup = true;
            liftPickup = true;
        }
        if (arm.getArmEncoderPosition() <= 125&& pickup) {
            lift.pickup();
            pickup = false;
        }
//        if (lift.getLift1Position() <= 10 && liftPickup) {
//            arm.armPickup2();
//            wrist.wristPickup2();
//            liftPickup = false;
//        }
        if (gamepad2.y) {
            dpad_up = true;
            liftWait = true;
            bucket.coverOpen();
            lift.bucketHigh();
        }

        if (gamepad2.b) {
            dpad_down =true;
            liftWait = true;
            bucket.coverOpen();
            lift.bucketLow();
        }
        if (gamepad2.x) {
            hold=true;
        }

        if ((arm.getArmEncoderPosition() >= 100 && dpad_up) || (arm.getArmEncoderPosition() >= 100 && dpad_down)) {
            //wrist.wristScore();
            pickup = false;
            liftPickup = false;
            dpad_down = false;
            dpad_up = false;
        }
        if(lift.getLift1Position() > 900 && liftWait){
            arm.armSample();
            wrist.wristOut();
            liftWait=false;
        }

        //intake control
        if (gamepad2.left_trigger > .1 || gamepad2.right_trigger > .1) {
            intakeSubsystem.intakeIn = Math.pow(gamepad2.right_trigger, 3);
            intakeSubsystem.intakeOut = Math.pow(-gamepad2.left_trigger, 3);
            intake.intakeSetPower();
        }
         else {
            intake.intakeSetIdle();
        }
        if(gamepad2.left_trigger > .1){
            bucket.coverOpen();
        }
        if(gamepad2.right_trigger > .1){
            bucket.coverClose();
        }
        //bucket control
        if (gamepad2.dpad_up) {
            bucket.bucketUp();
            bucketDown=false;
        }
        if (gamepad2.dpad_down) {
            bucket.bucketAlmostDown();
            bucketDown=true;
        }
        if(bucketDown) {
            if (intake.getIntakePower() > 0.1){
                bucket.bucketDown();
        } else {
                bucket.bucketAlmostDown();
            }
        }

//        if (spec){
//            green1.setState(true);
//            red1.setState(false);
//        }
//        if (ascent){
//            green1.setState(false);
//            red1.setState(true);
//        }
//        if (!spec && !ascent){
//                green1.setState(false);
//                red1.setState(false);
//        }




        lift.update();
        intake.update();
        bucket.update();
        claw.update();
        arm.update();
        wrist.update();



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

        telemetry.addData("coverTarget", bucket.getCoverPosition());
        telemetry.addData("coverEncoder", bucket.getCoverEncoderPosition());


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