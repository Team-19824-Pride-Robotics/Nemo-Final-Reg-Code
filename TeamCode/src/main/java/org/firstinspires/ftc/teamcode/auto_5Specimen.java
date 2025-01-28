package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Config
@Autonomous(name = "5 Specimen Auto")
public class auto_5Specimen extends LinearOpMode {

/////////////////////////
/////Mech Positions//////
/////////////////////////

    public static int spHeight1 = 750;
    public static int spHeight2 = 1650;
    public static double AHPos = 0.05; //linkage in
    public static double BHPos = 0.11; //linkage in
    public static double AHPos2 = 0.27; //linkage out
    public static double BHPos2 = 0.45; //linkage out
    public static double Bpos = 0.32; //bucket up (not final)
    public static double Bpos2 = 0.33; //bucket down (not final)
    public static double Epos1 = .33; //Specimen grab
    public static double Epos2 = .58; //Specimen hang
    public static double Cpos = 0.72; //open
    public static double Cpos2 = 0.96; //closed
    public static double rwGrab = .46; //grab specimen
    public static double lwGrab = .5; //grab specimen
    public static double lwHang = 0.65; //hang specimen
    public static double rwHang = 0.61; //hang specimen
    ///////////////////////////
    /////Robot Positions//////
    /////////////////////////
    public static double x0 = 29;
    public static double x1 = 20;
    public static double y2 = -30;
    public static double x3 = 50;

    public static double y3 = -45;
    public static double x4 = 25;
    public static double y4 = -25;
    public static double y5 = -55;
    public static double x5 = 15;
    public static double y6 = -60;
    public static double x8 = 5;
    public static double y8 = -15;
    public static double y9 = 2.5;
    public static double y11 = 5;
    public static double y13 = 7.5;
    public static double y15 = 30;
    public static double y16 = 30;
    public static double x16 = 3;
/////////////////////
/////Sleep vars//////
/////////////////////
    public static double hangSleep=0.6;
    public static double grabSleep=0.5;
    public static double downSleep=0.8;

    public static double outSleep=0.8;
    public class Intake {
        ServoImplEx backWrist;

        ServoImplEx frontWrist;
        Servo claw;
        DcMotor intake;
        ServoImplEx elbow;
        ServoImplEx hSlide;
        ServoImplEx hSlide2;
        ServoImplEx bucket;


        public Intake(HardwareMap hardwareMap) {
            elbow = (ServoImplEx)hardwareMap.servo.get("arm");
            elbow.setPwmRange(new PwmControl.PwmRange(505, 2495));
            claw = hardwareMap.servo.get("claw");
            hSlide = (ServoImplEx)hardwareMap.servo.get("ll");
            hSlide.setPwmRange(new PwmControl.PwmRange(505, 2495));
            hSlide2 = (ServoImplEx)hardwareMap.servo.get("rl");
            hSlide2.setPwmRange(new PwmControl.PwmRange(505, 2495));
            bucket = (ServoImplEx)hardwareMap.servo.get("bucket");
            bucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
            backWrist = (ServoImplEx) hardwareMap.get(Servo.class, "lw");
            backWrist.setPwmRange(new PwmControl.PwmRange(505, 2495));
            frontWrist = (ServoImplEx) hardwareMap.get(Servo.class, "rw");
            frontWrist.setPwmRange(new PwmControl.PwmRange(505, 2495));
            intake = hardwareMap.get(DcMotor.class, "intake");
        }

        public class spHangPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //bring claw to origin
                elbow.setPosition(Epos2);
                frontWrist.setPosition(rwHang);
                backWrist.setPosition(lwHang);


                return false;
            }
        }
        public Action spHangPos() {
            return new spHangPos();
        }

        public class openClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(Cpos);
                return false;
            }
        }
        public Action openClaw() {
            return new openClaw();
        }



        public class closeClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(Cpos2);
                return false;
            }
        }
        public Action closeClaw() {
            return new closeClaw();
        }


        public class spGrabPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                elbow.setPosition(Epos2);
                frontWrist.setPosition(rwGrab);
                backWrist.setPosition(lwGrab);

                return false;
            }
        }
        public Action spGrabPos() {
            return new spGrabPos();
        }
        public class intakeOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                hSlide.setPosition(AHPos2);
                hSlide2.setPosition(BHPos2);
                return false;
            }
        }
        public Action intakeOut() {
            return new intakeOut();
        }
        public class intakeIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                hSlide.setPosition(AHPos);
                hSlide2.setPosition(BHPos);
                return false;
            }
        }
        public Action intakeIn() {
            return new intakeIn();
        }
        public class bucketUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                bucket.setPosition(Bpos);
                return false;
            }
        }
        public Action bucketUp() {
            return new bucketUp();
        }
        public class bucketDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                bucket.setPosition(Bpos2);
                return false;
            }
        }
        public Action bucketDown() {
            return new bucketDown();
        }

        public class intakeOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                intake.setPower(0);




                return false;
            }
        }
        public Action intakeOff() {
            return new intakeOff();
        }
        public class intakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                intake.setPower(1);




                return false;
            }
        }
        public Action intakeOn() {
            return new intakeOn();
        }
        public class intakeExpel implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                intake.setPower(-1);




                return false;
            }
        }
        public Action intakeExpel() {
            return new intakeExpel();
        }




    }


    public class lift {

        private DcMotorEx lift1;
        private DcMotorEx lift2;
        
            
        public lift(HardwareMap hardwareMap) {

            lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
            lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift1.setDirection(DcMotorEx.Direction.REVERSE);
        }

        public class upABit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lift1.setTargetPosition(spHeight2);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setTargetPosition(spHeight2);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
                return false;
            }
        }
        public Action upABit() {
            return new upABit();
        }
        public class baseHeight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lift1.setTargetPosition(0);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setTargetPosition(0);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
                return false;
            }
        }
        public Action baseHeight() {
            return new baseHeight();
        }

        public class scoreHeight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lift1.setTargetPosition(spHeight1);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setTargetPosition(spHeight1);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
                return false;
            }
        }
        public Action scoreHeight() {
            return new scoreHeight();
        }


    }



    @Override
    public void runOpMode(){


        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make an Intake instance
        Intake intake = new Intake(hardwareMap);
        lift lift = new lift(hardwareMap);
        // make a Lift instance



        TrajectoryActionBuilder segment1;
        TrajectoryActionBuilder segment2;
        TrajectoryActionBuilder segment3;
        TrajectoryActionBuilder segment4;
        TrajectoryActionBuilder segment5;
        TrajectoryActionBuilder segment6;
        TrajectoryActionBuilder segment7;
        TrajectoryActionBuilder segment8;
        TrajectoryActionBuilder segment9;
        TrajectoryActionBuilder segment10;
        TrajectoryActionBuilder segment11;
        TrajectoryActionBuilder segment12;
        TrajectoryActionBuilder segment13;
        TrajectoryActionBuilder segment14;
        TrajectoryActionBuilder segment15;
        TrajectoryActionBuilder segment16;
        TrajectoryActionBuilder segment17;
        segment1 = drive.actionBuilder(initialPose)

                .strafeToLinearHeading(new Vector2d(x0, 0), 0);

        Action seg1 = segment1.build();

        //segment 2 - backs off the sub

        segment2 = segment1.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x1, 0), 0);

        Action seg2 = segment2.build();

        //segment 3 - clear submersible
        segment3 = segment2.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x0, y2), Math.toRadians(0));

        Action seg3 = segment3.build();

        //segment 4 - get behind block
        segment4 = segment3.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x3, y3), Math.toRadians(0));

        Action seg4 = segment4.build();

        //segment 5 - push block
        segment5 = segment4.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x4, y3), Math.toRadians(0));

        Action seg5 = segment5.build();

        //segment 6 - get behind 2nd sample
        segment6 = segment5.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x3, y5), Math.toRadians(0));

        Action seg6 = segment6.build();

        //segment 7 - Push 2nd sample
        segment7 = segment6.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x4, y5), Math.toRadians(0));

        Action seg7 = segment7.build();

        //segment 8 - get behind 3rd sample
        segment8 = segment7.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x3, y6), Math.toRadians(0));

        Action seg8 = segment8.build();

        //segment 9 - push 3rd sample
        segment9 = segment8.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x4, y6), Math.toRadians(0));

        Action seg9 = segment9.build();

        //segment 10 - hang 2nd specimen
        segment10 = segment9.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x0, y9), Math.toRadians(0));

        Action seg10 = segment10.build();

        //segment 11 - grab 3rd specimen
        segment11 = segment10.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x8, y8), Math.toRadians(0));

        Action seg11 = segment11.build();

        //segment 12 - hang 3rd specimen
        segment12 = segment11.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x0, y11), Math.toRadians(0));

        Action seg12 = segment12.build();

        //segment 13 - grab 4th specimen
        segment13 = segment12.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x8, y8), Math.toRadians(0));

        Action seg13 = segment13.build();

        //segment 14 - hang 4th specimen
        segment14 = segment13.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x0, y13), Math.toRadians(0));

        Action seg14 = segment14.build();

        //segment 15 - grab 5th specimen
        segment15 = segment14.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x8, y8), Math.toRadians(0));

        Action seg15 = segment15.build();

        //segment 16 - hang 5th specimen (yippee!!!)
        segment16 = segment15.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x0, y15), Math.toRadians(0));

        Action seg16 = segment16.build();
        waitForStart();

        //segment 17 - park
        segment17 = segment16.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x16, y16), Math.toRadians(0));

        Action seg17 = segment17.build();
        waitForStart();


        if (isStopRequested()) return;



        Actions.runBlocking(new SequentialAction(
                //score first specimen
                intake.closeClaw(),
                intake.bucketUp(),
                intake.spHangPos(),
                intake.intakeIn(),
                lift.scoreHeight(),
                seg1,
                lift.upABit(),
                new SleepAction(hangSleep),
                intake.openClaw(),
                lift.baseHeight(),
                intake.spGrabPos(),

                //push samples in observation zone
                seg2,
                seg3,
                seg4,
                new SleepAction(downSleep),
                intake.intakeOut(),
                intake.bucketDown(),
                intake.intakeExpel(),
                seg5,
                intake.bucketUp(),
                intake.intakeIn(),
                seg6,
                new SleepAction(downSleep),
                intake.intakeOut(),
                intake.bucketDown(),
                intake.intakeExpel(),
                seg7,
                intake.bucketUp(),
                intake.intakeIn(),
                seg8,
                intake.spGrabPos(),
                

                //score 2nd specimen
                seg9,
                new SleepAction(grabSleep),
                intake.closeClaw(),
                intake.spHangPos(),
                lift.scoreHeight(),
                seg10,
                lift.upABit(),
                new SleepAction(hangSleep),
                intake.openClaw(),
                lift.baseHeight(),
                intake.spGrabPos(),

                //score 3rd specimen
                seg11,
                new SleepAction(grabSleep),
                intake.closeClaw(),
                intake.spHangPos(),
                lift.scoreHeight(),
                seg12,
                lift.upABit(),
                new SleepAction(hangSleep),
                intake.openClaw(),
                lift.baseHeight(),
                intake.spGrabPos(),

                //score 4th specimen
                seg13,
                new SleepAction(grabSleep),
                intake.closeClaw(),
                intake.spHangPos(),
                lift.scoreHeight(),
                seg14,
                lift.upABit(),
                new SleepAction(hangSleep),
                intake.openClaw(),
                lift.baseHeight(),
                intake.spGrabPos(),

                //score 5th specimen
                seg15,
                new SleepAction(grabSleep),
                intake.closeClaw(),
                intake.spHangPos(),
                lift.scoreHeight(),
                seg16,
                lift.upABit(),
                new SleepAction(hangSleep),
                intake.openClaw(),
                lift.baseHeight(),
                intake.spGrabPos(),

                //park
                seg17



        ));


    }

}