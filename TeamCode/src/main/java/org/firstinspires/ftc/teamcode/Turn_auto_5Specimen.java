package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


@Config
@Autonomous(name = "Turn 5 Specimen Auto")
public class Turn_auto_5Specimen extends LinearOpMode {

/////////////////////////
/////Mech Positions//////
/////////////////////////

    public static int spHeight1 = 850;
    public static int spHeight2 = 1650;
    public static double AHPos = 0.05; //linkage in
    public static double BHPos = 0.11; //linkage in
    public static double AHPos1 = 0.24; //linkage in a little bit
    public static double BHPos1 = 0.42; //linkage in a little bit
    public static double AHPos2 = 0.27; //linkage out
    public static double BHPos2 = 0.45; //linkage out
    public static double Bpos = 0.3; //bucket up
    public static double Bpos2 = 0.31; //bucket Mid (not final)
    public static double Bpos3 = 0.33; //bucket down
    public static double Epos1 = .33; //Specimen grab
    public static double Epos2 = .58; //Specimen hang
    public static double Cpos = 0.72; //open
    public static double Cpos2 = 0.8; //open a little (to not descore other specimens)(not final)
    public static double Cpos3 = 0.97; //closed
    public static double rwGrab = .47; //grab specimen
    public static double lwGrab = .51; //grab specimen
    public static double lwHang = 0.67; //hang specimen
    public static double rwHang = 0.63; //hang specimen
    ///////////////////////////
    /////Robot Positions//////
    /////////////////////////
    public static double x0 = 35;
    public static double y0 = 7;
    public static double x1 = 22;
    public static double y1 = -34;
    public static double x2 = 15;
    public static double y2 = -27;
    public static double x3 = 28;
    public static double y3 = -40;
    public static double x4 = 15;
    public static double y4 = -25;
    public static double x5 = 25;
    public static double y5 = -52;
    public static double x6 = 16;
    public static double y6 = -20;
    public static double x7 = -8;
    public static double y7 = -24;
    public static double x8 = 12;
    public static double y8 = -15;
    public static double x9 = 38;
    public static double y9 = 22;
    public static double x10 = -8;
    public static double y10 = -24;
    public static double x11 = 38;
    public static double y11 = 30;
    public static double x12 = -8;
    public static double y12 = -30;
    public static double x13 = 37;
    public static double y13 = 20;
    public static double x14 = -8;
    public static double y14 = -30;
    public static double x15 = 37;
    public static double y15 = 35;
    public static double y16 = 30;
    public static double x16 = 3;
/////////////////////
/////Sleep vars//////
/////////////////////
    public static double hangSleep=0.5;
    public static double grabSleep=0.3;
    public static double clawSleep=0.5;
    public static double downSleep=0.1;
    public static double inSleep=0.1;
    public static double outSleep=0.5;
    public static double outSleep2=1;
    /////////////////////
    /////Speed vars//////
    /////////////////////
    public static double hangSpeed = 40;

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

        public class midClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(Cpos2);
                return false;
            }
        }
        public Action midClaw() {
            return new openClaw();
        }

        public class closeClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(Cpos3);
                return false;
            }
        }
        public Action closeClaw() {
            return new closeClaw();
        }


        public class spGrabPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                elbow.setPosition(Epos1);
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

        public class intakeInABit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                hSlide.setPosition(AHPos1);
                hSlide2.setPosition(BHPos1);
                return false;
            }
        }
        public Action intakeInABit() {
            return new intakeInABit();
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
        public class bucketMid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                bucket.setPosition(Bpos2);
                return false;
            }
        }
        public Action bucketMid() {
            return new bucketMid();
        }
        public class bucketDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                bucket.setPosition(Bpos3);
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

                .strafeToLinearHeading(new Vector2d(x0, y0), 0,  new TranslationalVelConstraint(hangSpeed));

        Action seg1 = segment1.build();

        //segment 2 - goes to 1st ground sample

        segment2 = segment1.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(140));

        Action seg2 = segment2.build();

        //segment 3 - brings 1st sample to obs zone
        segment3 = segment2.endTrajectory().fresh()

                .turn(Math.toRadians(-100));

        Action seg3 = segment3.build();

        //segment 4 - goes to 2nd sample
        segment4 = segment3.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x3, y3), Math.toRadians(140));

        Action seg4 = segment4.build();

        //segment 5 - brings 2nd sample to obs zone
        segment5 = segment4.endTrajectory().fresh()

                .turn(Math.toRadians(-100));

        Action seg5 = segment5.build();

        //segment 6 - goes to 3rd sample
        segment6 = segment5.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x5, y5), Math.toRadians(140));

        Action seg6 = segment6.build();

        //segment 7 - Brings 3rd sample to obs zone
        segment7 = segment6.endTrajectory().fresh()

                .turn(Math.toRadians(-100));

        Action seg7 = segment7.build();

        //segment 8 - go to grab 2nd specimen
        segment8 = segment7.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x7, y7), Math.toRadians(0));

        Action seg8 = segment8.build();

        //segment 9 - Nothing
//        segment9 = segment8.endTrajectory().fresh()
//
//                .strafeToLinearHeading(new Vector2d(x4, y6), Math.toRadians(0));
//
//        Action seg9 = segment9.build();

        //segment 10 - hang 2nd specimen
        segment10 = segment8.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x9, y9), Math.toRadians(0));

        Action seg10 = segment10.build();

        //segment 11 - grab 3rd specimen
        segment11 = segment10.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x10, y10), Math.toRadians(0));

        Action seg11 = segment11.build();

        //segment 12 - hang 3rd specimen
        segment12 = segment11.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x11, y11), Math.toRadians(0));

        Action seg12 = segment12.build();

        //segment 13 - grab 4th specimen
        segment13 = segment12.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x12, y12), Math.toRadians(0));

        Action seg13 = segment13.build();

        //segment 14 - hang 4th specimen
        segment14 = segment13.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x13, y13), Math.toRadians(0));

        Action seg14 = segment14.build();

        //segment 15 - grab 5th specimen
        segment15 = segment14.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x14, y14), Math.toRadians(0));

        Action seg15 = segment15.build();

        //segment 16 - hang 5th specimen (yippee!!!)
        segment16 = segment15.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x15, y15), Math.toRadians(0));

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
                //sample 1
                seg2,
                new SleepAction(downSleep),
                intake.intakeOut(),
                intake.bucketDown(),
                intake.intakeOn(),
                new SleepAction(inSleep),
                seg3,
                intake.bucketMid(),
                intake.intakeExpel(),
                new SleepAction(outSleep),
                intake.bucketUp(),
                intake.intakeIn(),
                intake.intakeOff(),
                 //new

                //sample 2
                seg4,
                intake.intakeOut(),
                new SleepAction(downSleep),
                intake.bucketDown(),
                intake.intakeOn(),
                new SleepAction(inSleep),
                seg5,
                intake.bucketMid(),
                intake.intakeExpel(),
                new SleepAction(outSleep),
                intake.bucketUp(),
                intake.intakeOff(),
                intake.intakeIn(),

                //sample 3
                seg6,
                intake.intakeOut(),
                new SleepAction(downSleep),
                intake.bucketDown(),
                intake.intakeOn(),
                new SleepAction(inSleep),
                intake.intakeInABit(),
                new SleepAction(.2),
                seg7,
                intake.bucketMid(),
                intake.intakeExpel(),
                new SleepAction(outSleep),
                intake.bucketUp(),
                intake.intakeOff(),
                intake.intakeIn(),

                intake.spGrabPos(),
                seg8,

                

                //score 2nd specimen

                intake.closeClaw(),
                new SleepAction(grabSleep),
                intake.spHangPos(),
                lift.scoreHeight(),
                seg10,
                lift.upABit(),
                new SleepAction(hangSleep),
                intake.openClaw(),
                intake.spGrabPos(),

                //score 3rd specimen
                new ParallelAction(
                        lift.baseHeight(),
                        seg11
                ),

                intake.closeClaw(),
                new SleepAction(grabSleep),
                intake.spHangPos(),
                lift.scoreHeight(),
                seg12,
                lift.upABit(),
                new SleepAction(hangSleep),

                intake.spGrabPos(),
                intake.openClaw(),

                //score 4th specimen
                new ParallelAction(
                        lift.baseHeight(),
                        seg13
                ),


                intake.closeClaw(),
                new SleepAction(grabSleep)
                /*
                intake.spHangPos(),
                lift.scoreHeight(),
                seg14,
                lift.upABit(),
                new SleepAction(hangSleep),
                lift.baseHeight(),
                intake.spGrabPos(),
                intake.openClaw(),

                //score 5th specimen
                seg15,
                intake.closeClaw(),
                new SleepAction(grabSleep),
                intake.spHangPos(),
                lift.scoreHeight(),
                seg16,
                lift.upABit(),
                new SleepAction(hangSleep),
                lift.baseHeight(),
                intake.spGrabPos(),
                intake.openClaw(),

                //park
                seg17

*/

        ));


    }

}