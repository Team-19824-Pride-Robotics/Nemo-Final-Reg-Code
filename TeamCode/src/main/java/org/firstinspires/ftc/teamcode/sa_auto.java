package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;



// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@Autonomous(name = "Sample auto")
public class sa_auto extends LinearOpMode {
    //Positions copied from Teleop
    public static int saHeight1 = 1300;
    public static int spHeight1 = 0;
    public static int saHeight2 = 3100;
    public static int spHeight2 = 1000;


    public static int baseHeight = 0;

    public static double HPos = 0.06;

    public static double HPos2 = 0.4;

    public static double HPos3 = 0.6;

    public static double Bpos = 0.33;

    public static double Bpos2 = 0.8;

    public static double Epos1 = 0.95;
    public static double Epos2 = 0.8;
    public static double Epos3 = 0.75;
    public static double Epos4 = 0.52;

    public static double Cpos = 1;

    public static double Cpos2 = 0.7;

    public static double Wpos1 = 0.3;

    public static double Wpos2 = 0.04;
    public static double Wpos3 = 0.7;

    public static double x0 = 27;

    public static double y0 = 0;
    public static double x1 = 25;

    public static double y1 = 0;
    public static double x2 = 11;
    public static double y2 = -25;
    public static double x3 = 60;
    public static double x8 = 60;
    public static double y6 = -35;
    public static double x4 = 3;
    public static double y3 = -55;

    public static double x5 = 7;
    public static double x6 = 6;
    public static double y4 = -2;

    public static double x7 = 8.5;
    public static double y5 = -18.5;
    public static double x9 = 20;
    public static double y9 = 0;
    public static double y11 = 5;
    public static double x10 = 15;
    public static double y12 = 0;
    public static double y13 = 55;
    public static double x11 = 60;
    public static double x12 = 10;
    public static double y14 = 52.5;
    public static double y15 = -33;
    public static double x13 = 12.5;
    public static double y16 = 70;
    public static double y17 = -54;
    public static double y18 = -54;
    public static double pickup_speed = 5;

    public class Mechs {
        ServoImplEx backWrist;

        ServoImplEx frontWrist;
        DcMotorEx lift1;
        DcMotorEx lift2;
        Servo claw;
        Servo elbow;
        public Mechs(HardwareMap hardwareMap) {
            elbow = hardwareMap.servo.get("armElbow");
            claw = hardwareMap.servo.get("claw");
            lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
            lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
            lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift1.setDirection(DcMotorEx.Direction.REVERSE);
            backWrist = (ServoImplEx) hardwareMap.get(Servo.class, "backWrist");
            backWrist.setPwmRange(new PwmControl.PwmRange(505, 2495));
            frontWrist = (ServoImplEx) hardwareMap.get(Servo.class, "frontWrist");
            frontWrist.setPwmRange(new PwmControl.PwmRange(505, 2495));
        }

        public class outakePos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //bring claw to origin
                elbow.setPosition(Epos2);
                    frontWrist.setPosition(Wpos3);
                    backWrist.setPosition(Wpos3);


                return false;
            }
        }
        public Action outakePos() {
            return new outakePos();
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


        public class originPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                return false;
            }
        }
        public Action originPos() {
            return new originPos();
        }

        public class Intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {



                return false;
            }
        }
        public Action Intake() {
            return new Intake();
        }



    }


    public class Lift {

        private DcMotorEx liftMotor1;
        private DcMotorEx liftMotor2;
        private Servo specArmServo;
        private Servo bucketServo;

        public Lift(HardwareMap hardwareMap) {

            liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
            liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor2.setDirection(DcMotorEx.Direction.REVERSE);

            specArmServo = hardwareMap.get(Servo.class,"specArmServo");
            bucketServo = hardwareMap.get(Servo.class, "bucketServo");

        }



    }



    @Override
    public void runOpMode(){


        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make an Intake instance
        Mechs intake = new Mechs(hardwareMap);
        // make a Lift instance



        TrajectoryActionBuilder segment1;
        TrajectoryActionBuilder segment2;
        TrajectoryActionBuilder segment2_5;
        TrajectoryActionBuilder segment3;
        TrajectoryActionBuilder segment4;
        TrajectoryActionBuilder segment5;
        TrajectoryActionBuilder segment6;
        TrajectoryActionBuilder segment7;
        TrajectoryActionBuilder segment7_5;
        TrajectoryActionBuilder segment7_6;
        TrajectoryActionBuilder segment8;
        TrajectoryActionBuilder segment8_5;
        TrajectoryActionBuilder segment9;
        TrajectoryActionBuilder segment10;
        TrajectoryActionBuilder segment11;
        TrajectoryActionBuilder segment12;
        TrajectoryActionBuilder segment13;
        TrajectoryActionBuilder segment14;
//segment 1 - strafe to bucket
        segment1 = drive.actionBuilder(initialPose)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(x0, y0),  Math.toRadians(45));

        Action seg1 = segment1.build();

        //segment 2 - strafe behind first block

        segment2 = segment1.endTrajectory().fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(0));

        Action seg2 = segment2.build();

        //segment 2.5 - strafe back to buckets

        segment2_5 = segment2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x0, y0) , Math.toRadians(45));

        Action seg2_5 = segment2_5.build();

        //segment 3 - strafe to 2nd block
        segment3 = segment2_5.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x1, y2), Math.toRadians(0));

        Action seg3 = segment3.build();

        //segment 4 - strafe back to bucket
        segment4 = segment3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x0, y0) , Math.toRadians(45));

        Action seg4 = segment4.build();

        //segment 5 - strafe to 3rd block
        segment5 = segment4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x1, y3), Math.toRadians(0));

        Action seg5 = segment5.build();

//        segment5_5 = segment5.endTrajectory().fresh()
//                .strafeToConstantHeading(new Vector2d(x4, y6));
//
//        Action seg5_5 = segment5_5.build();

        //segment 6 - strafe back to bucket
        segment6 = segment5.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x0, y0) , Math.toRadians(45));
        Action seg6 = segment6.build();


        waitForStart();




        if (isStopRequested()) return;


        Actions.runBlocking(new SequentialAction(


                //new ParallelAction(
                //intake.closeClaw(),
                seg1,
                //intake.spHangPos()
                //),
                intake.openClaw(),
                seg2,
                seg2_5,
                seg3,
                seg4,
                seg5,
                // new ParallelAction(
                seg6,
                //intake.spGrabPos()
                //),
                new SleepAction(1),
                intake.closeClaw()
                // new ParallelAction(

                //
//                segment6,
//
//                lift.specimenScoreHeight(),  //this takes the specimen off the wall
//
//                new SleepAction(lift_time),  //it needs time to go up before driving away
//
//                new ParallelAction(
//                        segment7,
//                        lift.specArmScore()
//                ),
//
//                new ParallelAction(
//                        segment8,
//                        lift.specimenPickupHeight(),
//                        lift.specArmPickup()
//                ),
//
//                segment6,
//
//                lift.specimenScoreHeight(),  //this takes the specimen off the wall
//
//                new SleepAction(lift_time),  //it needs time to go up before driving away
//
//                new ParallelAction(
//                        segment7,
//                        lift.specArmScore()
//                ),
//
//                new ParallelAction(
//                        segment8,
//                        lift.specimenPickupHeight(),
//                        lift.specArmPickup()
//                ),
//
//                segment6,
//
//                lift.specimenScoreHeight(),  //this takes the specimen off the wall
//
//                new SleepAction(lift_time),  //it needs time to go up before driving away
//
//                new ParallelAction(
//                        segment7,
//                        lift.specArmScore()
//                )
//
//

        ));


    }
}