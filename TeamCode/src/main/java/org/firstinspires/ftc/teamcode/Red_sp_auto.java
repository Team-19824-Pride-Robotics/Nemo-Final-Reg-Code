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
import com.arcrobotics.ftclib.controller.PIDController;
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
@Autonomous(name = "SpecimenAuto")
public class Red_sp_auto extends LinearOpMode {
    //Positions copied from Teleop
    public static int saHeight1 = 1300;
    public static int spHeight1 = 0;
    public static int saHeight2 = 3100;
    public static int spHeight2 = 1000;

    public static double p = 0.005, i = 0, d = 0;
    public static double f = 0;
    public static double target = 0;
    public static double target2 = 110;
    public static double target1 = 110;

    private PIDController controller;
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
    public static double x1 = 25;
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

    public class Intake {
        ServoImplEx backWrist;

        ServoImplEx frontWrist;
        DcMotorEx lift1;
        DcMotorEx lift2;
        Servo claw;
        Servo elbow;
        public Intake(HardwareMap hardwareMap) {
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

        public class spHangPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //bring claw to origin
                elbow.setPosition(Epos4);
                frontWrist.setPosition(Wpos2);
                backWrist.setPosition(Wpos2);
                target=spHeight2;
                controller.setPID(p, i, d);
                int liftPos1 = lift1.getCurrentPosition();
                int liftPos2 = lift2.getCurrentPosition();
                double pid = controller.calculate(liftPos1, target);
                double pid2 = controller.calculate(liftPos2, target);
                double ff = 0;

                double lPower1 = pid + ff;
                double lPower2 = pid2 + ff;

                lift1.setPower(lPower1);
                lift2.setPower(lPower2);
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

                elbow.setPosition(Epos4);
                frontWrist.setPosition(Wpos2);
                backWrist.setPosition(Wpos2);
                target=spHeight1;
                controller.setPID(p, i, d);
                int liftPos1 = lift1.getCurrentPosition();
                int liftPos2 = lift2.getCurrentPosition();
                double pid = controller.calculate(liftPos1, target);
                double pid2 = controller.calculate(liftPos2, target);
                double ff = 0;

                double lPower1 = pid + ff;
                double lPower2 = pid2 + ff;

                lift1.setPower(lPower1);
                lift2.setPower(lPower2);

                return false;
            }
        }
        public Action spGrabPos() {
            return new spGrabPos();
        }

        public class Return implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {


                    elbow.setPosition(Epos1);
                    frontWrist.setPosition(Wpos2);
                    backWrist.setPosition(Wpos2);
                    claw.setPosition(Cpos2);
                target=spHeight1;
                controller.setPID(p, i, d);
                int liftPos1 = lift1.getCurrentPosition();
                int liftPos2 = lift2.getCurrentPosition();
                double pid = controller.calculate(liftPos1, target);
                double pid2 = controller.calculate(liftPos2, target);
                double ff = 0;

                double lPower1 = pid + ff;
                double lPower2 = pid2 + ff;

                lift1.setPower(lPower1);
                lift2.setPower(lPower2);
                return false;
            }
        }
        public Action Return() {
            return new Return();
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
        Intake intake = new Intake(hardwareMap);
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

        segment1 = drive.actionBuilder(initialPose)
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(x0, 0));

        Action seg1 = segment1.build();

        //segment 2 - backs off the sub

        segment2 = segment1.endTrajectory().fresh()
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(x1, 0));

        Action seg2 = segment2.build();

        //segment 2.5 - strafes right to clear the sub
        // parallel with lift to pickup position

        segment2_5 = segment2.endTrajectory().fresh()
                .strafeTo(new Vector2d(x0, y2));

        Action seg2_5 = segment2_5.build();

        //segment 3 - moves on a diagonal to get behind the sample
        segment3 = segment2_5.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x3, y6), Math.toRadians(180));

        Action seg3 = segment3.build();

        //segment 4 - push a sample into the obs zone
        segment4 = segment3.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(x4, y6));

        Action seg4 = segment4.build();

        //segment 5 - push two samples into the zone
        segment5 = segment4.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(x13, y15));

        Action seg5 = segment5.build();

//        segment5_5 = segment5.endTrajectory().fresh()
//                .strafeToConstantHeading(new Vector2d(x4, y6));
//
//        Action seg5_5 = segment5_5.build();

        //segment 6 - slowly! to pick up the specimen
        segment6 = segment5.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(x5,y15), new TranslationalVelConstraint(pickup_speed));

        Action seg6 = segment6.build();

        //segment 7 - strafe back to the sub with a 180
        //parallel with lift to scoring position
        segment7 = segment6.endTrajectory().fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(x9, y11), Math.toRadians(0));

        Action seg7 = segment7.build();

        //segment 7.5 - scoring second specimen
        segment7_5 = segment7.endTrajectory().fresh()
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(x0, y11));

        Action seg7_5 = segment7_5.build();

        segment7_6 = segment7_5.endTrajectory().fresh()
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(x1, 0));

        Action seg7_6 = segment7_6.build();

        //segment 8 - strafe path back to the zone with a 180
        // parallel with lift to pickup position
        segment8 = segment7_6.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x4, y6), Math.toRadians(180));

        Action seg8 = segment8.build();

//segment 8_5 - slowly! to pick up the specimen
        segment8_5 = segment8.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(x5,y6));

        Action seg8_5 = segment8_5.build();
//
        segment9 = segment8_5.endTrajectory().fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(x9, y9), Math.toRadians(0));

        Action seg9 = segment9.build();
//Turn Around and go to put specimen on the bar
        segment10 = segment9.endTrajectory().fresh()

                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(x0, y9));

        Action seg10 = segment10.build();
//goes back and to the right in anticipation of pushing the block
        segment11 = segment10.endTrajectory().fresh()

                .strafeToConstantHeading(new Vector2d(x10, y12));

        Action seg11 = segment11.build();

        segment12 = segment11.endTrajectory().fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(x11, y17), Math.toRadians(0));

        Action seg12 = segment12.build();

        segment13 = segment12.endTrajectory().fresh()

                .strafeToConstantHeading(new Vector2d(x12, y18));

        Action seg13 = segment13.build();

        segment14 = segment13.endTrajectory().fresh()

                .strafeToConstantHeading(new Vector2d(x12, y18));

        Action seg14 = segment14.build();


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
                intake.closeClaw(),
               // new ParallelAction(
                        seg7,
                 //       intake.spHangPos()
                //),
                seg7_5,
                intake.closeClaw(),
                seg7_6,
                //new ParallelAction(
                        seg8,
                  //      intake.spGrabPos()
                //),
                seg8_5,
                //new ParallelAction(
                        seg9,
                  //      intake.spHangPos()
                //),
                seg10,
                intake.openClaw(),
                seg11
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