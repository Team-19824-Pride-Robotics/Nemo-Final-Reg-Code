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
    public static int saHeight2 = 2850;
    public static int spHeight2 = 1000;


    public static int baseHeight = 0;

    public static double HPos = 0.06;

    public static double HPos2 = 0.4;

    public static double HPos3 = 0.6;

    public static double Bpos = 0.33;

    public static double Bpos2 = 0.8;

    public static double Epos1 = .28; //Originpickup
    public static double Epos2 = .4; //Origin
    public static double Epos3 = 0.7; //Specimen
    public static double Epos4 = 0.65; //Sample

    public static double Cpos = 0.79; //open

    public static double Cpos2 = 0.935; //closed

    public static double Wpos1 = 0.35;

    public static double Wpos2 = 0.66;
    public static double Wpos3 = 0.5;

    public static double x0 = 12.5;

    public static double y0 = 16;
    public static double x1 = 12.5;

    public static double y1 = 16;
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

    public static double score_angle = 135;

    
        ServoImplEx backWrist;

        ServoImplEx frontWrist;
        DcMotorEx lift1;
        DcMotorEx lift2;
        Servo claw;
        Servo elbow;
        public class Mechs {
            ServoImplEx backWrist;

            ServoImplEx frontWrist;
            DcMotorEx lift1;
            DcMotorEx lift2;
            Servo claw;
            Servo elbow;

            Servo hSlide;
            Servo hSlide2;

            Servo bucket;
            DcMotor intake;
            public Mechs(HardwareMap hardwareMap) {
                elbow = hardwareMap.servo.get("arm");
                claw = hardwareMap.servo.get("claw");
                Servo horizontalSlides1 = hardwareMap.servo.get("ll");
                Servo horizontalSlides2 = hardwareMap.servo.get("rl");
                bucket = hardwareMap.servo.get("bucket");
                intake = hardwareMap.get(DcMotor.class, "intake");
                backWrist = (ServoImplEx) hardwareMap.get(Servo.class, "lw");
                backWrist.setPwmRange(new PwmControl.PwmRange(505, 2495));
                frontWrist = (ServoImplEx) hardwareMap.get(Servo.class, "rw");
                frontWrist.setPwmRange(new PwmControl.PwmRange(505, 2495));

            }

            public class saScorePos implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    //bring claw to origin
                    elbow.setPosition(Epos3);
                    frontWrist.setPosition(Wpos3);
                    backWrist.setPosition(Wpos3);


                    return false;
                }
            }
            public Action saScorePos() {
                return new saScorePos();
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


           

            public class Return implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {


                    elbow.setPosition(Epos1);
                    frontWrist.setPosition(Wpos1);
                    backWrist.setPosition(Wpos1);

                    return false;
                }
            }
            public Action Return() {
                return new Return();
            }

            public class slideOut implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                //hSlide.setPosition(HPos3);
                //hSlide2.setPosition(HPos3);
                bucket.setPosition(Bpos2);
                elbow.setPosition(Epos2);
                intake.setPower(1);




                    return false;
                }
            }
            public Action slideOut() {
                return new slideOut();
            }

            public class slideIn implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    //hSlide.setPosition(HPos);
                    //hSlide2.setPosition(HPos);
                    bucket.setPosition(Bpos);
                    elbow.setPosition(Epos1);
                    intake.setPower(0);




                    return false;
                }
            }
            public Action slideIn() {
                return new slideIn();
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

        public class downABit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lift1.setTargetPosition(spHeight1-150);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setTargetPosition(spHeight1-150);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setPower(1);
                lift2.setPower(1);
                return false;
            }
        }
        public Action downABit() {
            return new downABit();
        }
        public class baseHeight implements Action {
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
        public Action baseHeight() {
            return new baseHeight();
        }

        public class scoreHeight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lift1.setTargetPosition(saHeight2);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setTargetPosition(saHeight2);
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
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make an Mechs instance
        Mechs Mechs = new Mechs(hardwareMap);
        lift lift = new lift(hardwareMap);
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

                .strafeToLinearHeading(new Vector2d(x0, y0),  Math.toRadians(135));

        Action seg1 = segment1.build();

        //segment 2 - strafe behind first block

        segment2 = segment1.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(180));

        Action seg2 = segment2.build();
//
//        //segment 2.5 - strafe back to buckets
//
//        segment2_5 = segment2.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(x0, y0) , Math.toRadians(135));
//
//        Action seg2_5 = segment2_5.build();
//
//        //segment 3 - strafe to 2nd block
//        segment3 = segment2_5.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(x1, y2), Math.toRadians(0));
//
//        Action seg3 = segment3.build();
//
//        //segment 4 - strafe back to bucket
//        segment4 = segment3.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(x0, y0) , Math.toRadians(135));
//
//        Action seg4 = segment4.build();
//
//        //segment 5 - strafe to 3rd block
//        segment5 = segment4.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(x1, y2), Math.toRadians(45));//might not be y2
//
//        Action seg5 = segment5.build();
//
//        //segment 6 - strafe back to bucket
//        segment6 = segment5.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(x0, y0) , Math.toRadians(135));
//        Action seg6 = segment6.build();
//
//        //segment 7 - park
//        segment7 = segment6.endTrajectory().fresh()
//                .strafeToLinearHeading(new Vector2d(x2, y3) , Math.toRadians(-45));
//        Action seg7 = segment7.build();
        waitForStart();




        if (isStopRequested()) return;


        Actions.runBlocking(new SequentialAction(

                Mechs.closeClaw(),
                lift.scoreHeight(),
                Mechs.saScorePos(),
                new SleepAction(0.5),
                seg1,
                Mechs.openClaw(),
                new SleepAction(1),
                Mechs.Return(),
                lift.baseHeight(),
                seg2
//                Mechs.slideOut(),
//                new SleepAction(2),
//                Mechs.slideIn(),
//                new SleepAction(1)
//                seg2,
//                Mechs.slideOut(),
//                new SleepAction(2),
//                Mechs.slideIn(),
//                new SleepAction(1),
//                Mechs.closeClaw(),
//                lift.scoreHeight(),
//                Mechs.saScorePos(),
//                seg2_5,
//                Mechs.openClaw(),
//                Mechs.Return(),
//                lift.baseHeight(),
//
//                seg3,
//                Mechs.slideOut(),
//                new SleepAction(2),
//                Mechs.slideIn(),
//                new SleepAction(1),
//                Mechs.closeClaw(),
//                lift.scoreHeight(),
//                Mechs.saScorePos(),
//                seg4,
//                Mechs.openClaw(),
//                Mechs.Return(),
//                lift.baseHeight(),
//
//                seg5,
//                Mechs.slideOut(),
//                new SleepAction(2),
//                Mechs.slideIn(),
//                new SleepAction(1),
//                Mechs.closeClaw(),
//                lift.scoreHeight(),
//                Mechs.saScorePos(),
//                seg6,
//                Mechs.openClaw(),
//                Mechs.Return(),
//                lift.baseHeight(),
//                seg7


        ));


    }
}