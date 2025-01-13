package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
@Autonomous(name = "Sample auto 2")
public class sample_auto2 extends LinearOpMode {
    //Positions copied from Teleop
    public static int saHeight1 = 1300;
    public static int spHeight1 = 0;
    public static int saHeight2 = 2850;
    public static int spHeight2 = 1000;


    public static int baseHeight = 0;

    public static double AHPos = 0.06;
    public static double BHPos = 0.12;
    public static double AHPos3 = 0.27;
    public static double BHPos3 = 0.45;

    public static double Bpos = 0.32;

    public static double Bpos2 = 0.37;

    public static double Epos1 = .31; //Originpickup
    public static double Epos2 = .4; //Origin
    public static double Epos3 = 0.7; //Specimen
    public static double Epos4 = 0.65; //Sample

    public static double Cpos = 0.73; //open

    public static double Cpos2 = 0.935; //closed

    public static double rwIn = .33;
    public static double lwIn = .37;
    public static double lwPos3 = 0.5;

    public static double rwPos3 = 0.46;
    public static double armOut= .66;

    public static double x0 = 14;

    public static double y0 = 16;
    public static double x1 = 13.5;

    public static double y1 = 9; //9

    public static double x2 = 1;

    public static double y2 = 12;
    public static double x3 = 8;

    public static double y3 = 24; //24
    public static double x4 = 1;

    public static double y4 = 15;

    public static double x5 = 61;
    public static double y5 = -4;
    public static double x6 = 56;
    public static double y6 = -19;
    public static double h6 = 270;


    public static double tangent1 = 180;
    public static double tangent2 = -90;

public static double turn = 45;



        public class Mechs {
            ServoImplEx backWrist;

            ServoImplEx frontWrist;

            Servo claw;
            ServoImplEx elbow;

            ServoImplEx hSlide;
            ServoImplEx hSlide2;

            ServoImplEx bucket;
            DcMotor intake;
            public Mechs(HardwareMap hardwareMap) {
                elbow = (ServoImplEx)hardwareMap.servo.get("arm");
                elbow.setPwmRange(new PwmControl.PwmRange(505, 2495));
                claw = hardwareMap.servo.get("claw");
                hSlide = (ServoImplEx)hardwareMap.servo.get("ll");
                hSlide.setPwmRange(new PwmControl.PwmRange(505, 2495));
                hSlide2 = (ServoImplEx)hardwareMap.servo.get("rl");
                hSlide2.setPwmRange(new PwmControl.PwmRange(505, 2495));
                bucket = (ServoImplEx)hardwareMap.servo.get("bucket");
                bucket.setPwmRange(new PwmControl.PwmRange(505, 2495));
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
                    frontWrist.setPosition(rwPos3);
                    backWrist.setPosition(lwPos3);


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
                    frontWrist.setPosition(rwIn);
                    backWrist.setPosition(lwIn);

                    return false;
                }
            }
            public Action Return() {
                return new Return();
            }

            public class park implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {


                    elbow.setPosition(armOut);
                    frontWrist.setPosition(rwPos3);
                    backWrist.setPosition(lwPos3);

                    return false;
                }
            }
            public Action park() {
                return new park();
            }

            public class slideOut implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                hSlide.setPosition(AHPos3);
                hSlide2.setPosition(BHPos3);
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
                    hSlide.setPosition(AHPos);
                    hSlide2.setPosition(BHPos);
                    bucket.setPosition(Bpos);






                    return false;
                }
            }
            public Action slideIn() {
                return new slideIn();
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

                    intake.setPower(0.4);




                    return false;
                }
            }
            public Action intakeOn() {
                return new intakeOn();
            }
            public class armDownALil implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    elbow.setPosition(Epos1);





                    return false;
                }
            }
            public Action armDownALil() {
                return new armDownALil();
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
        Mechs Mechs = new Mechs(hardwareMap);
        lift lift = new lift(hardwareMap);

        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);




        TrajectoryActionBuilder segment1;
        TrajectoryActionBuilder segment2;
        TrajectoryActionBuilder segment3;
        TrajectoryActionBuilder segment4;
        TrajectoryActionBuilder segment5;
        TrajectoryActionBuilder segment6;
        TrajectoryActionBuilder segment7;
        TrajectoryActionBuilder segment8;



//segment 1 - strafe to bucket
        segment1 = drive.actionBuilder(initialPose)

                .strafeToLinearHeading(new Vector2d(x0, y0),  Math.toRadians(135));

        Action seg1 = segment1.build();

        //segment 2 - strafe behind first block

        segment2 = segment1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(180));


        Action seg2 = segment2.build();
//Strafe back to buckets
        segment3 = segment2.endTrajectory().fresh()
                //.splineToLinearHeading(new Vector2d(x0, y2), Math.toRadians(135))
                //.setTangent(Math.toRadians(180))
        .splineToLinearHeading(new Pose2d(x2,y2,Math.toRadians(135)),Math.toRadians(135));

        Action seg3 = segment3.build();

        segment4 = segment3.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(x3,y3,Math.toRadians(180)),Math.toRadians(180));

        Action seg4 = segment4.build();
        segment5 = segment4.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(x4,y4,Math.toRadians(135)),Math.toRadians(135));

        Action seg5 = segment5.build();
        segment6 = segment5.endTrajectory().fresh()
                .turn(Math.toRadians(turn));
        Action seg6 = segment6.build();
        segment7 = segment6.endTrajectory().fresh()
                //.setTangent(Math.toRadians(tangent1))
                .lineToX(x5);
        //.splineToLinearHeading(new Pose2d(x5,y5,Math.toRadians(thirdScoreAngle)),Math.toRadians(tangent2));

        Action seg7 = segment7.build();
        segment8 = segment7.endTrajectory().fresh()
                .setTangent(Math.toRadians(tangent1))

        .splineToLinearHeading(new Pose2d(x6,y6,Math.toRadians(h6)),Math.toRadians(tangent2));

        Action seg8 = segment8.build();

        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        if (isStopRequested()) return;


        Actions.runBlocking(new SequentialAction(
                Mechs.slideIn(),
                Mechs.closeClaw(),
                lift.scoreHeight(),
                Mechs.saScorePos(),
                new SleepAction(0.75),
                seg1,
                new SleepAction(0.5),
                Mechs.openClaw(),
                new SleepAction(1),
                Mechs.Return(),
                lift.baseHeight(),

                seg2,
                Mechs.slideOut(),
                new SleepAction(1),
                Mechs.intakeOff(),
                new SleepAction(0.25),
                Mechs.slideIn(),
                new SleepAction(0.5),
                Mechs.intakeOn(),
                new SleepAction(0.25),
                Mechs.intakeOff(),
                Mechs.armDownALil(),
                new SleepAction(0.5),
                Mechs.closeClaw(),
                new SleepAction(0.5),


                Mechs.saScorePos(),
                lift.scoreHeight(),
                new SleepAction(1.1),
                seg3,
                Mechs.openClaw(),
                new SleepAction(1),
                Mechs.Return(),
                lift.baseHeight(),

                seg4,
                Mechs.slideOut(),
                new SleepAction(1),
                Mechs.intakeOff(),
                new SleepAction(0.25),
                Mechs.slideIn(),
                new SleepAction(0.5),
                Mechs.intakeOn(),
                new SleepAction(0.25),
                Mechs.intakeOff(),
                new SleepAction(0.5),
                Mechs.armDownALil(),
                new SleepAction(0.5),
                Mechs.closeClaw(),
                new SleepAction(0.5),

                Mechs.saScorePos(),
                lift.scoreHeight(),
                new SleepAction(1.5),
                seg5,
                Mechs.openClaw(),
                new SleepAction(1),
                Mechs.Return(),
                lift.baseHeight(),
                //new SleepAction(2),
                seg6,
                seg7,
                seg8,
                Mechs.park(),
                new SleepAction(2)

//                seg6,
//                Mechs.slideOut(),
//                new SleepAction(1.5),
//                Mechs.intakeOff(),
//                new SleepAction(0.25),
//                Mechs.slideIn(),
//                new SleepAction(0.5),
//                Mechs.intakeOn(),
//                new SleepAction(0.25),
//                Mechs.intakeOff(),
//                new SleepAction(0.5),
//                Mechs.armDownALil(),
//                new SleepAction(0.5),
//                Mechs.closeClaw(),
//                new SleepAction(0.5),
//
//                Mechs.saScorePos(),
//                lift.scoreHeight(),
//                new SleepAction(1.5)

        ));
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
    telemetry.update();

    }
}