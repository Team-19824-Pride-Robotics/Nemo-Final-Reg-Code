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

    public static double AHPos = 0.07;
    public static double BHPos = 0.13;
    public static double AHPos3 = 0.9;
    public static double BHPos3 = 0.9;

    public static double Bpos = 0.32;

    public static double Bpos2 = 0.37;

    public static double Epos1 = .3; //Originpickup
    public static double Epos2 = .4; //Origin
    public static double Epos3 = 0.7; //Specimen
    public static double Epos4 = 0.65; //Sample

    public static double Cpos = 0.73; //open

    public static double Cpos2 = 0.935; //closed

    public static double rwIn = .33;
    public static double lwIn = .37;
    public static double lwPos3 = 0.5;

    public static double rwPos3 = 0.46;

    public static double x0 = 12.5;

    public static double y0 = 16;
    public static double x1 = 5;

    public static double y1 = 6;

    public static double x2 = 10;

    public static double y2 = 15;







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

                    intake.setPower(0);




                    return false;
                }
            }
            public Action slideIn() {
                return new slideIn();
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

//segment 1 - strafe to bucket
        segment1 = drive.actionBuilder(initialPose)

                .strafeToLinearHeading(new Vector2d(x0, y0),  Math.toRadians(135));

        Action seg1 = segment1.build();

        //segment 2 - strafe behind first block

        segment2 = segment1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(180));


        Action seg2 = segment2.build();

        segment3 = segment2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x2, y2), Math.toRadians(135));


        Action seg3 = segment3.build();

        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        if (isStopRequested()) return;


        Actions.runBlocking(new SequentialAction(

                Mechs.closeClaw(),
                lift.scoreHeight(),
                new SleepAction(0.75),
                seg1,
                Mechs.saScorePos(),
                new SleepAction(0.5),
                Mechs.openClaw(),
                new SleepAction(1),
                Mechs.Return(),
                lift.baseHeight(),

                seg2,
                Mechs.slideOut(),
                new SleepAction(2),
                Mechs.slideIn(),
                new SleepAction(1),
                Mechs.armDownALil(),
                new SleepAction(0.5),
                Mechs.closeClaw(),
                new SleepAction(0.5),


                Mechs.saScorePos(),
                lift.scoreHeight(),
                seg3,
                Mechs.openClaw(),
                new SleepAction(1),
                Mechs.Return(),
                lift.baseHeight(),
                new SleepAction(3)



        ));
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
    telemetry.update();

    }
}