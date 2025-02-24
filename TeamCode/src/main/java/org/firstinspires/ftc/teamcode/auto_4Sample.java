package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
@Disabled
@Config
@Autonomous(name = "4 Sample Auto")
public class auto_4Sample extends LinearOpMode {
    /////////////////////////
    /////Mech Positions//////
    /////////////////////////
    public static int spHeight1 = 0;
    public static int saHeight2 = 2850;

    public static double AHPos = 0.06; //left  linkage in
    public static double BHPos = 0.035; //right linkage in
    public static double AHPos3 = 0.43; //left linkage out
    public static double BHPos3 = 0.245; //right linkage out
    public static double Bpos = 0.29;
    public static double Bpos2 = 0.35;
    public static double Epos1 = 0.81; //Origin Pickuo
    public static double Epos2 = .71; //Origin
    public static double Epos3 = 0.4; //Sample
    public static double Cpos = 0.57; //open
    public static double Cpos2 = 0.75; //closed
    public static double rwIn = .2;
    public static double lwIn = .24;
    public static double lwPos3 = 0.5;
    public static double rwPos3 = 0.33;
    public static double armOut= .4;

    ///////////////////////////
    /////Robot Positions//////
    /////////////////////////
    public static double x0 = 10;
    public static double y0 = 22;
    public static double x1 = 27;
    public static double y1 = 7; //9
    public static double x2 = 7;
    public static double y2 = 22;
    public static double x3 = 25;
    public static double y3 = 24; //24
    public static double x4 = 7;
    public static double y4 = 19;
    public static double x5 = 30;
    public static double y5 = 20;
    public static double x6 = 8;
    public static double y6 = 17.5;
    public static double x7 = 80;
    public static double y7 = -17.5;
    /////////////////////
    /////Ang vars////////
    /////////////////////
    public static double grabAng = 185;
    /////////////////////
    /////Sleep vars//////
    /////////////////////

    public static double firstLiftWait = 1;
    public static double armStabilizeWait = 0.1;
    public static double clawOpenWait = 0.3;
    public static double intakeWait = 0.8;
    public static double inWait = 0.2;
    public static double linkInWait = 0.25;
    public static double intakeInsureWait = 0.2;
    public static double armDownWait = 0.75;
    public static double clawCloseWait = 0.25;
    public static double secondLiftWait = 0.85;
    public static double thirdLiftWait = 1;
    public static double lastIntakeWait = 1;
    public static double lastLiftWait = 1;
    public static double parkWait = 1;



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

                    intake.setPower(0.5);




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



        //segment 1 - score 1st sample
        segment1 = drive.actionBuilder(initialPose)

                .strafeToLinearHeading(new Vector2d(x0, y0),  Math.toRadians(135));

        Action seg1 = segment1.build();

        //segment 2 - grab 2nd sample

        segment2 = segment1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(grabAng));


        Action seg2 = segment2.build();

        //segment 3 - score 2nd sample
        segment3 = segment2.endTrajectory().fresh()
        .strafeToLinearHeading(new Vector2d(x2,y2),Math.toRadians(135));

        Action seg3 = segment3.build();

        //segment 4 - grab 3rd sample
        segment4 = segment3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x3, y3),Math.toRadians(grabAng));

        //segment 5 - score 3rd sample
        Action seg4 = segment4.build();
        segment5 = segment4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x4,y4),Math.toRadians(135));

        Action seg5 = segment5.build();
        //segment 6 - grab 4th sample
        segment6 = segment5.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x5,y5),Math.toRadians(230));
        Action seg6 = segment6.build();
        //segment 7 - score 4th sample
        segment7 = segment6.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(x6,y6),Math.toRadians(135));


        Action seg7 = segment7.build();
        //segment 6 - park
        segment8 = segment7.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x7,y7),Math.toRadians(-90));

        Action seg8 = segment8.build();

        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        if (isStopRequested()) return;


        Actions.runBlocking(new SequentialAction(
                //Score 1st sample
                Mechs.slideIn(),
                Mechs.closeClaw(),
                lift.scoreHeight(),
                Mechs.saScorePos(),
                new SleepAction(firstLiftWait), //.75 //.25
                seg1,
                new SleepAction(armStabilizeWait), //.5
                Mechs.openClaw(),
                new SleepAction(clawOpenWait), //.5
                Mechs.Return(),
                lift.baseHeight(),

                //grab 2nd sample
                seg2,
                Mechs.slideOut(),
                new SleepAction(intakeWait),
                Mechs.intakeOff(),
                new SleepAction(inWait),
                Mechs.slideIn(),
                new SleepAction(linkInWait),
                Mechs.intakeOn(),
                new SleepAction(intakeInsureWait),
                Mechs.intakeOff(),
                Mechs.armDownALil(),
                new SleepAction(armDownWait),
                Mechs.closeClaw(),
                new SleepAction(clawCloseWait),


                Mechs.saScorePos(),
                lift.scoreHeight(),
                new SleepAction(secondLiftWait),
                seg3,
                new SleepAction(armStabilizeWait),
                Mechs.openClaw(),
                new SleepAction(clawOpenWait), //.3
                new ParallelAction(
                        seg4,
                        Mechs.Return(),
                        lift.baseHeight()
                ),
                Mechs.slideOut(),
                new SleepAction(intakeWait), //1
                Mechs.intakeOff(),
                new SleepAction(inWait),
                Mechs.slideIn(),
                new SleepAction(linkInWait), //.5 //.25
                Mechs.intakeOn(),
                new SleepAction(intakeInsureWait), //.1),
                Mechs.intakeOff(),
                Mechs.armDownALil(),
                new SleepAction(armDownWait),

                Mechs.closeClaw(),
                new SleepAction(clawCloseWait),//.5

                Mechs.saScorePos(),
                lift.scoreHeight(),
                new SleepAction(thirdLiftWait),
                seg5,
                new SleepAction(armStabilizeWait),
                Mechs.openClaw(),
                new SleepAction(clawOpenWait), //.3
                seg6,
                Mechs.Return(),
                lift.baseHeight(),


                Mechs.slideOut(),
                new SleepAction(lastIntakeWait),
                Mechs.intakeOff(),
                new SleepAction(inWait),
                Mechs.slideIn(),
                new SleepAction(linkInWait), //.5 //.25
                Mechs.intakeOn(),
                new SleepAction(intakeInsureWait),
                Mechs.intakeOff(),
                Mechs.armDownALil(),
                new SleepAction(armDownWait),
                Mechs.closeClaw(),
                new SleepAction(clawCloseWait), //.5

                Mechs.saScorePos(),
                lift.scoreHeight(),
                new SleepAction(lastLiftWait),
                seg7,
                new SleepAction(armStabilizeWait), //.3
                Mechs.openClaw(),
                new SleepAction(clawOpenWait), //.3
                Mechs.Return(),
                lift.baseHeight(),
                seg8,
                Mechs.park(),
                new SleepAction(parkWait)




        ));
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
    telemetry.update();

    }
}