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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystem.LinkageSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.armSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.bucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.clawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.liftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.wristSubsystem;

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
    public static double AHPos2 = 0.16; //left linkage out a lil
    public static double BHPos2 = 0.135; //right linkage out a lil
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
    private LinkageSubsystem linkage;
    private liftSubsystem lift;
    private intakeSubsystem intake;
    private bucketSubsystem bucket;
    private clawSubsystem claw;
    private armSubsystem arm;
    private wristSubsystem wrist;
    ///////////////////////////
    /////Robot Positions//////
    /////////////////////////
    public static double x0 = 10;
    public static double y0 = 25;
    public static double x1 = 20;
    public static double y1 = 5.5; //9
    public static double x2 = 5;
    public static double y2 = 23;
    public static double x3 = 23;
    public static double y3 = 23.5; //24
    public static double x4 = 4;
    public static double y4 = 22;
    public static double x5 = 27;
    public static double y5 = 13;
    public static double x6 = 6;
    public static double y6 = 22;
    public static double x7 = 80;
    public static double y7 = -17.5;
    public double xSub = 80.1;
    public double ySub = -7;
    public static double x9 = 80;
    public static double y9 = -17.5;


    /////////////////////
    /////Ang vars////////
    /////////////////////
    public static double grabAng = 185;
    public double subAngle = -90;
    /////////////////////
    /////Sleep vars//////
    /////////////////////

    public static double firstLiftWait = 1;
    public static double armStabilizeWait = 0.1;
    public static double clawOpenWait = 0.3;

    public static double slideWait = 0.3;
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
public boolean exit=false;
public boolean fifthSamp=false;

        public class Mechs {

            private LinkageSubsystem linkage;
            private liftSubsystem lift;
            private intakeSubsystem intake;
            private bucketSubsystem bucket;
            private clawSubsystem claw;
            private armSubsystem arm;
            private wristSubsystem wrist;
            private Limelight3A limelight;
        public Mechs(HardwareMap hardwareMap){
            linkage = new LinkageSubsystem(hardwareMap);
            lift = new liftSubsystem(hardwareMap);
            intake = new intakeSubsystem(hardwareMap);
            bucket = new bucketSubsystem(hardwareMap);
            claw = new clawSubsystem(hardwareMap);
            arm = new armSubsystem(hardwareMap);
            wrist = new wristSubsystem(hardwareMap);
        }
            public class saScorePos implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    //bring claw to origin
                    arm.armSample();
                    wrist.wristScore();
                    arm.update();
                    wrist.update();

                    return false;
                }
            }
            public Action saScorePos() {
                return new saScorePos();
            }

            public class openClaw implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    claw.clawOpen();
                    claw.update();
                    return false;
                }
            }
            public Action openClaw() {
                return new openClaw();
            }



            public class closeClaw implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    claw.clawClose();
                    claw.update();
                    return false;
                }
            }
            public Action closeClaw() {
                return new closeClaw();
            }


           

            public class Return implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {


                    arm.armPickup2();
                    wrist.wristPickup2();
                    arm.update();
                    wrist.update();
                    return false;
                }
            }
            public Action Return() {
                return new Return();
            }


            public class park implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    arm.armPark();
                    wrist.wristScoreSpeicmen();
                    arm.update();
                    wrist.update();




                    return false;
                }
            }
            public Action park() {
                return new park();
            }
            public class slideOut implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                linkage.linkOut();
                bucket.bucketAlmostDown();
                arm.armPickup();
                wrist.wristPickup();
                bucket.coverClose();
                bucket.update();
                arm.update();
                wrist.update();





                    return false;
                }
            }
            public Action slideOut() {
                return new slideOut();
            }
            public class intakeOn implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    intakeSubsystem.intakeIn=1;
                    intakeSubsystem.intakeOut=0;
                    intake.intakeSetPower();
                    intake.update();
                    return false;
                }
            }
            public Action intakeOn() {
                return new intakeOn();
            }
            public class bucketWayUp implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    bucket.bucketEjec();
                    bucket.update();
                    return false;
                }
            }
            public Action bucketWayUp() {
                return new bucketWayUp();
            }
            public class coverOpen implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    bucket.coverOpen();
                    bucket.update();
                    return false;
                }
            }
            public Action coverOpen() {
                return new coverOpen();
            }
            public class slideIn implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    linkage.linkIn();
                    bucket.bucketUp();
                    intakeSubsystem.intakeIn=0;
                    intakeSubsystem.intakeOut=0;
                    bucket.update();
                    return false;
                }
            }
            public Action slideIn() {
                return new slideIn();
            }
            public class intakeOff implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    intakeSubsystem.intakeIn=0;
                    intakeSubsystem.intakeOut=0;
                    intake.intakeSetPower();
                    bucket.bucketUp();
                    intake.update();
                    bucket.update();




                    return false;
                }
            }
            public Action intakeOff() {
                return new intakeOff();
            }
            public class intakeALil implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {

                    intakeSubsystem.intakeIn=0.3;
                    intakeSubsystem.intakeOut=0;
                    intake.intakeSetPower();
                    intake.update();




                    return false;
                }
            }
            public Action intakeALil() {
                return new intakeALil();
            }
            public class armDownALil implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    arm.armPickup2();
                    wrist.wristPickup2();
                    arm.update();
                    wrist.update();





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

        public class maybeScoreHeight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (fifthSamp) {
                    lift1.setTargetPosition(spHeight1);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setTargetPosition(spHeight1);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift1.setPower(1);
                    lift2.setPower(1);
                }
                return false;
            }
        }

            public Action maybeScoreHeight() {
                return new maybeScoreHeight();
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
        TrajectoryActionBuilder segment9;
        TrajectoryActionBuilder segment10;



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
        //segment 8 - park
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
                Mechs.coverOpen(),
                new SleepAction(firstLiftWait), //.75 //.25
                Mechs.saScorePos(),
                seg1,
                new SleepAction(armStabilizeWait), //.5
                Mechs.openClaw(),
                new SleepAction(clawOpenWait), //.5
                Mechs.Return(),
                lift.baseHeight(),

                //grab 2nd sample
                seg2,
                Mechs.slideOut(),
                new SleepAction(slideWait),
                Mechs.intakeOn(),
                new SleepAction(intakeWait),
                Mechs.intakeOff(),
                //Mechs.bucketWayUp(),
                new SleepAction(inWait),
                Mechs.slideIn(),
                new SleepAction(linkInWait),
                Mechs.intakeALil(),
                new SleepAction(intakeInsureWait),
                Mechs.intakeOff(),
                Mechs.armDownALil(),
                new SleepAction(armDownWait),
                Mechs.closeClaw(),
                new SleepAction(clawCloseWait),

                //score 2nd sample
                lift.scoreHeight(),
                Mechs.coverOpen(),
                new SleepAction(secondLiftWait),
                Mechs.saScorePos(),
                seg3,
                new SleepAction(armStabilizeWait),
                Mechs.openClaw(),
                new SleepAction(clawOpenWait), //.3
                new ParallelAction(
                        seg4,
                        Mechs.Return(),
                        lift.baseHeight()
                ),

                //grab 3rd sample
                Mechs.slideOut(),
                new SleepAction(slideWait),
                Mechs.intakeOn(),
                new SleepAction(intakeWait), //1
                Mechs.intakeOff(),
                new SleepAction(inWait),
                Mechs.slideIn(),
                new SleepAction(linkInWait), //.5 //.25
                Mechs.intakeALil(),
                new SleepAction(intakeInsureWait), //.1),
                Mechs.intakeOff(),
                Mechs.armDownALil(),
                new SleepAction(armDownWait),
                Mechs.closeClaw(),
                new SleepAction(clawCloseWait),//.5

                //Score 3rd sample
                lift.scoreHeight(),
                Mechs.coverOpen(),
                new SleepAction(thirdLiftWait),
                Mechs.saScorePos(),
                seg5,
                new SleepAction(armStabilizeWait),
                Mechs.openClaw(),
                new SleepAction(clawOpenWait), //.3
                seg6,
                Mechs.Return(),
                lift.baseHeight(),


                Mechs.slideOut(),
                new SleepAction(slideWait),
                Mechs.intakeOn(),
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

                lift.scoreHeight(),
                Mechs.coverOpen(),
                new SleepAction(lastLiftWait),
                Mechs.saScorePos(),
                seg7,
                new SleepAction(armStabilizeWait), //.3
                Mechs.openClaw(),
                new SleepAction(clawOpenWait), //.3
                Mechs.park(),
                lift.baseHeight(),
                seg8





        ));
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
    telemetry.update();

    }
}