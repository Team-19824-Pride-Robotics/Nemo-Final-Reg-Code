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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name = "5 Specimen Auto")
public class auto_5Spec extends LinearOpMode {

/////////////////////////
/////Mech Positions//////
/////////////////////////
    private LinkageSubsystem linkage;
    private liftSubsystem lift;
    private intakeSubsystem intake;
    private bucketSubsystem bucket;
    private clawSubsystem claw;
    private armSubsystem arm;
    private wristSubsystem wrist;
    public static int spHeight1 = 900;
    public static int spHeight2 = 1550; //1650
    public static double AHPos = 0.06; //left  linkage in
    public static double BHPos = 0.035; //right linkage in
    public static double AHPos1 = 0.13; //left linkage mostly in
    public static double BHPos1 = 0.075; //right linkage mostly in
    public static double AHPos2 = 0.36; //left linkage out
    public static double BHPos2 = 0.2; //right linkage out
    public static double Bpos = 0.28; //bucket up
    public static double Bpos2 = 0.31; //bucket Mid (not final)
    public static double Bpos3 = 0.35; //bucket down
    public static double Epos1 = .8; //Specimen grab
    public static double Epos2 = .58; //Specimen hang
    public static double Cpos = 0.55; //open
    public static double Cpos2 = 0.9; //open a little (to not descore other specimens)(not final)
    public static double Cpos3 = 0.81; //closed
    public static double rwGrab = .37; //grab specimen
    public static double lwGrab = .3; //grab specimen
    public static double lwHang = 0.54; //hang specimen
    public static double rwHang = 0.54; //hang specimen
    ///////////////////////////
    /////Robot Positions//////
    /////////////////////////
    public static double x0 = 38;
    public static double y0 = -5;
    public static double x1 = 22; //24
    public static double y1 = -37; //-34
    public static double x2 = 16;
    public static double y2 = -24;
    public static double x3 = 23;
    public static double y3 = -46;
    public static double x4 = 15;
    public static double y4 = -25;
    public static double x5 = 26;
    public static double y5 = -58;
    public static double x6 = 16;
    public static double y6 = -20;
    public static double x7 = -1;
    public static double y7 = -28;
    public static double x8 = 12;
    public static double y8 = -15;
    public static double x9 = 44.5;
    public static double y9 = -1;
    public static double x10 = 0;
    public static double y10 = -35;
    public static double x11 = 44;
    public static double y11 = 3;
    public static double x12 = 1;
    public static double y12 = -37;
    public static double x13 = 43;
    public static double y13 = 7;
    public static double x14 = 1;
    public static double y14 = -41;
    public static double x15 = 43;
    public static double y15 = 12;
    public static double y16 = -30;
    public static double x16 = 10;
    public static  double parkTurn = 60;
    public static double sample1gr = 160;
    public static double sample1ex = -130;
    public static double sample2gr = 160;
    public static double sample2ex = -130;
    public static double sample3gr = 160;
    public static double sample3ex = -130;
    public static double spec4PickAng = 358;
    public static double spec5PickAng = 362;
/////////////////////
/////Sleep vars//////
/////////////////////
    public static double hangSleep=0.4; //.5
    public static double hangSleep2=0.4; //.5
    public static double grabSleep=0.2;
    public static double abramSleep=0;
    public static double clawSleep=0.1;
    public static double downSleep=0;
    public static double inSleep=0.1;
    public static double inSleep2=0.4;
    public static double outSleep=0.4; //.5
    public static double outSleep2=0.5;//.8
    /////////////////////
    /////Speed vars//////
    /////////////////////
    public static double hangSpeed = 40;
    public static double grabSpeed = 30;

    public class Intake {




        public class spHangPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //bring claw to origin
                wrist.wristScoreSpeicmen();
                arm.armSpecimen();

                return false;
            }
        }
        public Action spHangPos() {
            return new spHangPos();
        }

        public class openClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.clawOpen();
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
                return false;
            }
        }
        public Action closeClaw() {
            return new closeClaw();
        }


        public class spGrabPos implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                arm.armPickupSpeicmen();
                wrist.wristPickupSpeicmen();

                return false;
            }
        }
        public Action spGrabPos() {
            return new spGrabPos();
        }
        public class intakeOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

               linkage.linkOut();
                return false;
            }
        }
        public Action intakeOut() {
            return new intakeOut();
        }
        public class intakeIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                linkage.linkIn();
                return false;
            }
        }
        public Action intakeIn() {
            return new intakeIn();
        }

        public class intakeInABit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                linkage.linkMid();
                return false;
            }
        }
        public Action intakeInABit() {
            return new intakeInABit();
        }

        public class bucketUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                bucket.bucketUp();
                return false;
            }
        }
        public Action bucketUp() {
            return new bucketUp();
        }
        public class bucketDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                bucket.bucketDown();
                return false;
            }
        }
        public Action bucketDown() {
            return new bucketDown();
        }

        public class intakeOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                intake.intakeSetIdle();




                return false;
            }
        }
        public Action intakeOff() {
            return new intakeOff();
        }
        public class intakeOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                intakeSubsystem.intakeIn=0.5;
                intakeSubsystem.intakeOut=0;
                intake.intakeSetPower();



                return false;
            }
        }

        public Action intakeOn() {
            return new intakeOn();
        }
        public class intakeOn2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                intakeSubsystem.intakeIn=0.8;
                intakeSubsystem.intakeOut=0;
                intake.intakeSetPower();




                return false;
            }
        }

        public Action intakeOn2() {
            return new intakeOn2();
        }
        public class intakeExpel implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                intakeSubsystem.intakeIn=0;
                intakeSubsystem.intakeOut=-1;
                intake.intakeSetPower();




                return false;
            }
        }
        public Action intakeExpel() {
            return new intakeExpel();
        }

        public class coverOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                bucket.coverOpen();




                return false;
            }
        }
        public Action coverOpen() {
            return new intakeExpel();
        }
        public class coverClose implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                bucket.coverOpen();




                return false;
            }
        }
        public Action coverClose() {
            return new intakeExpel();
        }


    }


    public class lift {

        private DcMotorEx lift1;
        private DcMotorEx lift2;
        
            


        public class upABit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                lift.score();
                return false;
            }
        }
        public Action upABit() {
            return new upABit();
        }
        public class baseHeight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lift.pickup();
                return false;
            }
        }
        public Action baseHeight() {
            return new baseHeight();
        }

        public class scoreHeight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lift.barHigh();
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
        Intake intake = new Intake();
        lift lift = new lift();
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

                .strafeToLinearHeading(new Vector2d(x1, y1), Math.toRadians(sample1gr));

        Action seg2 = segment2.build();

        //segment 3 - brings 1st sample to obs zone
        segment3 = segment2.endTrajectory().fresh()

                .turn(Math.toRadians(sample1ex));

        Action seg3 = segment3.build();

        //segment 4 - goes to 2nd sample
        segment4 = segment3.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x3, y3), Math.toRadians(sample2gr));

        Action seg4 = segment4.build();

        //segment 5 - brings 2nd sample to obs zone
        segment5 = segment4.endTrajectory().fresh()

                .turn(Math.toRadians(sample2ex));

        Action seg5 = segment5.build();

        //segment 6 - goes to 3rd sample
        segment6 = segment5.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x5, y5), Math.toRadians(sample3gr));

        Action seg6 = segment6.build();

        //segment 7 - Brings 3rd sample to obs zone
        segment7 = segment6.endTrajectory().fresh()

                .turn(Math.toRadians(sample3ex));

        Action seg7 = segment7.build();

        //segment 8 - go to grab 2nd specimen
        segment8 = segment7.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x7, y7), Math.toRadians(356));

        Action seg8 = segment8.build();

        //segment 9 - Nothing
//        segment9 = segment8.endTrajectory().fresh()
//
//                .strafeToLinearHeading(new Vector2d(x4, y6), Math.toRadians(0));
//
//        Action seg9 = segment9.build();

        //segment 10 - hang 2nd specimen
        segment10 = segment8.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x9, y9), Math.toRadians(356));

        Action seg10 = segment10.build();

        //segment 11 - grab 3rd specimen
        segment11 = segment10.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x10, y10), Math.toRadians(356));

        Action seg11 = segment11.build();

        //segment 12 - hang 3rd specimen
        segment12 = segment11.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x11, y11), Math.toRadians(356));

        Action seg12 = segment12.build();

        //segment 13 - grab 4th specimen
        segment13 = segment12.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x12, y12), Math.toRadians(spec4PickAng));

        Action seg13 = segment13.build();

        //segment 14 - hang 4th specimen
        segment14 = segment13.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x13, y13), Math.toRadians(356));

        Action seg14 = segment14.build();

        //segment 15 - grab 5th specimen
        segment15 = segment14.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x14, y14), Math.toRadians(spec5PickAng));

        Action seg15 = segment15.build();

        //segment 16 - hang 5th specimen (yippee!!!)
        segment16 = segment15.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x15, y15), Math.toRadians(356));

        Action seg16 = segment16.build();
        waitForStart();

        //segment 17 - park
        segment17 = segment16.endTrajectory().fresh()

                .strafeToLinearHeading(new Vector2d(x16, y16), Math.toRadians(parkTurn));

        Action seg17 = segment17.build();
        Actions.runBlocking(
new SequentialAction(
        intake.closeClaw()
)
        );

        waitForStart();

        if (isStopRequested()) return;



        Actions.runBlocking(new SequentialAction(
                //score first specimen
                intake.closeClaw(),
                intake.bucketUp(),
                intake.spHangPos(),
                intake.intakeIn(),
                intake.coverClose(),
                lift.scoreHeight(),
                seg1,
                lift.upABit(),
                new SleepAction(hangSleep2),
                intake.openClaw(),
                new SleepAction(clawSleep),
                lift.baseHeight(),


                //push samples in observation zone
                //sample 1
                seg2,
                new SleepAction(downSleep),
                intake.intakeOut(),
                intake.bucketDown(),
                intake.intakeOn(),
                new SleepAction(inSleep),
                seg3,
                intake.bucketDown(),
                intake.coverOpen(),
                intake.intakeExpel(),
                new SleepAction(outSleep),
                intake.bucketUp(),
                intake.intakeInABit(),
                intake.intakeOff(),
                 //new

                //sample 2
                seg4,
                intake.intakeOut(),
                intake.coverClose(),
                new SleepAction(downSleep),
                intake.bucketDown(),
                intake.intakeOn(),
                new SleepAction(inSleep),
                seg5,
                intake.coverOpen(),
                intake.bucketDown(),
                intake.intakeExpel(),
                new SleepAction(outSleep),
                intake.bucketUp(),
                intake.intakeOff(),
                intake.intakeInABit(),

                //sample 3
                seg6,
                intake.intakeOut(),
                new SleepAction(downSleep),
                intake.coverClose(),
                intake.bucketDown(),
                intake.intakeOn2(),
                new SleepAction(inSleep2),
                new ParallelAction(
                        intake.intakeInABit(),
                        seg7
                ),



                intake.intakeOut(),
                intake.coverOpen(),
                intake.bucketDown(),
                intake.intakeExpel(),
                new SleepAction(outSleep2),
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
                new SleepAction(clawSleep),
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

                new SleepAction(abramSleep),
                intake.closeClaw(),
                new SleepAction(grabSleep),

                intake.spHangPos(),
                lift.scoreHeight(),
                seg14,
                lift.upABit(),
                new SleepAction(hangSleep),
                intake.openClaw(),
                new SleepAction(clawSleep),
                lift.baseHeight(),
                intake.spGrabPos(),


                //score 5th specimen
                seg15,
                new SleepAction(abramSleep),
                intake.closeClaw(),
                new SleepAction(grabSleep),
                intake.spHangPos(),
                lift.scoreHeight(),
                seg16,
                lift.upABit(),
                new SleepAction(hangSleep),
                intake.openClaw(),
                new SleepAction(clawSleep),
                lift.baseHeight(),
                intake.spGrabPos(),


                //park
                intake.intakeOut(),
                seg17



        ));


    }

}