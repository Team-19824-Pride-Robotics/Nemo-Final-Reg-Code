package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Mechs.claw;


@Config
@Autonomous(name = "Red_Specimen_Auto", group = "Autonomous")
public class Test_auto extends LinearOpMode {
    public static double x1 = 5;
    public static double y1 = 5;

    public void runOpMode() throws InterruptedException {

        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // make a Claw instance
        //claw claw = new claw(hardwareMap);


        TrajectoryActionBuilder segment1;


        segment1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(x1, 0));
        Action seg1 = segment1.build();


        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(


                seg1


        );

    }
}
