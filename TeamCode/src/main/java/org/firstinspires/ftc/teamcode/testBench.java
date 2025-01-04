/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Pride robot 2 player")
@Config
public class testBench extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //lift pid
    public static double p = 0.005, i = 0, d = 0;
    public static double f = 0;
    public static double target = 0;
    public static double target2 = 110;
    public static double target1 = 110;

    private PIDController controller;


    //Slide heights
    public static int saHeight1 = 1300;
    public static int spHeight1 = 0;
    public static int saHeight2 = 3100;
    public static int spHeight2 = 1000;


    public static int baseHeight = 0;
    public static double HPos;

    public static double HPos2;
    public static double Bpos=.31; //Up

    public static double Bpos2=0.37; //Down

    public static double Epos1 = .28; //Originpickup
    public static double Epos2 = .3; //Origin
    public static double Epos3 = 0.55; //Specimen
    public static double Epos4 = 0.65; //Sample

    public static double Cpos = 0.79; //open

    public static double Cpos2 = 0.935; //closed

    public static double Wpos1 = 0.35;

    public static double Wpos2 = 0.66;
    public static double Wpos3 = 0.5;

    DcMotorEx lift1;
    DcMotorEx lift2;

    ServoImplEx backWrist;

    ServoImplEx frontWrist;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        controller = new PIDController(p, i, d);


        //Drive base config
        DcMotor frontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeft = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRight = hardwareMap.dcMotor.get("rightBack");
        //Claw config
        Servo claw = hardwareMap.servo.get("claw");

        //Wrist Config
        backWrist = (ServoImplEx) hardwareMap.get(Servo.class, "lw");
        backWrist.setPwmRange(new PwmControl.PwmRange(505, 2495));
        frontWrist = (ServoImplEx) hardwareMap.get(Servo.class, "rw");
        frontWrist.setPwmRange(new PwmControl.PwmRange(505, 2495));


        //Elbow config
        Servo elbow = hardwareMap.servo.get("arm");

        //Horizontal slide config
        Servo horizontalSlides1 = hardwareMap.servo.get("ll");
        Servo horizontalSlides2 = hardwareMap.servo.get("rl");
        //intake position
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo intakeBucket = hardwareMap.servo.get("bucket");

        //lift
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift1.setDirection(DcMotorEx.Direction.REVERSE);



//Various drive code variables
        int spStage = -1;
        boolean atOrigin = true;
        int saStage = -1;
        int outStage = 0;
        boolean intakeIsOut = false;
        boolean slideAdjusted = false;
        boolean stupidButton = false;
        boolean atBase = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Starting values
        //intakeBucket.setPosition(Bpos);
//        frontWrist.setPosition(Wpos2);
//        backWrist.setPosition(Wpos2);
//        elbow.setPosition(Epos1);
//        horizontalSlides1.setPosition(HPos);
//        horizontalSlides2.setPosition(HPos);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Pid stuff lol
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
            if(gamepad1.a){
                frontWrist.setPosition(Wpos1);
                backWrist.setPosition(Wpos1);
                elbow.setPosition(Epos1);
            }
            if(gamepad1.b){
                frontWrist.setPosition(Wpos2);
                backWrist.setPosition(Wpos2);
                elbow.setPosition(Epos3);
            }

            if(gamepad1.y){
                frontWrist.setPosition(Wpos3);
                backWrist.setPosition(Wpos3);
                elbow.setPosition(Epos4);
            }
            if(gamepad1.x){
                elbow.setPosition(Epos4);
            }

//            Claw controls
//
//            close claw




            //Drive code
            double y = gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx =  gamepad2.right_stick_x;



            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;



//pid


            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);



//             Send calculated power to wheels
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            telemetry.addData("outStage", outStage);
            telemetry.addData("stupid button", stupidButton);
            telemetry.addData("1", "test");
            telemetry.addData("pos1", lift1.getCurrentPosition());
            telemetry.addData("power1", lift1.getPower());
            telemetry.addData("pos2", lift2.getCurrentPosition());
            telemetry.addData("power2", lift2.getPower());
            telemetry.update();

        }
    }
}
