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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.sun.tools.javac.processing.PrintingProcessor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;


@Autonomous(name="AUTO_KK_LEFT_TEST", group="HARRYBOTTERS")
@Disabled
public class Auto_KK_Left_TEST extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardwareMecanum robot = new RobotHardwareMecanum();     // Use a Mecanum's hardware

    //declare motors
    private DcMotor driveFLM;
    private DcMotor driveFRM;
    private DcMotor driveBLM;
    private DcMotor driveBRM;
    private DcMotor LinearSlider;

    //declare servos
    private Servo clawLeftHand;
    private Servo clawRightHand;

    //declare colorrange sensor
    private ColorSensor coneSensor;
    private int robotWhere =0;

    private boolean isTravelComplete = true;
    private boolean isSlideMovementComplete = true;

    public static final int HIGH_POLE_TICK_COUNT = 2200;
    public static final int MEDIUM_POLE_TICK_COUNT = 1000;
    public static final int SMALL_POLE_TICK_COUNT = 2000;
    public static final int STACK_5_TICK_COUNT = 380;
    public static final int STACK_4_TICK_COUNT = 2300;
    public static final int STACK_3_TICK_COUNT = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.autoInit(hardwareMap);
/*
        //motors to configuration
        driveFLM = hardwareMap.dcMotor.get("MDriveFL");
        driveFRM = hardwareMap.dcMotor.get("MDriveFR");
        driveBLM = hardwareMap.dcMotor.get("MDriveBL");
        driveBRM = hardwareMap.dcMotor.get("MDriveBR");
        LinearSlider = hardwareMap.dcMotor.get("MSlider");

        //servos to configuration
        clawLeftHand = hardwareMap.servo.get("SClawLeft");
        clawRightHand = hardwareMap.servo.get("SClawRight");

        //color-range sensor
        coneSensor = hardwareMap.colorSensor.get("ColorSensor");

        //set direction of the motors to drive in mecanum fashion
        driveFLM.setDirection(FORWARD);
        driveFRM.setDirection(REVERSE);
        driveBLM.setDirection(FORWARD);
        driveBRM.setDirection(REVERSE);
        LinearSlider.setDirection(FORWARD);

        //set motors to run with encoders
        driveFLM.setMode(RUN_USING_ENCODER);
        driveFRM.setMode(RUN_USING_ENCODER);
        driveBLM.setMode(RUN_USING_ENCODER);
        driveBRM.setMode(RUN_USING_ENCODER);
        LinearSlider.setMode(RUN_USING_ENCODER);

        // reset encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);
        LinearSlider.setMode(STOP_AND_RESET_ENCODER);


        // set motors to brake, so they dont move during initialization

        driveFLM.setZeroPowerBehavior(BRAKE);
        driveFRM.setZeroPowerBehavior(BRAKE);
        driveBLM.setZeroPowerBehavior(BRAKE);
        driveBRM.setZeroPowerBehavior(BRAKE);
        LinearSlider.setZeroPowerBehavior(BRAKE);

        telemetry.addData("ready?: ", "stupid bitch");
        telemetry.update();

        //color sense telemetry
            telemetry.addData("red", coneSensor.red());
            telemetry.addData("green", coneSensor.green());
            telemetry.addData("blue", coneSensor.blue());
            telemetry.update();

        //initializing components
        clawLeftHand.setPosition(.25);
        clawRightHand.setPosition(.75);

*/
        waitForStart();

        forwardAutoLeft();

        //robot moving
        // useful information: wait = sleep(time in milliseconds); 1sec = 1000millis

        //test


        //test



    }

    private void forwardAutoLeft() throws InterruptedException{

        closeClaw();
        telemetry.addLine("Closed claw");
        telemetry.update();

        driveForwardE(.5,2400, false);
        slideUp(1, 2150);

        //sleep(20000);
        colorsense();

        driveBackwardE(.5, 150, true);
        spinRightE(.25, 500, true);
        driveForwardE(.25,275, true);

        sleep(5000);

//        driveBackwardE(.5,2600, false);
//        telemetry.addLine("going backward");
//        telemetry.update();
//        slideUp(1, 3000, false);
//        telemetry.addLine("Moving up the slide");
//        telemetry.update();
    }

    private void backwardAutoLeft() throws InterruptedException{

        closeClaw();
        telemetry.addLine("Closed claw");
        telemetry.update();
        driveBackwardE(.5,2600, false);
        telemetry.addLine("going backward");
        telemetry.update();
        slideUp(1, 3000, false);
        telemetry.addLine("Moving up the slide");
        telemetry.update();

        //sleep(2000000000);
        while (isRobotMoving() ){
            telemetry.addLine("Waiting on movement" );
            telemetry.update();

            if (!isRobotMoving()){
                stopRobot();
            }
            if(! isSlideMoving()){
                stopSlide();
            }
        }
        telemetry.addLine("Stopping");
        telemetry.update();

        stopRobot();
        stopSlide();

        colorsense();
        //sleep(100);
        driveForwardE(.5, 600);
        sleep(100);
        spinLeftE(.5, 1325);
        driveForwardE(.5,400);
        sleep(200);
        slideDown(.5, 200);
        openClaw();
        slideUp(.5,200);
        driveBackwardE(.5, 400);

        //LinearSlider.setPower(0);

        spinLeftE(.5, 1200);
        driveForwardE(.5, 1080, false); //added 50
        slideDown(.5, 2450, true);

        while (isRobotMoving() || isSlideMoving()){
            if (!isRobotMoving()){
                stopRobot();
            }
            if(! isSlideMoving()){
                //stopSlide();
            }
        }
        stopRobot();
        stopSlide();



        sleep(20000);
        closeClaw();
        sleep(200);
        slideUp(1, 2600);
        LinearSlider.setPower(0);
        sleep(300);

        sleep(20000);




        //robot where code
        if (robotWhere == 0) {
            //doing cone
            doConeThings();

            //parking
            moveRightE(.5, 1050);

        }
        if (robotWhere == 1) {
            //doing cone
            doConeThings();

            //parking
            sleep(500);
        }
        if (robotWhere == 2) {
            //doing cone
            doConeThings();

            //parking
            moveLeftE(.5, 1050);
        }
    }


    //mini methods for ease
    //cone things
    private void doConeThings() throws InterruptedException {
        driveForwardE(.5, 600);
        sleep(100);
        spinLeftE(.5, 1325);
        //slideUp(1, 3000);
        driveForwardE(.5,400);
        sleep(200);
        slideDown(.5, 200);
        openClaw();
        slideUp(.5,200);
        slideDown(1, 2450);
        LinearSlider.setPower(0);

        goingStack();

        goingScoring();

        driveBackwardE(.5, 400  );
        spinRightE(.5, 1325);
        closeClaw();
        slideDown(1, 570);
        openClaw();
        driveBackwardE(.5, 100);
    }

    //sub mini methods for ease
    //go to cone
    private void goingScoring() throws InterruptedException {
        driveBackwardE(.5, 1075); //added 50
        spinRightE(.5, 1200);
        slideUp(1, 50);
        driveForwardE(.5, 400);
        sleep(100);
        slideDown(.5, 50);
        sleep(100);
        openClaw();
        slideUp(1, 50);
        driveBackwardE(.5, 50);
        slideDown(1, 2500);
        driveForwardE(.5, 50);
        LinearSlider.setPower(0);
    }

    //to stack
    private void goingStack() throws InterruptedException {
        sleep(300);
        driveBackwardE(.5, 400);
        spinLeftE(.5, 1200);
        driveForwardE(.5, 1080); //added 50
        sleep(200);
        closeClaw();
        sleep(200);
        slideUp(1, 2600);
        LinearSlider.setPower(0);
        sleep(300);
    }

    //ALL YOUR METHODS:
    //drive forward method
    private boolean isRobotMoving() throws InterruptedException {
        if(driveFLM.isBusy() || driveFRM.isBusy() || driveBLM.isBusy() || driveBRM.isBusy()) {
            telemetry.addLine("Robot still moving");
            telemetry.update();
            return true;
        } else {
            telemetry.addLine("Robot stopped");
            telemetry.update();
            return false;
        }
    }

    private boolean isSlideMoving() throws InterruptedException {
        if(LinearSlider.isBusy()) {
            telemetry.addLine("slide still moving");
            telemetry.update();
            return true;
        } else {
            telemetry.addLine("slide stopped");
            telemetry.update();
            return false;
        }
    }

    private void stopSlide() throws InterruptedException {
        telemetry.addLine("received call to stop slide");
        telemetry.update();
        LinearSlider.setPower(0);
        LinearSlider.setZeroPowerBehavior(BRAKE);
        LinearSlider.setMode(RUN_USING_ENCODER);

    }

    private void stopRobot() throws InterruptedException {
        telemetry.addLine("received call to stop robot");
        telemetry.update();
        //stop moving
        driveFLM.setPower(0);
        driveFRM.setPower(0);
        driveBLM.setPower(0);
        driveBRM.setPower(0);

        driveFLM.setZeroPowerBehavior(BRAKE);
        driveFRM.setZeroPowerBehavior(BRAKE);
        driveBLM.setZeroPowerBehavior(BRAKE);
        driveBRM.setZeroPowerBehavior(BRAKE);

        driveFLM.setMode(RUN_USING_ENCODER);
        driveFRM.setMode(RUN_USING_ENCODER);
        driveBLM.setMode(RUN_USING_ENCODER);
        driveBRM.setMode(RUN_USING_ENCODER);

    }

    private void driveForwardE(double power, int ticks, boolean waitOnTravel) throws InterruptedException {
        // reset encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        //set target position, basically so you dont have to do minus ticks when writing code
        driveFLM.setTargetPosition(ticks);
        driveFRM.setTargetPosition(ticks);
        driveBLM.setTargetPosition(ticks);
        driveBRM.setTargetPosition(ticks);

        //set to run to position mode
        driveFLM.setMode(RUN_TO_POSITION);
        driveFRM.setMode(RUN_TO_POSITION);
        driveBLM.setMode(RUN_TO_POSITION);
        driveBRM.setMode(RUN_TO_POSITION);

        //drive backward
        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        //wait until target position
        if(waitOnTravel) {
            while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) {
                    // wait
            }

            //stop moving
            driveFLM.setPower(0);
            driveFRM.setPower(0);
            driveBLM.setPower(0);
            driveBRM.setPower(0);
            Thread.sleep(50);

            driveFLM.setZeroPowerBehavior(BRAKE);
            driveFRM.setZeroPowerBehavior(BRAKE);
            driveBLM.setZeroPowerBehavior(BRAKE);
            driveBRM.setZeroPowerBehavior(BRAKE);

            driveFLM.setMode(RUN_USING_ENCODER);
            driveFRM.setMode(RUN_USING_ENCODER);
            driveBLM.setMode(RUN_USING_ENCODER);
            driveBRM.setMode(RUN_USING_ENCODER);
        }

    }


    private void driveForwardE(double power, int ticks) throws InterruptedException {

       driveForwardE(power, ticks, true);
    }

    private void driveBackwardE(double power, int ticks, boolean waitOnTravel) throws InterruptedException {

        // reset encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        //set target position, basically so you dont have to do minus ticks when writing code
        driveFLM.setTargetPosition(-ticks);
        driveFRM.setTargetPosition(-ticks);
        driveBLM.setTargetPosition(-ticks);
        driveBRM.setTargetPosition(-ticks);

        //set to run to position mode
        driveFLM.setMode(RUN_TO_POSITION);
        driveFRM.setMode(RUN_TO_POSITION);
        driveBLM.setMode(RUN_TO_POSITION);
        driveBRM.setMode(RUN_TO_POSITION);

        //drive backward
        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        if(waitOnTravel) {
            while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) {

            }

            //stop moving
            driveFLM.setPower(0);
            driveFRM.setPower(0);
            driveBLM.setPower(0);
            driveBRM.setPower(0);
            Thread.sleep(50);

            driveFLM.setZeroPowerBehavior(BRAKE);
            driveFRM.setZeroPowerBehavior(BRAKE);
            driveBLM.setZeroPowerBehavior(BRAKE);
            driveBRM.setZeroPowerBehavior(BRAKE);

            driveFLM.setMode(RUN_USING_ENCODER);
            driveFRM.setMode(RUN_USING_ENCODER);
            driveBLM.setMode(RUN_USING_ENCODER);
            driveBRM.setMode(RUN_USING_ENCODER);
        }

    }

    //drive backward method
    private void driveBackwardE(double power, int ticks) throws InterruptedException {

        driveBackwardE(power, ticks, true);

    }

    //spin left method
    private void spinLeftE(double power, int ticks, boolean waitOnTravel) throws InterruptedException {

        // reset encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        //set target position, basically so you dont have to do minus ticks when writing code
        driveFLM.setTargetPosition(-ticks);
        driveFRM.setTargetPosition(ticks);
        driveBLM.setTargetPosition(-ticks);
        driveBRM.setTargetPosition(ticks);

        //set to run to position mode
        driveFLM.setMode(RUN_TO_POSITION);
        driveFRM.setMode(RUN_TO_POSITION);
        driveBLM.setMode(RUN_TO_POSITION);
        driveBRM.setMode(RUN_TO_POSITION);

        //drive backward
        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        if(waitOnTravel) {
            //wait until target position
            while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) {

            }

            //stop moving
            driveFLM.setPower(0);
            driveFRM.setPower(0);
            driveBLM.setPower(0);
            driveBRM.setPower(0);
            Thread.sleep(50);

            driveFLM.setZeroPowerBehavior(BRAKE);
            driveFRM.setZeroPowerBehavior(BRAKE);
            driveBLM.setZeroPowerBehavior(BRAKE);
            driveBRM.setZeroPowerBehavior(BRAKE);

            driveFLM.setMode(RUN_USING_ENCODER);
            driveFRM.setMode(RUN_USING_ENCODER);
            driveBLM.setMode(RUN_USING_ENCODER);
            driveBRM.setMode(RUN_USING_ENCODER);
        }

    }

    private void spinLeftE(double power, int ticks) throws InterruptedException {
        spinLeftE(power, ticks, true);
    }

        //spin right method
    private void spinRightE(double power, int ticks, boolean waitOnTravel) throws InterruptedException {

        // reset encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        //set target position, basically so you dont have to do minus ticks when writing code
        driveFLM.setTargetPosition(ticks);
        driveFRM.setTargetPosition(-ticks);
        driveBLM.setTargetPosition(ticks);
        driveBRM.setTargetPosition(-ticks);

        //set to run to position mode
        driveFLM.setMode(RUN_TO_POSITION);
        driveFRM.setMode(RUN_TO_POSITION);
        driveBLM.setMode(RUN_TO_POSITION);
        driveBRM.setMode(RUN_TO_POSITION);

        //drive backward
        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        if(waitOnTravel) {
            //wait until target position
            while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) {

            }

            //stop moving
            driveFLM.setPower(0);
            driveFRM.setPower(0);
            driveBLM.setPower(0);
            driveBRM.setPower(0);
            Thread.sleep(50);

            driveFLM.setZeroPowerBehavior(BRAKE);
            driveFRM.setZeroPowerBehavior(BRAKE);
            driveBLM.setZeroPowerBehavior(BRAKE);
            driveBRM.setZeroPowerBehavior(BRAKE);

            driveFLM.setMode(RUN_USING_ENCODER);
            driveFRM.setMode(RUN_USING_ENCODER);
            driveBLM.setMode(RUN_USING_ENCODER);
            driveBRM.setMode(RUN_USING_ENCODER);
        }

    }

    private void spinRightE(double power, int ticks) throws InterruptedException {
        spinRightE(power, ticks, true);
    }
    //move left method
    private void moveLeftE(double power, int ticks) throws InterruptedException {

        // reset encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        //set target position, basically so you dont have to do minus ticks when writing code
        driveFLM.setTargetPosition(-ticks);
        driveFRM.setTargetPosition(ticks);
        driveBLM.setTargetPosition(ticks);
        driveBRM.setTargetPosition(-ticks);

        //set to run to position mode
        driveFLM.setMode(RUN_TO_POSITION);
        driveFRM.setMode(RUN_TO_POSITION);
        driveBLM.setMode(RUN_TO_POSITION);
        driveBRM.setMode(RUN_TO_POSITION);

        //drive backward
        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        //wait until target position
        while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) {

        }

        //stop moving
        driveFLM.setPower(0);
        driveFRM.setPower(0);
        driveBLM.setPower(0);
        driveBRM.setPower(0);
        Thread.sleep(50);

        driveFLM.setZeroPowerBehavior(BRAKE);
        driveFRM.setZeroPowerBehavior(BRAKE);
        driveBLM.setZeroPowerBehavior(BRAKE);
        driveBRM.setZeroPowerBehavior(BRAKE);

        driveFLM.setMode(RUN_USING_ENCODER);
        driveFRM.setMode(RUN_USING_ENCODER);
        driveBLM.setMode(RUN_USING_ENCODER);
        driveBRM.setMode(RUN_USING_ENCODER);
    }

    //move right method
    private void moveRightE(double power, int ticks) throws InterruptedException {

        // reset encoders
        driveFLM.setMode(STOP_AND_RESET_ENCODER);
        driveFRM.setMode(STOP_AND_RESET_ENCODER);
        driveBLM.setMode(STOP_AND_RESET_ENCODER);
        driveBRM.setMode(STOP_AND_RESET_ENCODER);

        //set target position, basically so you dont have to do minus ticks when writing code
        driveFLM.setTargetPosition(ticks);
        driveFRM.setTargetPosition(-ticks);
        driveBLM.setTargetPosition(-ticks);
        driveBRM.setTargetPosition(ticks);

        //set to run to position mode
        driveFLM.setMode(RUN_TO_POSITION);
        driveFRM.setMode(RUN_TO_POSITION);
        driveBLM.setMode(RUN_TO_POSITION);
        driveBRM.setMode(RUN_TO_POSITION);

        //drive backward
        driveFLM.setPower(power);
        driveFRM.setPower(power);
        driveBLM.setPower(power);
        driveBRM.setPower(power);

        //wait until target position
        while (driveFLM.isBusy() && driveFRM.isBusy() && driveBLM.isBusy() && driveBRM.isBusy()) {

        }

        //stop moving
        driveFLM.setPower(0);
        driveFRM.setPower(0);
        driveBLM.setPower(0);
        driveBRM.setPower(0);
        Thread.sleep(50);

        driveFLM.setZeroPowerBehavior(BRAKE);
        driveFRM.setZeroPowerBehavior(BRAKE);
        driveBLM.setZeroPowerBehavior(BRAKE);
        driveBRM.setZeroPowerBehavior(BRAKE);

        driveFLM.setMode(RUN_USING_ENCODER);
        driveFRM.setMode(RUN_USING_ENCODER);
        driveBLM.setMode(RUN_USING_ENCODER);
        driveBRM.setMode(RUN_USING_ENCODER);
    }

    //linear slide up
    private void slideUp(double power, int ticks) throws InterruptedException {

        slideUp(power, ticks, true);
    }

    private void slideUp(double power, int ticks, boolean waitOnMovement) throws InterruptedException {

        //LinearSlider.setMode(STOP_AND_RESET_ENCODER);
        LinearSlider.setTargetPosition(ticks);
        LinearSlider.setMode(RUN_TO_POSITION);
        LinearSlider.setPower(power);

        if(waitOnMovement) {
            while (LinearSlider.isBusy()) {

            }
            LinearSlider.setPower(0.1);
            //LinearSlider.setZeroPowerBehavior(BRAKE);
            //LinearSlider.setMode(RUN_USING_ENCODER);

        }
    }

    //linear slide down
    private void slideDown(double power, int ticks) throws InterruptedException {

        slideDown(power, ticks, true);

    }

    private void slideDown(double power, int ticks, boolean waitOnMovement) throws InterruptedException {

        //LinearSlider.setMode(STOP_AND_RESET_ENCODER);
        LinearSlider.setTargetPosition(-ticks);
        LinearSlider.setMode(RUN_TO_POSITION);
        LinearSlider.setPower(power);
        if(waitOnMovement) {
            while (LinearSlider.isBusy()) {

            }
            LinearSlider.setPower(0);
            LinearSlider.setZeroPowerBehavior(BRAKE);
            LinearSlider.setMode(RUN_USING_ENCODER);

        }
    }

    //color sensor sensing code
    private void colorsense() {

        telemetry.addData("Color - red(): ", coneSensor.red() );
        //sleep(100);
        telemetry.addData("Color - green(): ", coneSensor.green() );
        //sleep(100);
        telemetry.addData("Color - blue(): ", coneSensor.blue());
        telemetry.update();
        sleep(2000);

        //area where sensor output tells what path to take
        if (coneSensor.red() > 40) { //(coneSensor.blue() & coneSensor.green())
            robotWhere = 0;
        }else {
            if (coneSensor.green() > 40) {//(coneSensor.red() & coneSensor.green())
                robotWhere = 1;
            }else{
                robotWhere = 2;
            }
        }

    }

    //open claw and closing claw
    private void openClaw() {
        clawLeftHand.setPosition(.45);
        clawRightHand.setPosition(.55);
    }
    private void closeClaw() {
        clawLeftHand.setPosition(.25);
        clawRightHand.setPosition(.75);
    }
}
