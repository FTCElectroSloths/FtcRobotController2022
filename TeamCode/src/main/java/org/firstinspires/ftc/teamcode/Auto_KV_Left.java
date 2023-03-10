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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.Closeable;


@Autonomous(name="AUTO_KV_LEFT", group="HARRYBOTTERS")
//@Disabled
public class Auto_KV_Left extends LinearOpMode {

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

    @Override
    public void runOpMode() throws InterruptedException {

        if (true) {

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

            //reset encoders
            driveFLM.setMode(STOP_AND_RESET_ENCODER);
            driveFRM.setMode(STOP_AND_RESET_ENCODER);
            driveBLM.setMode(STOP_AND_RESET_ENCODER);
            driveBRM.setMode(STOP_AND_RESET_ENCODER);
            LinearSlider.setMode(STOP_AND_RESET_ENCODER);

            //set motors to run with encoders
            driveFLM.setMode(RUN_USING_ENCODER);
            driveFRM.setMode(RUN_USING_ENCODER);
            driveBLM.setMode(RUN_USING_ENCODER);
            driveBRM.setMode(RUN_USING_ENCODER);
            LinearSlider.setMode(RUN_USING_ENCODER);

            // set motors to brake, so they dont move during initialization

            driveFLM.setZeroPowerBehavior(BRAKE);
            driveFRM.setZeroPowerBehavior(BRAKE);
            driveBLM.setZeroPowerBehavior(BRAKE);
            driveBRM.setZeroPowerBehavior(BRAKE);
            LinearSlider.setZeroPowerBehavior(BRAKE);

            telemetry.addData("ready?: ", "Electro Sloths");
            telemetry.update();

            //color sense telemetry




            //initializing components
            clawLeftHand.setPosition(.25);
            clawRightHand.setPosition(.75);

            //turn led off
            coneSensor.enableLed(false);
        }

        waitForStart();
        //robot moving
        // useful information: wait = sleep(time in milliseconds); 1sec = 1000millis
        //test
        //test

        slideUp(1, 2100, false);
        driveForwardE(.5, 950, true);
        coneSensor.enableLed(true);
        colorsense();
        telemetry.addData("red", coneSensor.red());
        telemetry.addData("green", coneSensor.green());
        telemetry.addData("blue", coneSensor.blue());
        telemetry.addData("sensed", robotWhere);
        telemetry.update();
        sleep(500);
        coneSensor.enableLed(false);
        driveForwardE(.5, 1400, true);
        sleep(100);

        //starter cone
        driveBackwardE(.5, 150, true);
        spinRightE(.5, 500, true);
        driveForwardE(.5,275, true);
        sleep(100);

        slideDown(.5, 1730, false);
        sleep(500);
        openClaw();

        //1st cone in stack
        driveBackwardE(.5, 500, true);
        sleep(100);

        spinLeftE(.5, 1365, true);
        driveForwardE(.5, 860, true);
        closeClaw();
        sleep(1000);
        slideUp(1, 1730, true);
        sleep(100);

        driveBackwardE(.5, 800, true);
        spinRightE(.5, 1365, true);
        driveForwardE(.5, 500, true);
        sleep(100);

        //make sure slide is set to next cone level
        slideDown(.5, 500, true);
        openClaw();
        sleep(100);
        slideUp(1, 500, true);

        //sleep(500);
        //openClaw();

        //2nd cone in stack
        driveBackwardE(.5, 500, true);
        slideDown(.5, 1900, false);

        sleep(100);

        spinLeftE(.5, 1350, true);
        driveForwardE(.5, 850, true);
        closeClaw();
        sleep(1000);
        slideUp(1, 1900, true);
        sleep(100);

        driveBackwardE(.5, 800, true);
        spinRightE(.5, 1350, true);
        driveForwardE(.5, 500, true);
        sleep(100);

        //make sure slide is set to next cone level
        slideDown(.5, 2100, false);
        sleep(200);
        openClaw();
        //sleep(100);

        //to park
        driveBackwardE(.5, 550,true);
        closeClaw();
        slideDown(.5, 200, false);

        spinLeftE(.5, 515, true);



        //robot where code
        if (robotWhere == 1) {
             //parking
            moveLeftE(.5, 1200, true);

        }
        if (robotWhere == 2) {
            //parking
            moveRightE(.5, 300, true);
        }
        if (robotWhere == 3) {
            //parking
            moveRightE(.75, 1400, true);
        }

    }

    //mini methods for ease
    //cone things
    private void doConeThings() throws InterruptedException {
//        driveForwardE(.5, 600, true);
//        sleep(100);
//        spinLeftE(.5, 1325, true);
//        slideUp(1, 3000, true);
//        driveForwardE(.5,400, true);
//        sleep(200);
//        slideDown(.5, 200, true);
//        openClaw();
//        slideUp(.5,200, true);
//        slideDown(1, 2450, true);
//        LinearSlider.setPower(0);
//
//        goingStack();
//
//        goingScoring();
//
//        driveBackwardE(.5, 400, true);
//        spinRightE(.5, 1325, true);
//        closeClaw();
//        slideDown(1, 570, true);
//        openClaw();
//        driveBackwardE(.5, 100, true);
    }

    //sub mini methods for ease
    //go to cone
    private void goingScoring() throws InterruptedException {
//        driveBackwardE(.5, 1075, true); //added 50
//        spinRightE(.5, 1200, true);
//        slideUp(1, 50, true);
//        driveForwardE(.5, 400, true);
//        sleep(100);
//        slideDown(.5, 50, true);
//        sleep(100);
//        openClaw();
//        slideUp(1, 50, true);
//        driveBackwardE(.5, 50, true);
//        slideDown(1, 2500, true);
//        driveForwardE(.5, 50, true);
//        LinearSlider.setPower(0);
    }

    //to stack
    private void goingStack() throws InterruptedException {
//        sleep(300);
//        driveBackwardE(.5, 400, true);
//        spinLeftE(.5, 1200, true);
//        driveForwardE(.5, 1080, true); //added 50
//        sleep(200);
//        closeClaw();
//        sleep(200);
//        slideUp(1, 2600, true);
//        LinearSlider.setPower(0);
//        sleep(300);
    }

    //stop moving
    private void stopRobot() throws InterruptedException {
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

    //stop slide
    private void stopSlide() throws InterruptedException {
        LinearSlider.setPower(0);
        LinearSlider.setZeroPowerBehavior(BRAKE);
        LinearSlider.setMode(RUN_USING_ENCODER);
    }

    //ALL YOUR METHODS:
    //allows for parallel things
    private boolean isRobotMoving(){
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

    private boolean isSlideMoving(){
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

    //drive forward method
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

    //drive backward method
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

    //move left method
    private void moveLeftE(double power, int ticks, boolean waitOnTravel) throws InterruptedException {

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

    //move right method
    private void moveRightE(double power, int ticks, boolean waitOnTravel) throws InterruptedException {

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

    //linear slide up
    private void slideUp(double power, int ticks, boolean waitOnMovement) throws InterruptedException {

        LinearSlider.setMode(STOP_AND_RESET_ENCODER);
        LinearSlider.setTargetPosition(ticks);
        LinearSlider.setMode(RUN_TO_POSITION);
        LinearSlider.setPower(power);

        if(waitOnMovement) {
            while (LinearSlider.isBusy()) {

            }
            LinearSlider.setPower(.05);
            LinearSlider.setZeroPowerBehavior(BRAKE);
            LinearSlider.setMode(RUN_USING_ENCODER);

        }

        //if(waitOnMovement)
    }

    //linear slide down
    private void slideDown(double power, int ticks, boolean waitOnMovement) throws InterruptedException {

        LinearSlider.setMode(STOP_AND_RESET_ENCODER);
        LinearSlider.setTargetPosition(-ticks);
        LinearSlider.setMode(RUN_TO_POSITION);
        LinearSlider.setPower(power);

        if(waitOnMovement) {
            while (LinearSlider.isBusy()) {

            }
            LinearSlider.setPower(.05);
            LinearSlider.setZeroPowerBehavior(BRAKE);
            LinearSlider.setMode(RUN_USING_ENCODER);

        }
    }

    //color sensor sensing code
    private void colorsense() {
        //area where sensor output tells what path to take
        if ((coneSensor.red() > coneSensor.blue()) && (coneSensor.red() > coneSensor.green())) { //(coneSensor.blue() & coneSensor.green())
            robotWhere = 1;
        }else {
            if ((coneSensor.green() > coneSensor.red()) && (coneSensor.green() > coneSensor.blue())) {//(coneSensor.red() & coneSensor.green())
                robotWhere = 2;
            }else{
                robotWhere = 3;
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
