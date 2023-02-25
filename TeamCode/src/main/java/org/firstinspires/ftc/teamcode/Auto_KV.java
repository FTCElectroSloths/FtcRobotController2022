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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous(name="AUTO_KV", group="AUTOKV")
//@Disabled
public class Auto_KV extends LinearOpMode {

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

            //set motors to run with encoders
            driveFLM.setMode(RUN_USING_ENCODER);
            driveFRM.setMode(RUN_USING_ENCODER);
            driveBLM.setMode(RUN_USING_ENCODER);
            driveBRM.setMode(RUN_USING_ENCODER);

            // set motors to brake, so they dont move during initialization

            driveFLM.setZeroPowerBehavior(BRAKE);
            driveFRM.setZeroPowerBehavior(BRAKE);
            driveBLM.setZeroPowerBehavior(BRAKE);
            driveBRM.setZeroPowerBehavior(BRAKE);

            telemetry.addData("ready?: ", "stupid bitch");
            telemetry.update();

            //color sense telemetry
            while (opModeIsActive()) {
                telemetry.addData("red", coneSensor.red());
                telemetry.addData("green", coneSensor.green());
                telemetry.addData("blue", coneSensor.blue());
                telemetry.update();
            }

            //initializing components
            clawLeftHand.setPosition(.25);
            clawRightHand.setPosition(.75);

        }

        waitForStart();

        //robot moving
        // useful information: wait = sleep(time in milliseconds); 1sec = 1000millis
        closeClaw();
        driveBackwardE(0.5,2600);
        colorsense();
        sleep(200);

        //robot where code
        if (robotWhere == 0) {
            //doing cone
            //code

            //parking
            moveRightE(1, 1000);

        }
        if (robotWhere == 1) {
            //doing cone
            doConeThings();

            //parking
            sleep(500);
        }
        if (robotWhere == 2) {
            //doing cone
            //code

            //parking
            moveLeftE(1, 1000);
        }

    }

    //mini methods for ease
    //cone things
    private void doConeThings() throws InterruptedException {
        driveForwardE(.5, 600);
        sleep(100);
        spinLeftE(.4, 1325);
        slideUp(1, 2750);
        driveForwardE(.5,350);
        sleep(200);
        slideDown(.5, 100);
        openClaw();
        slideDown(.75, 2150);
        LinearSlider.setPower(0);

        //mini methods execute
        goingStack();

        goingScoring();

        //stack time
        sleep(300);
        driveBackwardE(.5, 325);
        slideDown(.5, 50);
        spinLeftE(.5, 1200);
        driveForwardE(.5, 1030);
        closeClaw();
        sleep(200);
        slideUp(1, 2450);
        LinearSlider.setPower(0);
        sleep(300);

        goingScoring();
        driveBackwardE(.5, 325);
        spinRightE(.5, 1325);
        closeClaw();
        slideDown(.5, 570);
    }

    //sub mini methods for ease
    //go to cone
    private void goingScoring() throws InterruptedException {
        driveBackwardE(.5, 1025);
        spinRightE(.5, 1200);
        slideUp(1, 50);
        driveForwardE(.5, 325);
        sleep(100);
        slideDown(.5, 100);
        sleep(100);
        openClaw();
        slideDown(.5, 2300);
        LinearSlider.setPower(0);
    }

    //to stack
    private void goingStack() throws InterruptedException {
        sleep(300);
        driveBackwardE(.5, 325);
        spinLeftE(.5, 1200);
        driveForwardE(.5, 1030);
        closeClaw();
        sleep(200);
        slideUp(1, 2400);
        LinearSlider.setPower(0);
        sleep(300);
    }

    //ALL YOUR METHODS:
    //drive forward method
    private void driveForwardE(double power, int ticks) throws InterruptedException {
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

    //drive backward method
    private void driveBackwardE(double power, int ticks) throws InterruptedException {

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

    //spin left method
    private void spinLeftE(double power, int ticks) throws InterruptedException {

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

    //spin right method
    private void spinRightE(double power, int ticks) throws InterruptedException {

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

        LinearSlider.setMode(STOP_AND_RESET_ENCODER);
        LinearSlider.setTargetPosition(ticks);
        LinearSlider.setMode(RUN_TO_POSITION);
        LinearSlider.setPower(power);
        while (LinearSlider.isBusy()) {

        }
        LinearSlider.setPower(0);
        LinearSlider.setZeroPowerBehavior(BRAKE);
        LinearSlider.setMode(RUN_USING_ENCODER);
    }

    //linear slide down
    private void slideDown(double power, int ticks) throws InterruptedException {

        LinearSlider.setMode(STOP_AND_RESET_ENCODER);
        LinearSlider.setTargetPosition(-ticks);
        LinearSlider.setMode(RUN_TO_POSITION);
        LinearSlider.setPower(power);
        while (LinearSlider.isBusy()) {

        }
        LinearSlider.setPower(0);
        LinearSlider.setZeroPowerBehavior(BRAKE);
        LinearSlider.setMode(RUN_USING_ENCODER);
    }

    //color sensor sensing code
    private void colorsense() {

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
