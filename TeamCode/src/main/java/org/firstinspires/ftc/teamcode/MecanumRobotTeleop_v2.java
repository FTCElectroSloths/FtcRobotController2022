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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum: Teleop - v2", group="Mecanum")
//@Disabled
public class MecanumRobotTeleop_v2 extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardwareMecanum robot  = new RobotHardwareMecanum();     // Use a Mecanum's hardware

//    double clawOffset = 0;
//
//    public static final double MID_SERVO   =  0.5 ;
//    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables
         * The init(method of the hardware class does all the work here)
         */

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "ElectroSloths - v2, Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            int newUpTarget;
            int newDownTarget;

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            robot.driveRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            //robot.linearSlider.setPower(gamepad2.left_stick_y);

            double sliderSpeed = 0;
//            if (gamepad1.left_trigger != 0){
//                sliderSpeed = gamepad1.left_trigger;
//            } else {
//                sliderSpeed = -gamepad1.right_trigger;
//            }

            if (gamepad1.left_bumper){
                robot.linearSlider.setDirection(DcMotor.Direction.FORWARD);
                newUpTarget = robot.linearSlider.getCurrentPosition() + 1100;
                if(newUpTarget > 2000) {
                    newUpTarget = 2000;
                }
                robot.linearSlider.setTargetPosition(newUpTarget);  //newTarget
                robot.linearSlider.setPower(1);
                robot.linearSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(robot.linearSlider.isBusy() ){}
                //robot.linearSlider.setPower(0);
            }

            if (gamepad1.right_bumper){
                robot.linearSlider.setDirection(DcMotor.Direction.REVERSE);
                newDownTarget = 0; //robot.linearSlider.getCurrentPosition() - 1000;
                 robot.linearSlider.setTargetPosition(newDownTarget);  //newTarget
                robot.linearSlider.setPower(1);
                robot.linearSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(robot.linearSlider.isBusy() ){}
                //robot.linearSlider.setPower(0);
            }



            if (gamepad1.y){
                robot.linearSlider.setDirection(DcMotor.Direction.FORWARD);
            }

            if (gamepad1.a){
                robot.linearSlider.setDirection(DcMotor.Direction.REVERSE);
            }

            robot.linearSlider.setPower(gamepad1.left_trigger );
            //robot.linearSlider.setPower(-gamepad1.right_trigger);


            if (gamepad1.dpad_left){
                robot.openClaw();
            }

            if (gamepad1.dpad_right){
                robot.closeClaw();
            }


            telemetry.addData("x1",  "%.2f", gamepad1.left_stick_x);
            telemetry.addData("y1", "%.2f", -gamepad1.left_stick_y);
            telemetry.addData("Slide", "%.2f", (double) robot.linearSlider.getCurrentPosition());
            telemetry.addData("Left Trigger", "%.2f", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", "%.2f", gamepad1.right_trigger);
            telemetry.addData("Slider speed", "%.2f", sliderSpeed);
            telemetry.addData("Left Claw position", "%.2f", robot.clawLeftHand.getPosition());
            telemetry.addData("Right Claw position", "%.2f", robot.clawRightHand.getPosition());

            telemetry.update();

  /*
            robot.moveSliderManually(gamepad2.left_stick_y);

            if(gamepad1.a && robot.isSliderIdle)   {
                robot.lowerSlider();
                //robot.isSliderIdle = false;
            }

            if(gamepad1.x && robot.isSliderIdle)   {
                robot.raiseSlider(1);
                //robot.isSliderIdle = false;
            }

            if(gamepad1.y && robot.isSliderIdle)   {
                robot.raiseSlider(2);
                //robot.isSliderIdle = false;
            }

            if(gamepad1.b && robot.isSliderIdle)   {
                robot.raiseSlider(3);
                //robot.isSliderIdle = false;
            }
*/
            // Send telemetry message to signify robot running;
            //telemetry.addData("claw",  "Offset = %.2f", clawOffset);

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
