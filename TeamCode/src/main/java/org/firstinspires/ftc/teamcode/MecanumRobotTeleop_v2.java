package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

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
    RobotHardwareMecanum robot = new RobotHardwareMecanum();     // Use a Mecanum's hardware

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

            int newUpTarget = 0 ;
            int newDownTarget = 0;

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

/*            if (gamepad1.left_bumper) {
                robot.closeClaw();
                //wait(0.25);
                sleep(250);
                robot.linearSlider.setDirection(DcMotor.Direction.FORWARD);

                newUpTarget = robot.linearSlider.getCurrentPosition() + 1200;  //1200
                //newUpTarget = robot.linearSlider.getCurrentPosition() + 1200;  //1200
                //newUpTarget = 2400
                                if (newUpTarget > 2400) {
                    newUpTarget = 2400;
                }
                robot.linearSlider.setTargetPosition(newUpTarget);  //newTarget
                robot.linearSlider.setPower(1);
                robot.linearSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (robot.linearSlider.isBusy()) {
                }
                //robot.linearSlider.setPower(0);
            }
*/
            if (gamepad1.left_bumper) {

                robot.closeClaw();
                //wait(0.25);
                sleep(250);
                robot.linearSlider.setDirection(DcMotor.Direction.FORWARD);
                if (newUpTarget <1000) {
                    newUpTarget = Integer.max(1400,robot.linearSlider.getCurrentPosition() + 1400);  //1200

                }
                else{
                    newUpTarget = Integer.max(2200,robot.linearSlider.getCurrentPosition() + 800);  //1200
                }


                //newUpTarget = robot.linearSlider.getCurrentPosition() + 1200;  //1200
                //newUpTarget = 2200
                if (newUpTarget > 2200) {
                    newUpTarget = 2200;
                }
                robot.linearSlider.setTargetPosition(newUpTarget);  //newTarget
                robot.linearSlider.setPower(1);
                robot.linearSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (robot.linearSlider.isBusy()) {
                }
                //robot.linearSlider.setPower(0);
            }

            if (gamepad1.right_bumper) {
                robot.openClaw();
                sleep(200);
                robot.driveRobot(-0.35,0,0);

                robot.linearSlider.setPower(0);
                sleep(200);
                robot.closeClaw();

                robot.linearSlider.setDirection(DcMotor.Direction.REVERSE);
                newDownTarget = 0; //robot.linearSlider.getCurrentPosition() - 1000;
                robot.linearSlider.setTargetPosition(newDownTarget);  //newTarget
                robot.linearSlider.setPower(.75);
                robot.linearSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (robot.linearSlider.isBusy()) {
                }
                sleep(200);
                robot.openClaw();
                //robot.linearSlider.setPower(0);
            }



           /* if (gamepad1.y){
                robot.linearSlider.setDirection(DcMotor.Direction.FORWARD);
            }

            if (gamepad1.a){
                robot.linearSlider.setDirection(DcMotor.Direction.REVERSE);
            }*/

            robot.linearSlider.setPower(gamepad1.left_trigger);
            //robot.linearSlider.setPower(-gamepad1.right_trigger);


            /*if (gamepad1.dpad_left){
                robot.closeClaw();
            }

            if (gamepad1.dpad_right) {
                robot.openClaw();
            }*/
            if (gamepad1.y || gamepad1.x || gamepad1.b) {
                robot.openClaw();
            }
            if (gamepad1.a) {
                robot.closeClaw();
            }

            //if(gamepad1.right_trigger){

            // }


            telemetry.addData("x1", "%.2f", gamepad1.left_stick_x);
            telemetry.addData("y1", "%.2f", -gamepad1.left_stick_y);
            telemetry.addData("Slide", "%.2f", (double) robot.linearSlider.getCurrentPosition());
            telemetry.addData("Left Trigger", "%.2f", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", "%.2f", gamepad1.right_trigger);
            telemetry.addData("Slider speed", "%.2f", sliderSpeed);
            telemetry.addData("Left Claw position", "%.2f", robot.clawLeftHand.getPosition());
            telemetry.addData("Right Claw position", "%.2f", robot.clawRightHand.getPosition());

            telemetry.update();

 /*
            //robot.moveSliderManually(gamepad2.left_stick_y);

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