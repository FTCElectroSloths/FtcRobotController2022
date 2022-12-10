package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class RobotHardwareMecanum {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    /* Public OpMode member */
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor frontLeftDrive   = null;
    public DcMotor frontRightDrive  = null;
    public DcMotor backLeftDrive    = null;
    public DcMotor backRightDrive   = null;

    public DcMotor linearSlider     = null;

    public Servo   clawLeftHand     = null;
    public Servo   clawRightHand    = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO       =  0.5 ;
    public static final double CLAW_SPEED      =  0.02 ;  // sets rate to move servo

    public static final double SLIDER_UP_POWER    =    0.45 ;
    public static final double SLIDER_DOWN_POWER  =     -0.45 ;

    public static final double MOTOR_TICK_COUNT = 1120;
    public static final double SLIDER_LEVEL1_TICK_COUNT = 1000;
    public static final double SLIDER_LEVEL2_TICK_COUNT = 2000;
    public static final double SLIDER_LEVEL3_TICK_COUNT = 380;
    public static final double SLIDER_MAX_TICK_COUNT = 2300;
    public static final double SLIDER_MIN_TICK_COUNT = 0;


    int sliderCurrentPosition = 0;
    public boolean isSliderIdle = true;

    // Define a constructor that allows the OpMode to pass a reference to itself.
//    public RobotHardwareMecanum(LinearOpMode opmode) {
//        myOpMode = opmode;
//    }

    /* local OpMode members */
    HardwareMap hwMap   = null;

    // Constructor
    public RobotHardwareMecanum() {

    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap ahwMap)    {
        // Save the reference to the Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        frontLeftDrive  = hwMap.get(DcMotor.class, "MDriveFL");
        frontRightDrive = hwMap.get(DcMotor.class, "MDriveFR");
        backLeftDrive   = hwMap.get(DcMotor.class, "MDriveBL");
        backRightDrive   = hwMap.get(DcMotor.class, "MDriveBR");

        linearSlider   = hwMap.get(DcMotor.class, "MSlider");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        linearSlider.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        linearSlider.setPower(0);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //linearSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);   // sets the counter of ticks to 0


        // Define and initialize ALL installed servos.
        clawLeftHand = hwMap.get(Servo.class, "SClawLeft");
        clawRightHand = hwMap.get(Servo.class, "SClawRight");

//        clawLeftHand.setPosition(.75); //MID_SERVO higher opens up
//        clawRightHand.setPosition(.75); // lower opens
        // 12/9/22
//        clawLeftHand.setPosition(.45); //MID_SERVO higher opens up
//        clawRightHand.setPosition(.55); // lower opens

        clawLeftHand.setPosition(.25);
        clawRightHand.setPosition(.75);

//        myOpMode.telemetry.addData(">", "Hardware Initialized - Electo Sloths");
//        myOpMode.telemetry.update();
    }

    public void openClaw(){
        //setClawPositions(0.25);
        clawLeftHand.setPosition(.45); //.45
        clawRightHand.setPosition(.55); //.55
    }


    public void closeClaw(){
        //setClawPositions(-0.25);
        clawLeftHand.setPosition(.25); //.3
        clawRightHand.setPosition(.75); //.7
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param Drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */

    public void driveRobot(double Drive, double Turn, double Spin) {
        // Turn = left/right
        // Drive = forward/backward

        double fortyFiveInRads = -Math.PI/4;
        double cosine45 = Math.cos(fortyFiveInRads);
        double sin45 = Math.sin(fortyFiveInRads);

//        myOpMode.telemetry.addData("Drive (y)",  "%.2f", Drive);
//        myOpMode.telemetry.addData("Turn (x)",  "%.2f", Turn);
//        myOpMode.telemetry.update();

        double x;
        double y;

        // need to rotate 45 degrees
        y = Drive * cosine45 + Turn * sin45;
        x = Turn * cosine45 - Drive * sin45;

        if(Math.abs(Spin) > 0.1){
            // spin code
            frontRightDrive.setPower(-Spin/1.5);
            backRightDrive.setPower(-Spin/1.5);

            frontLeftDrive.setPower(Spin/1.5);
            backLeftDrive.setPower(Spin/1.5);
        } else {
            frontLeftDrive.setPower(x/1.25);  //x1
            backRightDrive.setPower(x/1.25);  //x1

            frontRightDrive.setPower(y/1.25);
            backLeftDrive.setPower(y/1.25);


        }


        // Drive both wheels.



    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftWheel     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightWheel    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */

    public void setDrivePower(double leftWheel, double rightWheel) {
        // Output the values to the motor drives.
        frontLeftDrive.setPower(leftWheel);
        backRightDrive.setPower(leftWheel);

        frontRightDrive.setPower(rightWheel);
        backLeftDrive.setPower(rightWheel);

    }


    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */

    public void setSliderPower(double power) {

        linearSlider.setPower(power);
    }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    public void setClawPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        clawLeftHand.setPosition(MID_SERVO + offset);
        clawRightHand.setPosition(MID_SERVO - offset);
    }

    /*
    public void raiseSlider(int level) {
        int newTarget = 0;
        int levelTarget = 0;

        //isSliderIdle = false;

        switch(level){
            case 1:
                levelTarget = (int)SLIDER_LEVEL1_TICK_COUNT;
                break;
            case 2:
                levelTarget = (int)SLIDER_LEVEL2_TICK_COUNT;
                break;
            case 3:
                levelTarget = (int)SLIDER_LEVEL3_TICK_COUNT;
                break;
        }

//        if (level == 1){
//            levelTarget = (int)SLIDER_LEVEL1_TICK_COUNT;
//        } else if (level == 2){
//            levelTarget = (int)SLIDER_LEVEL2_TICK_COUNT;
//        } else if (level == 3){
//            levelTarget = (int)SLIDER_LEVEL3_TICK_COUNT;
//        }

        newTarget = linearSlider.getTargetPosition() - levelTarget;

//        if (newTarget > (int)SLIDER_MAX_TICK_COUNT){
//            newTarget = (int)SLIDER_MAX_TICK_COUNT;
//        }

        linearSlider.setTargetPosition(-500);  //newTarget
        linearSlider.setPower(SLIDER_DOWN_POWER);
        linearSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (linearSlider.isBusy()){
//            myOpMode.telemetry.addData("Slider position", linearSlider.getCurrentPosition());
//            myOpMode.telemetry.update();
        }

        linearSlider.setPower(-0.02);
        sliderCurrentPosition = newTarget;

        //isSliderIdle = true;

    }

    public void lowerSlider() {
        int newTarget = 0;

        newTarget = linearSlider.getTargetPosition() + sliderCurrentPosition;

//        if (newTarget < (int)SLIDER_MIN_TICK_COUNT){
//            newTarget = (int)SLIDER_MIN_TICK_COUNT;
//        }
        //newTarget = (newTarget < (int)SLIDER_MIN_TICK_COUNT) ? (int)SLIDER_MIN_TICK_COUNT : newTarget;

        linearSlider.setTargetPosition(-500);  //newTarget
        linearSlider.setPower(SLIDER_DOWN_POWER);
        linearSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (linearSlider.isBusy()){
//            myOpMode.telemetry.addData("Slider position", linearSlider.getCurrentPosition());
//            myOpMode.telemetry.update();
        }

        linearSlider.setPower(0);
        sliderCurrentPosition = newTarget;

    }

    public void moveSliderManually(double power) {
        linearSlider.setPower(power);
//        myOpMode.telemetry.addData("Slider position", linearSlider.getCurrentPosition());
//        myOpMode.telemetry.update();
    }
*/

}