package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import javax.sql.StatementEvent;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@TeleOp(name = "TeleOp_v2", group = "Sample")
public class TeleOp_v2 extends LinearOpMode {

    //declare Motors
    private DcMotor MSlider;

    //declare servos
    private Servo SClawLeft;
    private Servo SClawRight;

    private double driveSpeed = 0.6;
    private boolean MikeMode = true;
    private boolean flipperUp = false;
    private int i = 0;

    private boolean stopDown = false;
    private double stageNum = 0;
    private double stageTarget;
    private int stageTicks;
    private boolean isStagingUp = false;
    private boolean isStagingDown = false;

    private final int stage0 = 0;
    private final int stage1 = 100;
    private final int stage2 = 200;
    private final int stage3 = 300;


    @Override
    public void
    runOpMode() {

        //configuration
        if (true){

            //configure motors
            MSlider = hardwareMap.dcMotor.get("MSlider");

            //configure servos
            SClawLeft = hardwareMap.servo.get("SClawLeft");
            SClawRight = hardwareMap.servo.get("SClawRight");

            //configure motor directions
            MSlider.setDirection(DcMotor.Direction.FORWARD); //we want the slider to go up;FORWARD = clockwise;look from wire to front of motor
            MSlider.setZeroPowerBehavior(BRAKE);
        }

        waitForStart();

        //SClawLeft.setPosition(0);
        //SClawRight.setPosition(0);

        while(opModeIsActive()){

            if (gamepad1.a) {
                //to do: do we need to initialize?
                MSlider.setDirection(DcMotor.Direction.REVERSE);
                MSlider.setPower(driveSpeed);
            }
            if (gamepad1.x) {
                //to do: do we need to initialize?
                MSlider.setDirection(DcMotor.Direction.FORWARD);
                MSlider.setPower(driveSpeed);
            }
            if (gamepad1.y) {
                //to do: do we need to initialize?
                MSlider.setDirection(DcMotor.Direction.FORWARD);
                MSlider.setPower(driveSpeed);
            }
            if (gamepad1.b) {
                //to do: do we need to initialize?
                MSlider.setDirection(DcMotor.Direction.FORWARD);
                MSlider.setPower(driveSpeed);
            }
            //find the stage number
            if (true) {
                if (MSlider.getCurrentPosition() <= 50) stageNum = 0.0;
                if (MSlider.getCurrentPosition() > stage0+50) stageNum = 0.5;
                if (MSlider.getCurrentPosition() > stage1-50) stageNum = 1.0;
                if (MSlider.getCurrentPosition() > stage1+50) stageNum = 1.5;
                if (MSlider.getCurrentPosition() > stage2-50) stageNum = 2.0;
                if (MSlider.getCurrentPosition() > stage2+50) stageNum = 2.5;
                if (MSlider.getCurrentPosition() > stage3-50) stageNum = 3.0;
                if (MSlider.getCurrentPosition() > stage3+50) stageNum = 3.5;
            }
//for each press of the a,x,y,b buttons move the stageTargets a goes down; x goes until stage 1; y goes until stage2; b goes until
            //if (gamepad1.x /*&& stageNum<2.0*/) {
            if (gamepad1.x && stageNum<2.0) {

                stageTarget=1;//Math.ceil(stageNum+0.5);
                isStagingUp=true;
            }
            //if (gamepad1.y /*&& stageNum<3.0*/) {
           if (gamepad1.y && stageNum<3.0) {

                stageTarget=2;//Math.ceil(stageNum+0.5);
                isStagingUp=true;
            }
            //if (gamepad1.b /*&& stageNum<4.0*/) {
            if (gamepad1.b && stageNum<4.0) {

                stageTarget=3;//Math.ceil(stageNum+0.5);
                isStagingUp=true;
            }
            //if (gamepad1.a /*&& stageNum>0*/) {
            if (gamepad1.a && stageNum>0) {
                stageTarget=0;//Math.floor(stageNum-0.5);
                isStagingDown=true;
            }

            if (true) {
                if (stageTarget==0) stageTicks=stage0;
                if (stageTarget==1) stageTicks=stage1;
                if (stageTarget==2) stageTicks=stage2;
                if (stageTarget==3) stageTicks=stage3;
            }
            if (isStagingDown) {
                MSlider.setDirection(DcMotor.Direction.REVERSE);
                MSlider.setTargetPosition(stageTicks);
                MSlider.setPower(-0.75);

                while (MSlider.isBusy() && !stopDown) {
                }
                stopDown = false;
                isStagingDown = false;
                MSlider.setPower(0);

            }
            if (isStagingUp) {
                MSlider.setDirection(DcMotor.Direction.FORWARD);
                MSlider.setTargetPosition(stageTicks);
                MSlider.setPower(0.75);

                while (MSlider.isBusy() && !stopDown) {
                    if (gamepad1.a) {
                        stopDown = true;
                    }
                }
                stopDown = false;
                isStagingUp = false;
                MSlider.setPower(0);
            }

        }
    }
}
