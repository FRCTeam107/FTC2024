/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor. See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial: Driving forward and backward Left-joystick Forward/Backward
 * 2) Lateral: Strafing right and left Left-joystick Right and Left
 * 3) Yaw: Rotating Clockwise and counter clockwise Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor towerMotor = null;
    private CRServo samplePickup = null;
    private DcMotor armMotor = null;
    private DcMotor climberMotor = null;
    private Servo hookServo = null;


//sample operation


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        towerMotor = hardwareMap.get(DcMotor.class, "tower_motor");
        samplePickup = hardwareMap.get(CRServo.class, "sample_pickup");
        hookServo = hardwareMap.get(Servo.class, "hook_servo");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        climberMotor = hardwareMap.get(DcMotor.class, "climber_motor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        towerMotor.setDirection(DcMotor.Direction.REVERSE);

        boolean activelyIntaking = false;
        boolean liftToggleUp = false;
        boolean liftToggleDown = false;

        towerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        towerMotor.setTargetPosition(0);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        towerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -0.5 * gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double lateral = 0.5* gamepad1.left_stick_x;
            double yaw = 0.5* gamepad1.right_stick_x;
            boolean sample_in = gamepad1.left_bumper;
            boolean sample_out = gamepad1.right_bumper;

            boolean tower_up = gamepad2.dpad_right;
            boolean tower_down = gamepad2.dpad_left;
            boolean tower_top = gamepad2.dpad_up;
            boolean tower_low_basket = gamepad2.dpad_down;
            boolean tower_bottom = gamepad2.b;
            boolean tower_ground = gamepad2.a;
            boolean climber_up = gamepad2.right_stick_button;
            boolean climber_down = gamepad2.left_stick_button;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            //change this to output the current value of the encoder




            if (sample_in) {
                samplePickup.setPower(-1);
                armMotor.setTargetPosition(1450);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
                towerMotor.setTargetPosition(400);
                towerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                towerMotor.setPower(1);
                activelyIntaking=true;
            } else if (sample_out) {
                samplePickup.setPower(1);
            } else {
                samplePickup.setPower(0);
            }


            if(activelyIntaking && !sample_in){
                samplePickup.setPower(0);
                armMotor.setTargetPosition(900);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
                towerMotor.setTargetPosition(400);
                towerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                towerMotor.setPower(1);
                activelyIntaking=false;
            }


            if (tower_top) {
                towerMotor.setTargetPosition(3750);
                towerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                towerMotor.setPower(1);
                armMotor.setTargetPosition(200);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);

            } else if (tower_low_basket) {
                towerMotor.setTargetPosition(2000);
                towerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                towerMotor.setPower(1);
                armMotor.setTargetPosition(500);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);

            } else if (tower_ground) {
                towerMotor.setTargetPosition(0);
                towerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                towerMotor.setPower(.5);

            //set tower to pickup position. This should not need to be used.
            } else if (tower_bottom) {
                towerMotor.setTargetPosition(200);
                towerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                towerMotor.setPower(.5);

//            //move tower down until encoder at zero
//            }else if(tower_down && towerMotor.getCurrentPosition() < 0) {
//                towerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                towerMotor.setPower(-.75);


            } else if(tower_up && !liftToggleUp) {
                towerMotor.setTargetPosition(towerMotor.getTargetPosition()+100);
                towerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                towerMotor.setPower(1);
                liftToggleUp=true;

            } else if(tower_down && !liftToggleDown) {
                towerMotor.setTargetPosition(towerMotor.getTargetPosition()-100);
                towerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                towerMotor.setPower(1);
                liftToggleDown=true;
            }

            //Reset the variables for toggling the lift up/down
            if (!tower_up && liftToggleUp) {
                liftToggleUp = false;
            }
            if (!tower_down && liftToggleDown) {
                liftToggleDown = false;
            }
//            //set flipper motor position
//            if (gamepad2.right_bumper) {
//                flipperMotor.setTargetPosition(-200);
//                flipperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                flipperMotor.setPower(1);
//
//                telemetry.addData("current spot:", "%7d", flipperMotor.getCurrentPosition());
//                telemetry.update();
//
//            } else if (gamepad2.left_bumper) {
//                flipperMotor.setTargetPosition(-950);
//                flipperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                flipperMotor.setPower(1);
//
//                telemetry.addData("current Waffle:", "%7d", flipperMotor.getCurrentPosition());
//                telemetry.update();
//
//
//            } else {
//                if (!flipperMotor.isBusy()) {
//                    flipperMotor.setPower(0.0);
//                } else { }
//            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Lift Position", "%7d", towerMotor.getCurrentPosition());
            telemetry.addData("Arm Position", "%7d", flipperMotor.getCurrentPosition());
            telemetry.update();
        }
    }}