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

package Opmodes.recorder;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
/* I imported Range as well, I'm not sure if there is a different way to do this, but in both the tutorials I watched
   they used "Range.clip" to set the power of the motors with the left joystick input. - Colson
 */
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;

// TODO: 4/14/2025 Colson, please read the following message:
/*

Colson,

Welcome to your personal TeleOp program! I know you are eager to get into the actual programming, so I will do my best to guide you through it.

Now that you have watched the videos I sent you, you should have a general understanding of what TeleOp is, and how it works.
I know it's new, but I want to give you the chance to work on it hands-on. In the video, you learned the program layout, initialization, and how to
control motors and servos. Although the video is old, and there's a little more to the "initialization", I want you to use what you learned in the video
to attempt the following:

todo Review the following tasks, and attempt to complete them:
1. Initialize the robot's wheel motors
(Hint: we name these motors "leftFront" "leftBack" "rightFront" "rightBack")
(Hint: the motor types we use are called "DcMotor")
(Hint: at the start of the program, a motor or servo should be initialized with a power/position of "null")

2. Initialize a servo
(Hint: you can call this servo "claw")
(Hint: the servo types for claws are called "Servo")
(Hint: at the start of the program, a motor or servo should be initialized with a power/position of "null")

3. Set the power of these motors at 80%
(Hint: when setting a power, 1 = 100% and 0 = 0%)

4. Setup a conditional statement for the claw, where the button "a" will open the claw and "b" will close it
(Hint: For this exercise, we will use gamepad1)
(Hint: For this exercise, we will assume the value 1 opens the claw and 0 closes it)

NOTE: You do not have to get through all of this in one night! I want you to take your time and make sure you understand what you are doing.
Please ask for help if you need it. You can refer back to the video, look stuff up online, or ask either me or Coach Pavan.
Scroll down and review each section of the code; I put the titles, descriptions, and formatting examples to help you. So PLEASE READ THEM
Good luck!!

- Andrew :)

 */

@TeleOp(name="Colson_TeleopProgram", group="Robot")
public class Colson_TeleopProgram extends LinearOpMode {


    // Initialization:
    // NOTE: In the Example Formats, do not include the brackets [] with your input


    // ======================= MOTORS =======================
    // Example format:
    // public [motor type] [insert name of motor] = [insert power/position];

    // Your input:
    DcMotor leftFront, leftBack;
    DcMotor rightFront, rightBack;

    DcMotor liftMotor;




    // ======================= SERVOS =======================
    // Example format:
    // public [servo type] [insert name of servo] = [insert power/position];

    // Your input:
    Servo claw;

    @Override
    public void runOpMode() {

        // Define and Initialize Motors
        // Example format:
        // [name of motor] = hardwareMap.get([motor type].class, "[name of motor]");
        double liftMotorPower = 0;
        // Your input:
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);





        // Define and Initialize Servos
        // Example format:
        // [name of servo] = hardwareMap.get([servo type].class, "[name of servo]");

        // Your input:
        claw = hardwareMap.get(Servo.class, "claw");


        telemetry.addLine("__ Qualifier - Teleop Code Initialized");
        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();



        while (opModeIsActive()) {
            // Set power to motors:
            // Example Format:
            // [name of motor].setpower([insert value here]);
            leftFront.setPower(Range.clip(gamepad1.left_stick_y, -.8, .8));
            leftBack.setPower(Range.clip(gamepad1.left_stick_y, -.8, .8));
            rightFront.setPower(Range.clip(gamepad1.right_stick_y, -.8, .8));
            rightBack.setPower(Range.clip(gamepad1.right_stick_y, -.8, .8));

            liftMotorPower = 0;
            if (gamepad2.right_stick_y < -0.03) {
                telemetry.addData("right_stick_y < -0.03", "");
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotorPower = 0.5;
            }
            else if (gamepad2.right_stick_y > 0.03) {
                telemetry.addData("right_stick_y > 0.03", "");
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotorPower = 0.4;
            }
            else if (gamepad2.left_stick_y < -0.03) {
                telemetry.addData("left_stick_y < -0.03", "");
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotorPower = 1;
            }
            else if (gamepad2.left_stick_y > 0.03) {
                telemetry.addData("left_stick_y > 0.03", "");
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotorPower = -1;
            }
            liftMotor.setPower(liftMotorPower);


            // Your input:




            // Create condition statement for claw:
            // Example Format:
            /*

            if ([name of gamepad].[button of choice] == true) {
                [name of servo].setPosition([insert value of position]);
            }
            else if ([name of gamepad].[button of choice] == true) {
                [name of servo].setPosition([insert value of position]);
            }

            */

            // Your input:
            if (gamepad1.a) {
                claw.setPosition(1);
            }
            else if (gamepad1.b) {
                claw.setPosition(.65);
            }



            }
        }
    }