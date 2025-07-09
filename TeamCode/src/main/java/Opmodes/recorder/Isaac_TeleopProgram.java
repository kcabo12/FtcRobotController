package Opmodes.recorder;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// TODO: 7/1/2025 Isaac, please read the following message:
/*

Hello Isaac!

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
@Disabled
@TeleOp(name="Tutorial_TeleopProgram", group="Robot")
public class Isaac_TeleopProgram extends LinearOpMode {


    // Initialization:
    // NOTE: In the Example Formats, do not include the brackets [] with your input


    // ======================= MOTORS =======================
    // Example format:
    // public [motor type] [insert name of motor] = [insert power/position];

    // Your input:




    // ======================= SERVOS =======================
    // Example format:
    // public [servo type] [insert name of servo] = [insert power/position];

    // Your input:



    @Override
    public void runOpMode() {

        // Define and Initialize Motors
        // Example format:
        // [name of motor] = hardwareMap.get([motor type].class, "[name of motor]");
        // Your input:


//        DcMotor rightfront DcMotor leftfront;
//        DcMotor rightback DcMotor leftback;





        // Define and Initialize Servos
        // Example format:
        // [name of servo] = hardwareMap.get([servo type].class, "[name of servo]");

        // Your input:





        waitForStart();



        while (opModeIsActive()) {
            // Set power to motors:
            // Example Format:
            // [name of motor].setpower([insert value here]);












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






        }
    }
}