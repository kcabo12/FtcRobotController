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

package Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="TEST_TeleOp", group="Robot")
public class TEST_TeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    // ======================= MOTORS =======================
    public DcMotor leftFront   = null;
    public DcMotor leftBack   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightBack  = null;
    public DcMotor liftMotor = null;
    public DcMotor extensionMotor = null;
//    public DcMotor odometryX = null;
//    public DcMotor odometryY = null;
    public DcMotor ascentMotor1 = null;
    public DcMotor ascentMotor2 = null;

    // ======================= SERVOS =======================
    public Servo claw = null;
    double clawOffset = 0;
    public Servo intake = null;
    public Servo PivotServo = null;
    public Servo basketServo = null;

    // ======================= STATE MACHINES =======================
    int liftMotorStateMachine = 1;
    int intakeDropStateMachine = 0;
    int intakeReturnStateMachine = 0;
    int clawStateMachine = 1;
    int scaleSpeedStateMachine = 1;
    double scaleTurningSpeed = .8;
    double scaleFactor = 1; //.5;
    int direction = -1;

    // ======================= SENSORS =======================
    public DistanceSensor distanceSensor;
    HardwareMap hwMap = null;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime currentTime = new ElapsedTime();
    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ; // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    double v1, v2, v3, v4;
    public CameraName Webcam1;


    @Override
    public void runOpMode() {
        ElapsedTime intakeTimer = new ElapsedTime();

        double left;
        double right;
        double drive;
        double turn;
        double max;
        double liftMotorPower = 0;

        ElapsedTime liftTimer = new ElapsedTime();

        boolean rightStickButtonPushed;
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        float hsvValuesFloor[] = {0F,0F,0F};
//        int colorSensorState = 0, pixels = 0;

        // Define and Initialize Motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        ascentMotor1 = hardwareMap.get(DcMotor.class, "ascentMotor1");
        ascentMotor2 = hardwareMap.get(DcMotor.class, "ascentMotor2");
        extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        Webcam1  = hardwareMap.get(CameraName.class, "Webcam 1");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Only use these if necessary for configuration:
        ascentMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(Servo.class, "intake");
        PivotServo = hardwareMap.get(Servo.class, "PivotServo");
        basketServo = hardwareMap.get(Servo.class, "basketServo");

        // Send telemetry message to signify robot waiting;

        telemetry.addLine("__ Qualifier - Teleop Code Initialized");
        telemetry.addData(">", "TO TEST ROBOT, Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            currentTime.reset();

// ===================== ASCENT MOTOR =====================
            ascentMotor1.setPower(1);
            ascentMotor2.setPower(-1);
            sleep(2000);
            ascentMotor1.setPower(-1);
            ascentMotor2.setPower(1);
            sleep(2000);
            ascentMotor1.setPower(0);
            ascentMotor2.setPower(0);

// ===================== EXTENSION MOTOR =====================
            extensionMotor.setPower(0.8);
            sleep(200);
            extensionMotor.setPower(-0.8);
            sleep(200);
            extensionMotor.setPower(0);

// ===================== INTAKE SERVOS =====================
            PivotServo.setPosition(0);
            sleep(300);
            intake.setPosition(0.47); // opens intake claw

            // ===================== BASKET SERVO =====================
            basketServo.setPosition(1);
            sleep(1000);
            basketServo.setPosition(0);
            sleep(1000);

            intake.setPosition(0.25); // closes intake claw
            PivotServo.setPosition(0.86);

// ===================== LIFTMOTOR =====================
            liftMotor.setTargetPosition(1020); // lift liftmotor
            setLiftMotorClaw();
            sleep(2000);
            liftMotor.setTargetPosition(0); // pull down liftmotor
            setLiftMotorDown();

// ===================== CLAW =====================
            claw.setPosition(.7);
            sleep(500);
            claw.setPosition(1);
            sleep(500);
            claw.setPosition(.7);
            sleep(500);
            claw.setPosition(1);
            sleep(500);


// ==================================== TELEMETRY =========================================
            telemetry.addData("liftMotor pos: ", liftMotor.getCurrentPosition());
            telemetry.addData("liftMotor spd: ", liftMotor.getPower());
            telemetry.addData("extensionMotor ", extensionMotor.getCurrentPosition());
            // ======================= SERVOS =======================
            telemetry.addData("PivotServo: ", PivotServo.getPosition());
            telemetry.addData("intakeServo: ", intake.getPosition());
            telemetry.update();
        }
    }
    public void setLiftMotorClaw () {
        double liftMotorPower = 0;
        PivotServo.setPosition(0.5); // make sure intake claw is not in the way
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorPower = 1;
        liftMotor.setPower(liftMotorPower);
//        while (liftMotor.getCurrentPosition() < 4000){
//        }
//        liftMotor.setPower(.05);
//        sleep(1500);
    }
    public void setLiftMotorDown () {
        double liftMotorPower = 0;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorPower = 1;
        liftMotor.setPower(liftMotorPower);
    }
}