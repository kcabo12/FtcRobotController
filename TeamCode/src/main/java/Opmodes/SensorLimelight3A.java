 /*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


//=====================================================================================
//* * * * * * * * * * * * * * * CURRENT AUTONOMOUS CODE * * * * * * * * * * * * * * * *
//=====================================================================================

 package Opmodes;

import android.graphics.Color;
import android.support.v4.os.IResultReceiver;
import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.SimplifiedOdometryRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;
import java.util.function.ToDoubleFunction;

 /*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https:/ /limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
// @Disabled
@Autonomous(name = "Sensor: Limelight3A", group = "Sensor")
public class SensorLimelight3A extends LinearOpMode {

    public SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    public Limelight3A limelight;
    public CameraName Webcam1;
    public double expectedStartPoseX = 0;
    public double expectedStartPoseY = 0;
    boolean runonce = false;
    boolean prelimelight = false;
    boolean preprogram = false;
    // ======================= MOTORS =======================
    public DcMotor leftFront   = null;
    public DcMotor leftBack   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightBack  = null;
    public DcMotor liftMotor = null;
    public DcMotor ascentMotor1 = null;
    public DcMotor ascentMotor2 = null;
    public DcMotor extensionMotor  = null;
//    public DcMotor odometryX = null;
//    public DcMotor odometryY = null;

    // ======================= SERVOS =======================
    public Servo claw = null;
    double clawOffset = 0;
    public Servo intake = null;
    public Servo PivotServo = null;
    public Servo basketServo = null;
    // ======================= STATE MACHINES =======================
    int liftMotorStateMachine = 1;
    int clawStateMachine = 1;
    int scaleSpeedStateMachine = 1;
    double scaleTurningSpeed = .8;
    double scaleFactor = .5;
    int direction = -1;

    private static SampleAuto_IntoTheDeep.START_POSITION startPosition = SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT;
    enum START_POSITION {
        //IN RELATION TO SUBMERSIBLE
        RED_BLUE_RIGHT,
        RED_BLUE_LEFT,
        RED_BLUE_LEFT_SAMPLE_ONLY,
    }
    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(true);

        // Define and Initialize Motors/Sensors/Servos
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        Webcam1  = hardwareMap.get(CameraName.class, "Webcam 1");
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        ascentMotor1 = hardwareMap.get(DcMotor.class, "ascentMotor1");
        ascentMotor2 = hardwareMap.get(DcMotor.class, "ascentMotor2");
//        odometryX = hardwareMap.get(DcMotor.class, "odometryX");
//        odometryY = hardwareMap.get(DcMotor.class, "odometryY");

        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(Servo.class, "intake");
        PivotServo = hardwareMap.get(Servo.class, "PivotServo");
        basketServo = hardwareMap.get(Servo.class, "basketServo");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odometryX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        odometryY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odometryX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        odometryY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Only use these if necessary for configuration:
        ascentMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
        telemetry.setMsTransmissionInterval(11);//50);

        telemetry.addData(">", "Robot Ready.  Press Play.");
//        telemetry.addData("Robot Starting Position", "X: %.2f", "Y: %.2f");
        telemetry.update();
        selectStartingPosition();

        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto
        limelight.start();
        limelight.pipelineSwitch(0);

        while (opModeIsActive() && runonce == false) {

//            PivotServo.setPosition(0.5);
//            extensionMotor.setTargetPosition(780); // extend extensionMotor
//            setExtensionMotor();
//            sleep(1000);
//
//            // test code
//            PivotServo.setPosition(0);
//            intake.setPosition(0.47);
//            sleep(200);
//            intake.setPosition(0.25);
//            sleep(300);
//
//            PivotServo.setPosition(0.5);
//            robot.turnToWithTimeout(-140,0.45,0.5,2);
//            PivotServo.setPosition(0);
//            intake.setPosition(0.47);
//            robot.turnToWithTimeout(0,0.45,0.1,2);
//
////            intakeSequence();
////            sleep(700); // lets the sample drop in the basket
////            intake.setPosition(0.47); // opens intake claw
////            sleep(300); // give the sample enough time to land in the basket
////            PivotServo.setPosition(0); // make sure intake claw is not in the way
////            claw.setPosition(0.7); // open claw
//
////                    robot.driveWithTimeout(15,0.5,0.25,2);    // grab second specimen
//
//            // CHUCK SAMPLE INTO OBSERVATION ZONE
////            basketServo.setPosition(0.5);
//            // SIMULTANEOUSLY DRIVE INTO OBSERVATION ZONE
//            robot.driveWithTimeout(-18,0.3,0.1,2);
//            // SIMULTANEOUSLY RESET LIFTMOTOR
////            liftMotor.setTargetPosition(1000);
////            setLiftMotorClaw();
////            basketServo.setPosition(0); // return basket
////            liftMotor.setTargetPosition(0);
////            setLiftMotorDown();
//            robot.driveWithTimeout(-4,0.3,0.25,0.5);
//
//            // GRAB SPECIMEN
//            claw.setPosition(1); // close claw
//            sleep(3000);

///** PAVAN ADDITIONS
            // PREREQUISITE PROGRAM FOR BOTH SIDES
            // ======================= DRIVE
//            intake.setPosition(0.47); // open intake claw
//            claw.setPosition(1); // close claw
////                sleep(200);
//            liftMotor.setTargetPosition(1120); // lift liftmotor
//            setLiftMotorClaw();
////                robot.driveWithTimeout(-21.75, 0.9, 0,1);
////                robot.driveWithTimeout(-8.5, 0.3, 0.2, 1);
//            robot.driveWithTimeout(-15.75, 0.9, 0,1);
//            robot.driveWithTimeout(-14.5, 0.3, 0.3, 1);
//            // ======================= PLACE SPECIMEN
//            liftMotor.setTargetPosition(0); // pull down liftmotor
//            setLiftMotorDown();
            // END PREREQUSITE PROGRAM






            //START GRAB SAMPLE AND GRAB SPECIMIN

//            robot.driveWithTimeout(15,0.5,0.25,2);    // grab second specimen
////                    robot.driveWithTimeout(-20,0.5,0.1,2);
//            robot.driveWithTimeout(-18,0.5,0.1,2);
////                    liftMotor.setTargetPosition(1000);
////                    setLiftMotorClaw();
////                    basketServo.setPosition(0); // return basket
////                    claw.setPosition(0.7);
////                    liftMotor.setTargetPosition(0);
////                    setLiftMotorDown();
//            robot.driveWithTimeout(-4,0.3,0.25,0.5);
//            claw.setPosition(1); // close claw
//            sleep(200);
//            liftMotor.setTargetPosition(1120); // lift liftmotor
//            setLiftMotorClaw();
////                    sleep(500);
//            robot.driveWithTimeout(15,0.5,0.25,2);
//            intake.setPosition(0.47); // make sure intake claw is not in the way
//            robot.turnToWithTimeout(180,0.55,0.25,2);
//            robot.strafe(-35, 0.9, 0);
//            robot.strafeWithTimeout(-10, 0.55, 0.1,2.5);
//            robot.driveWithTimeout(-15,0.4,0.1,2);
//            liftMotor.setTargetPosition(0); // pull down liftmotor
//            setLiftMotorDown();
////                    sleep(1200);
//            claw.setPosition(0.7); // open claw
//
//            robot.driveWithTimeout(-15,0.4,0.1,2);

            //START GRAB SAMPLE AND GRAB SPECIMIN






// ============================= W O R K I N G =============================================
//            claw.setPosition(1); // close claw
//            //sleep(200);
//            liftMotor.setTargetPosition(1020); // lift liftmotor
//            setLiftMotorClaw();
////          sleep(500);
//            robot.driveWithTimeout(10,1,0,2);
//            //sleep(500);
//            robot.turnTo(-15,0.5,0);
//            //sleep(500);
//            robot.strafeWithTimeout(36,1,0,10);
//            //sleep(500);
//            robot.turnTo(180,0.5,0);
//            //sleep(500);
//            robot.driveWithTimeout(-8,0.4,0,1.3);
//            //sleep(500);
//            liftMotor.setTargetPosition(0); // pull down liftmotor
//            setLiftMotorDown();
//
//            //~~~~~~~~END FIRST PLACEMENT
//            //~~~~~~~~END FIRST PLACEMENT
//            //~~~~~~~~END FIRST PLACEMENT
//            robot.driveWithTimeout(7,0.6,0,2);
//            claw.setPosition(0.7); // open claw
//            robot.turnTo(-110,0.5,0);
//            robot.drive(32,1,0);
//            robot.turnTo(0,0.5,0);
//            robot.strafe(-2,0.6,0.1);
//            robot.driveWithTimeout(-8,0.4,0.1,1);
//            //sleep(500);
//            //~~~~~~~~END THIRD SPECEMIN GRAB
//            //~~~~~~~~END THIRD SPECEMIN GRAB
//            //~~~~~~~~END THIRD SPECEMIN GRAB
//
////            sleep(3000);
//
//
//            claw.setPosition(1); // close claw
//            sleep(150);
//            liftMotor.setTargetPosition(1020); // lift liftmotor
//            setLiftMotorClaw();
////                    sleep(500);
//            robot.driveWithTimeout(10,1,0,2);
//            robot.turnTo(-15,0.5,0);
//            intake.setPosition(0.47); // make sure intake claw is not in the way
//            robot.strafeWithTimeout(36,1,0,10);
//            robot.turnTo(180,0.5,0);
//            robot.driveWithTimeout(-8,0.4,0.1,2);
//            liftMotor.setTargetPosition(0); // pull down liftmotor
//            setLiftMotorDown();
//            sleep(300);
//            claw.setPosition(0.7); // open claw
// ============================= W O R K I N G =============================================


//** END PAVAN ADDITIONS **/
//            PivotServo.setPosition(0.5);
//            intake.setPosition(0.47); // opens intake claw
//            extensionMotor.setTargetPosition(740); // extend extensionMotor
//            setExtensionMotor();
//            sleep(1700);
//
//            // place second sample
//            intakeSequence();
//            sleep(700); // lets the sample drop in the basket
//            intake.setPosition(0.47); // opens intake claw
//            sleep(300); // give the sample enough time to land in the basket
//
//            sleep(1000);
//
//            intake.setPosition(0.47); // opens intake claw
//            liftMotor.setTargetPosition(1690);
//            setLiftMotorBasket(); // lift liftmotor
//            basketServo.setPosition(1);
//            sleep(5000);

            if ((startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT) || (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHT) && preprogram == false) {
                telemetry.addData("Running >> ", startPosition.getClass());
//                telemetry.update();

                PredominantColorProcessor.Result result = colorSensor.getAnalysis();

                // Telemetry for testing
                // Display the Color Sensor result.
                telemetry.addData("Best Match:", result.closestSwatch);
                telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
                telemetry.update();
/*                the expected pose is the final position of the robot
//                expectedStartPoseX = -12.6;//26.8;
//                expectedStartPoseY = -62.7;
//                double DeltaX = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x - expectedStartPoseX;
//                double DeltaY = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y - expectedStartPoseY;
//                double driveDistance = -24 + DeltaY;
//                double strafeDistance = 26 + DeltaX;

                // Pseudocode for BlueLeft/RedLeft auto program:
                // 1. Close claw on specimen
                // 2. Drive up to the submersible
                // 3. Turn 180 degrees
                // 4. Place specimen
                // 5. Turn 180 degrees
                // 6. Strafe towards april tag 16
                // 7. Intake sample
                // 8. Turn right 45 degrees
                // 9. Drive backwards (if necessary) to line up against the basket
                // 10. Place sample in the basket
                // 11. Park in Level 1 Ascent (back of the robot is touching the low rung) */

                // PREREQUISITE PROGRAM FOR BOTH SIDES
                // ======================= DRIVE
                intake.setPosition(0.47); // open intake claw
                claw.setPosition(1); // close claw
//                liftMotor.setTargetPosition(1120); // lift liftmotor
                liftMotor.setTargetPosition(1020); // lift liftmotor
                setLiftMotorClaw();
                robot.driveWithTimeout(-15.75, 0.9, 0,1);
                robot.driveWithTimeout(-14.5, 0.3, 0.3, 1);
                // ======================= PLACE SPECIMEN
                liftMotor.setTargetPosition(0); // pull down liftmotor
                setLiftMotorDown();

                runonce = true;
                preprogram = true; // prerequisite program complete


                if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT && preprogram == true) {
                    robot.drive(7.5, 0.55, 0.25);
                    robot.resetHeading();
                    robot.turnToWithTimeout(180, 0.45, 0.1,3);
                    robot.resetHeading(); // reset heading for limelight

//                    robot.strafe(35, 0.7, 0);
//                    robot.strafeWithTimeout(7, 0.55, 0.1,2.5);
                    robot.strafe(26.5, 1, 0);
                    robot.strafeWithTimeout(6, 0.55, 0.1,2.5);
                    intakeSequence();
                    sleep(700); // lets the sample drop in the basket
                    intake.setPosition(0.47); // opens intake claw
                    sleep(300); // give the sample enough time to land in the basket

                    robot.strafeWithTimeout(15,0.4,0.2,4);
                    robot.strafeWithTimeout(-7,0.55,0.1,2.5);
                    robot.drive(-10,0.5,0.2);
                    robot.turnToWithTimeout(-40,0.5,0.2,2);

//                    robot.drive(-12,0.55,0.2);
                    intake.setPosition(0.43); // opens intake claw
                    sleep(250);
//                    intake.setPosition(0.25); // close intake
//                    sleep(250);
                    PivotServo.setPosition(0.5); // make sure intake claw is not in the way

//                    liftMotor.setTargetPosition(1690);
//                    liftMotor.setTargetPosition(1500);
//                    setLiftMotorBasket(); // lift liftmotor
//                    basketServo.setPosition(1); // dump basket
//                    sleep(1200); // this provides time for the basket to dump
//                    sleep(250); // this provides time for the basket to return

                    liftMotor.setTargetPosition(0);
//                    basketServo.setPosition(0); // return basket
                    setLiftMotorDown(); // return liftmotor
                    robot.turnToWithTimeout(0,0.6,0.2,2);

                    // ======================= PARK
                    robot.strafeWithTimeout(-19, 0.5, 0.2,2); // strafe right 24 inches
                    robot.driveWithTimeout(30, 0.8, 0,2); // drive forward 48 inches
                    robot.driveWithTimeout(10, 0.3, 0.2,2); // drive forward 48 inches
                    intake.setPosition(0.47); // make sure claw is out of the way
                    robot.turnToWithTimeout(90,0.5,0.2,2);
//                    PivotServo.setPosition(0.5);
//                    basketServo.setPosition(1); // dump basket servo
                    robot.drive(-12, 0.55, 0);
                    robot.driveWithTimeout(-6,0.3,0.5,2);

//                    // ======================= START LIMELIGHT
//                    /* Starts polling for data.  If you neglect to call start(), getLatestResult() will return null. */
//                    //                sleep(200);
//                    limelight.start();
//                    limelight.pipelineSwitch(0);
//                    telemetry.addData("limelight started", "");
                    runonce = true;
//                    prelimelight = true;
//                    sleep(1000);
                }

                if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHT && preprogram == true) {
//                    robot.drive(7.5, 0.55, 0.25);
                    robot.drive(6.5, 0.55, 0.25);
                    robot.resetHeading();
                    robot.turnToWithTimeout(180, 0.45, 0.1,3);
                    robot.resetHeading();

//                    robot.strafe(35, 0.7, 0);
//                    robot.strafeWithTimeout(7, 0.55, 0.1,2.5);
                    // STRAFE TO GRAB FIRST SAMPLE
//                    robot.strafe(-26.5, 1, 0);
                    robot.strafe(-29, 1, 0);
                    // SIMULTANEOUSLY EXTEND EXTENSION MOTOR
                    PivotServo.setPosition(0.5);
                    intake.setPosition(0.47); // opens intake claw
//                    extensionMotor.setTargetPosition(780); // extend extensionMotor
//                    extensionMotor.setTargetPosition(740); // extend extensionMotor
                    extensionMotor.setTargetPosition(780); // extend extensionMotor
                    setExtensionMotor();
                    // CONTINUE STRAFING TO FIRST SAMPLE
                    robot.strafeWithTimeout(-6, 0.55, 0.3,2.5);

                    // test code
                    PivotServo.setPosition(0);
                    intake.setPosition(0.47);
                    sleep(200);
                    intake.setPosition(0.25);
                    sleep(300);

                    PivotServo.setPosition(0.86);
                    robot.turnToWithTimeout(-140,0.45,0,2);
                    PivotServo.setPosition(0);
                    intake.setPosition(0.47);
//                    PivotServo.setPosition(0);
                    sleep(200);
                    robot.turnToWithTimeout(0,0.45,0.1,2);
                    PivotServo.setPosition(0.5);
                    extensionMotor.setTargetPosition(0);
                    setExtensionMotor();

//                    intakeSequence();
//                    sleep(700); // lets the sample drop in the basket
//                    intake.setPosition(0.47); // opens intake claw
//                    sleep(300); // give the sample enough time to land in the basket
//                    PivotServo.setPosition(0); // make sure intake claw is not in the way
                    claw.setPosition(0.7); // open claw

//                    robot.driveWithTimeout(15,0.5,0.25,2);    // grab second specimen

                    // CHUCK SAMPLE INTO OBSERVATION ZONE
//                    basketServo.setPosition(0.5);
                    // SIMULTANEOUSLY DRIVE INTO OBSERVATION ZONE
//                    robot.driveWithTimeout(-18,0.5,0.1,2);
                    robot.driveWithTimeout(-15,0.5,0.1,2);
                    // SIMULTANEOUSLY RESET LIFTMOTOR
//                    liftMotor.setTargetPosition(1000);
//                    setLiftMotorClaw();
//                    basketServo.setPosition(0); // return basket
//                    liftMotor.setTargetPosition(0);
//                    setLiftMotorDown();
                    robot.driveWithTimeout(-4,0.3,0.25,0.5);

                    // GRAB SPECIMEN
                    claw.setPosition(1); // close claw
                    liftMotor.setTargetPosition(1020); // lift liftmotor
                    setLiftMotorClaw();
                    robot.driveWithTimeout(10,1,0,2);
                    robot.turnTo(-15,0.5,0);
                    robot.strafeWithTimeout(36,1,0,10);
                    robot.turnTo(180,0.5,0);
                    robot.driveWithTimeout(-8,0.4,0,1);
                    liftMotor.setTargetPosition(0); // pull down liftmotor
                    setLiftMotorDown();

                    //~~~~~~~~END FIRST PLACEMENT
                    //~~~~~~~~END FIRST PLACEMENT
                    //~~~~~~~~END FIRST PLACEMENT
                    robot.driveWithTimeout(7,0.6,0,2);
                    claw.setPosition(0.7); // open claw
                    robot.turnTo(-110,0.5,0);
                    robot.drive(32,1,0);
                    robot.turnTo(0,0.5,0);
                    robot.strafe(-2,0.6,0.1);
                    robot.driveWithTimeout(-8,0.4,0.1,1);
                    //~~~~~~~~END THIRD SPECEMIN GRAB
                    //~~~~~~~~END THIRD SPECEMIN GRAB
                    //~~~~~~~~END THIRD SPECEMIN GRAB

                    claw.setPosition(1); // close claw
//                    sleep(150);
                    liftMotor.setTargetPosition(1020); // lift liftmotor
                    setLiftMotorClaw();
                    robot.driveWithTimeout(10,1,0,2);
                    robot.turnTo(-15,0.5,0);
                    intake.setPosition(0.47); // make sure intake claw is not in the way
                    robot.strafeWithTimeout(36,1,0,10);
                    robot.turnTo(180,0.5,0);
                    robot.driveWithTimeout(-8,0.4,0,1);
                    liftMotor.setTargetPosition(0); // pull down liftmotor
                    setLiftMotorDown();
                    sleep(300);
                    claw.setPosition(0.7); // open claw


                    // OLD CODE FOR HANGING THE SECOND AND THIRD SPECIMENS
                    /*

                    // GRAB SPECIMEN
                    claw.setPosition(1); // close claw
                    sleep(150);
                    liftMotor.setTargetPosition(1020); // lift liftmotor
                    setLiftMotorClaw();

                    robot.driveWithTimeout(15,0.5,0.25,2);
                    intake.setPosition(0.47); // make sure intake claw is not in the way
                    robot.turnToWithTimeout(180,0.55,0.25,2);
                    robot.strafe(-35, 0.9, 0);
                    robot.strafeWithTimeout(-10, 0.55, 0.1,2.5);
                    robot.driveWithTimeout(-15,0.4,0.1,2);
                    liftMotor.setTargetPosition(0); // pull down liftmotor
                    setLiftMotorDown();
//                    sleep(1200);
                    claw.setPosition(0.7); // open claw


                    // grab third specimen
                    robot.driveWithTimeout(15,0.4,0.1,2);
                    intake.setPosition(0.47); // make sure intake claw is not in the way
                    robot.strafe(35, 0.9, 0);
                    robot.strafeWithTimeout(10, 0.55, 0.1,2.5);
                    robot.turnToWithTimeout(0,0.55,0.2,2);

//                    robot.driveWithTimeout(-18,0.5,0.1,2);
                    robot.driveWithTimeout(-13,0.5,0.1,2);
                    robot.driveWithTimeout(-4,0.3,0.25,0.3);
                    claw.setPosition(1); // close claw
                    sleep(200);
                    liftMotor.setTargetPosition(1120); // lift liftmotor
                    setLiftMotorClaw();
                    robot.driveWithTimeout(15,0.5,0.25,2);
                    robot.turnToWithTimeout(180,0.55,0.25,2);
                    robot.strafe(-37, 0.9, 0);
                    robot.strafeWithTimeout(-10, 0.55, 0.1,2.5);
                    robot.driveWithTimeout(-15,0.4,0.2,3);
                    liftMotor.setTargetPosition(0); // pull down liftmotor
                    setLiftMotorDown();
                    claw.setPosition(0.7); // open claw */

                    // ======================= PARK (you won't have time to park, but do this just in case)
                    robot.driveWithTimeout(7,0.6,0,2);
                    claw.setPosition(0.7); // open claw
                    robot.turnTo(-110,0.5,0);
                    robot.drive(32,1,0);

//                    intake.setPosition(0.47); // make sure intake is out of the way
//                    robot.drive(18,0.7,0);
//                    robot.drive(5,0.3,0);
//                    robot.strafe(42,1,0);

                    // ============= Contingency plan !!
                    // Facing forwards
//                    robot.drive(14,0.5,0.5);
//                    robot.strafe(-45, 0.55, 0.25);
//                    sleep(500);
//                    runonce = true;
/*                                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                                telemetry.addData("RobotXPos", fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x);
//                                telemetry.addData("RobotYPos", fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y);
//                                telemetry.addData("Yaw", fr.getRobotPoseFieldSpace().getOrientation().getYaw(AngleUnit.DEGREES));
//                                telemetry.addData("Pitch", fr.getRobotPoseFieldSpace().getOrientation().getPitch(AngleUnit.DEGREES));
//                                telemetry.addData("Roll", fr.getRobotPoseFieldSpace().getOrientation().getRoll(AngleUnit.DEGREES));
//                                telemetry.addData("DeltaX", DeltaX);
//                                telemetry.addData("DeltaY", DeltaY);
//                                telemetry.addData("driveDistance", driveDistance);
//                                telemetry.addData("strafeDistance", strafeDistance);
//                                telemetry.addData("odometryX", ascentMotor1);
//                                telemetry.addData("odometryY", ascentMotor2); */
                    preprogram = false;
                }
            }
            if ((startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT_SAMPLE_ONLY) && preprogram == false) {
                telemetry.addData("Running >> ", startPosition.getClass());

                // MAKE SURE ROBOT STARTS FACING FORWARDS
                // strafe left and place in the basket
//                robot.strafe(35,0.8,0);
//                robot.strafeWithTimeout(15,0.4,0.2,2);
                robot.driveWithTimeout(6,0.5,0.2,2);

                intake.setPosition(0.47); // opens intake claw
                liftMotor.setTargetPosition(1690);
                setLiftMotorBasket(); // lift liftmotor

                robot.strafe(33,1,0);
                robot.strafeWithTimeout(12,0.4,0.2,2);
                robot.turnToWithTimeout(-40,0.45,0.1,2);

//                robot.strafeWithTimeout(3,0.4,0.2,2);

                // place first sample
//                intake.setPosition(0.47); // opens intake claw
//                liftMotor.setTargetPosition(1690);
//                setLiftMotorBasket(); // lift liftmotor
                basketServo.setPosition(1); // dump basket
                sleep(1000); // this provides time for the basket to dump
                basketServo.setPosition(0); // return basket

                liftMotor.setTargetPosition(0);
                basketServo.setPosition(0); // return basket
                setLiftMotorDown(); // return liftmotor
                robot.turnToWithTimeout(0,0.45,0.2,2);

//                robot.strafeWithTimeout(4,0.55,0.2,2);
                robot.strafeWithTimeout(5,0.55,0.2,2);
                PivotServo.setPosition(0.5);
                intake.setPosition(0.47); // opens intake claw
                extensionMotor.setTargetPosition(350); // extend extensionMotor
                setExtensionMotor();
//                robot.driveWithTimeout(10,0.55,0.1,2);
                robot.driveWithTimeout(12,0.55,0.1,2);
//                robot.strafeWithTimeout(-4,0.4,0.2,2);


                // place second sample
                sleep(500);
                intakeSequence();
                sleep(700); // lets the sample drop in the basket
                intake.setPosition(0.47); // opens intake claw
                sleep(300); // give the sample enough time to land in the basket

                robot.strafeWithTimeout(-5,0.55,0.2,2);
//                robot.strafeWithTimeout(-4.5,0.55,0.2,2);

                intake.setPosition(0.47); // opens intake claw
                liftMotor.setTargetPosition(1690);
                setLiftMotorBasket(); // lift liftmotor

//                robot.drive(-10,0.5,0.2);
                robot.drive(-11.5,0.5,0.2);
//                robot.turnToWithTimeout(-40,0.45,0.2,2.5);
                robot.turnToWithTimeout(-30,0.45,0.2,2.5);

//                intake.setPosition(0.47); // opens intake claw
//                liftMotor.setTargetPosition(1690);
//                setLiftMotorBasket(); // lift liftmotor
                basketServo.setPosition(1); // dump basket
                sleep(1000); // this provides time for the basket to dump
                basketServo.setPosition(0); // return basket

                liftMotor.setTargetPosition(0);
                setLiftMotorDown(); // return liftmotor
                robot.turnToWithTimeout(0,0.45,0.1,2.5);


                // place third sample
//                robot.driveWithTimeout(10,0.55,0.3,2);
                robot.driveWithTimeout(11,0.55,0.15,2);
                PivotServo.setPosition(0.5);
                intake.setPosition(0.47); // opens intake claw
//                    extensionMotor.setTargetPosition(780); // extend extensionMotor
                extensionMotor.setTargetPosition(460); // extend extensionMotor
                setExtensionMotor();
                robot.strafeWithTimeout(-8.7,0.5,0.2,2);

                intakeSequence();
                sleep(700); // lets the sample drop in the basket
                intake.setPosition(0.47); // opens intake claw
                sleep(300); // give the sample enough time to land in the basket

//                robot.driveWithTimeout(-10,0.55,0.1,2);
                robot.driveWithTimeout(-12.5,0.55,0.1,2);

                intake.setPosition(0.47); // opens intake claw
                liftMotor.setTargetPosition(1690);
                setLiftMotorBasket(); // lift liftmotor

                robot.strafeWithTimeout(10,0.55,0.2,2);
//                robot.turnToWithTimeout(-40,0.45,0.1,2);
                robot.turnToWithTimeout(-30,0.45,0.1,2);

//                intake.setPosition(0.47); // opens intake claw
//                liftMotor.setTargetPosition(1690);
//                setLiftMotorBasket(); // lift liftmotor
                basketServo.setPosition(1); // dump basket
                sleep(1000); // this provides time for the basket to dump
                basketServo.setPosition(0); // return basket

                liftMotor.setTargetPosition(0);
                setLiftMotorDown(); // return liftmotor
//                robot.turnToWithTimeout(0,0.45,0.1,2);

                // place fourth sample

                // ======================= PARK
                robot.turnToWithTimeout(65,0.45,0,2);

//                robot.strafeWithTimeout(-18, 1, 0,2); // strafe right 24 inches
//                robot.driveWithTimeout(-18, 1, 0,2); // strafe right 24 inches
//                robot.driveWithTimeout(30, 1, 0,2); // drive forward 48 inches

//                robot.strafeWithTimeout(-37, 1, 0,2); // drive forward 48 inches
                robot.strafeWithTimeout(-39, 1, 0,2); // drive forward 48 inches

//                robot.driveWithTimeout(10, 0.3, 0.2,2); // drive forward 48 inches
                robot.turnToWithTimeout(90,0.45,0,2);
                basketServo.setPosition(1); // dump basket servo
                robot.driveWithTimeout(-12, 1, 0,2); // drive forward 48 inches
//                intake.setPosition(0.47); // make sure claw is out of the way
//                robot.turnToWithTimeout(90,0.5,0.2,2);
//                PivotServo.setPosition(0.5);
                PivotServo.setPosition(0);
//                robot.drive(-12, 0.55, 0);
                robot.driveWithTimeout(-6,0.3,0.5,2);
                runonce = true;
                preprogram = true; // prerequisite program complete
            }
//            if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHT && prelimelight == false) {
//                telemetry.addData("Running RED_BLUE_RIGHT ", "");
//                telemetry.update();
//                telemetry.update();
//                // the expected pose is the final position of the robot
//                // for this specific test, the final position = (starting position + 24 in)
//                expectedStartPoseX = -12.6;//26.8;
//                expectedStartPoseY = -62.7;
////                double DeltaX = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x - expectedStartPoseX;
////                double DeltaY = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y - expectedStartPoseY;
////                double driveDistance = -24 + DeltaY;
////                double strafeDistance = -36 + DeltaX;
//
//                // Pseudocode for BlueRight/RedRight auto program:
//                // [NOTE you start facing backwards]
//                // 1. Close claw on specimen
//                // 2. Drive up to the submersible
//                // 3. Place specimen
//                // 4. Strafe towards april tag 14
//                // 5. Turn 180 degrees
//                // 6. Pick up sample
//                // 7. Place sample(s) in observation zone
//                // 8. Drive backwards to park in the observation zone
//
////                robot.drive(-24, 0.4, 0.25); // drive BACKWARDS 24 inches
////                liftMotor.setTargetPosition(572); // Lift claw/basket slide
////                robot.drive(4, 0.4, 0.25);// Move forward
////                liftMotor.setTargetPosition(276); // Pull down
////                claw.setPosition(0); // Open claw
////                robot.drive(4, 0.4, 0.25);// Move forward
////                liftMotor_Claw_Reset();
////                robot.strafe(-36, 0.45, 0.5); // strafe right towards tag 14
////                robot.turnTo(180, 0.4, 0.25); // turn 180 degrees
////                robot.drive(-22, 0.4, 0.25); // drive backwards 22 inches
////                PivotServo.setPower(0.8); // drop pivot servo
////                sleep(200); // run pivot servo for __ seconds
////                PivotServo.setPower(0); // stop pivot servo if necessary
////                intake(); // intake MIDDLE sample
////                PivotServo.setPower(-0.8); // return pivot servo
////                sleep(200); // run pivot servo for __ seconds
////                PivotServo.setPower(0); // stop pivot servo if necessary
////                stopIntakeOROuttake(); // Stop intake
////                robot.drive(-4, 0.4, 0.25); // Drive backward
////                liftMotor.setTargetPosition(276); // Lift claw/basket slide to set position
////                basket_place_sequence(); // Flip basket servo
////                liftMotor_basketServo_Reset();
////                // NOTE: You may have to repeat the previous steps to intake and deposit all of the samples
////                robot.drive(-20, 0.4, 0.25); // Drive backwards to park
//
////                robot.strafe(-50,0.5,0.25); // strafe left and park
////                claw.setPosition(1); // close claw on specimen
////                robot.drive(-50,0.5,0.5);
////                sleep(500);
////                runonce = true;
///*                telemetry.addData("odometryX", odometryX.getCurrentPosition());
////                telemetry.addData("odometryY", odometryY.getCurrentPosition());
////                telemetry.addData("odometryY", robot.driveEncoder);
//                  telemetry.addData("odometryX", robot.strafeEncoder);
// */
//            }
//            LLStatus status = limelight.getStatus();
//            telemetry.addData("Name", "%s",
//                    status.getName());
//            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                    status.getTemp(), status.getCpu(),(int)status.getFps());
//            telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose_MT2();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
//                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                if (result.isValid()) {
//                    telemetry.addData("tx", result.getTx());
//                    telemetry.addData("txnc", result.getTxNC());
//                    telemetry.addData("ty", result.getTy());
//                    telemetry.addData("tync", result.getTyNC());
                    telemetry.addData("Botpose (robot's orientation/location):", botpose.toString());

                    // Access barcode results
                    List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                    for (LLResultTypes.BarcodeResult br : barcodeResults) {
                        telemetry.addData("Barcode", "Data: %s", br.getData());
                    }

                    // Access classifier results
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }

                    // Access detector results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }

                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                        for (LLResultTypes.FiducialResult fr : fiducialResults) {
                            if (startPosition == SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT && prelimelight == true) {
/*                                // the expected pose is the final position of the robot
                                // for this specific test, the final position = (starting position + 24 in)
//                                expectedStartPoseX = -12.6;//26.8;
//                                expectedStartPoseY = -62.7;
//                                double DeltaX = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x - expectedStartPoseX;
//                                double DeltaY = fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y - expectedStartPoseY;
//                                double driveDistance = -24 + DeltaY;
//                                double strafeDistance = 34 + DeltaX;

//                                telemetry.addData("preset_expectedStartPoseX -->", expectedStartPoseX);
//                                telemetry.addData("preset_expectedStartPoseY -->", expectedStartPoseY);
//                                telemetry.addData("DeltaX -->", DeltaX);
//                                telemetry.addData("DeltaY -->", DeltaY);
//                                telemetry.addData("driveDistance -->", driveDistance);
//                                telemetry.addData("strafeDistance -->", strafeDistance);
//                                telemetry.addData("Limelight_RobotPoseX -->", fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x);
//                                telemetry.addData("Limelight_RobotPoseY -->", fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y);
//                                telemetry.update();
//                                sleep(10000); */
                                // ========================================== STRAFE TO APRILTAG 16
                                // NOTE: You may have to check the startpose to see if this strafing will work
                            }
/*                                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                                telemetry.addData("RobotXPos", fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).x);
//                                telemetry.addData("RobotYPos", fr.getRobotPoseFieldSpace().getPosition().toUnit(DistanceUnit.INCH).y);
//                                telemetry.addData("Yaw", fr.getRobotPoseFieldSpace().getOrientation().getYaw(AngleUnit.DEGREES));
//                                telemetry.addData("Pitch", fr.getRobotPoseFieldSpace().getOrientation().getPitch(AngleUnit.DEGREES));
//                                telemetry.addData("Roll", fr.getRobotPoseFieldSpace().getOrientation().getRoll(AngleUnit.DEGREES));
//                                telemetry.addData("DeltaX", DeltaX);
//                                telemetry.addData("DeltaY", DeltaY);
//                                telemetry.addData("driveDistance", driveDistance);
//                                telemetry.addData("strafeDistance", strafeDistance);
//                                telemetry.addData("odometryX", ascentMotor1);
//                                telemetry.addData("odometryY", ascentMotor2); */

                            // INSERT RIGHT SIDE CODE HERE
                        }

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    }

//                    List<ColorResult> colorTargets = result.getColorResults();
//                    for (ColorResult colorTarget : colorTargets) {
//                        double x = detection.getTargetXDegrees(); // Where it is (left-right)
//                        double y = detection.getTargetYDegrees(); // Where it is (up-down)
//                        double area = colorTarget.getTargetArea(); // size (0-100)
//                        telemetry.addData("Color Target", "takes up " + area + "% of the image");
//                    }
                    /* Pseudocode for robot repositioning based off of apriltag:

                    deltax = pose.x - expectedPose.x;
                    deltay = pose.y - expectedPose.y;

                     // drive forward 24 inches, but the robot is setup 1 inch off the wall.
                     driveForward(24 - deltay);
                     */
                limelight.stop();
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }
        }
    }

    public void selectStartingPosition () {
        boolean selected = false;
        while (!isStopRequested() || (selected)) {
            telemetry.addLine("State Tournament - Code Initialized");
            telemetry.addData("---------------------------------------", "");
            telemetry.addLine("Select Starting Position using DPAD Keys");
            telemetry.addData("    Red/Blue Left Sample ONLY ", "(^)");
//            telemetry.addData("    Square Testing ", "(v)");
//            telemetry.addData("    Red/Blue Left    ", "(<)");
            telemetry.addData("    Red/Blue Right  ", "(>)");

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                startPosition = SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT_SAMPLE_ONLY;
                telemetry.addData("Red/Blue Left Sample ONLY: ", "selected");
                telemetry.update();
                selected = true;
                break;
            }
//            if (gamepad1.dpad_left || gamepad2.dpad_down) {
//                startPosition = SampleAuto_IntoTheDeep.START_POSITION.SQUARE_TESTING;
//                selected = true;
//                break;
//            }
//            if (gamepad1.dpad_left || gamepad2.dpad_left) {
//                startPosition = SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_LEFT;
//                telemetry.addData("Red/Blue Left: ", "selected");
//                telemetry.update();
//                selected = true;
//                break;
//            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                startPosition = SampleAuto_IntoTheDeep.START_POSITION.RED_BLUE_RIGHT;
                selected = true;
                telemetry.addData("Red/Blue Right: ", "selected");
                telemetry.update();
                break;
            }
            telemetry.update();
        }
    }
    public void setLiftMotorClaw () {
        double liftMotorPower = 0;
        PivotServo.setPosition(0.5); // make sure intake claw is not in the way
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorPower = 1;
        liftMotor.setPower(liftMotorPower);
    }
     public void setExtensionMotor () {
         double ExtensionMotorPower = 0;
         extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         ExtensionMotorPower = 1;
         extensionMotor.setPower(ExtensionMotorPower);
     }
    public void setLiftMotorBasket () {
        double liftMotorPower = 0;
        PivotServo.setPosition(0.5); // make sure intake claw is not in the way
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorPower = 1;
        liftMotor.setPower(liftMotorPower);
//        sleep(2000);
    }
    public void setLiftMotorDown () {
        double liftMotorPower = 0;
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorPower = 1;
        liftMotor.setPower(liftMotorPower);
//        while (liftMotor.getCurrentPosition() > 500){
//        }
//        liftMotor.setPower(.05);
//        sleep(500);
    }
    public void liftMotor_Claw_Reset () {
        claw.setPosition(0); // leave claw open
        liftMotor.setTargetPosition(0); // bring slide down
    }
    public void liftMotor_basketServo_Reset () {
        basketServo.setPosition(0); // return basket
        liftMotor.setTargetPosition(0); // bring slide down
    }
    public void basketDumpSequence () {
        basketServo.setPosition(1); // dump basket
        sleep(500); // this provides time for the basket to dump
        basketServo.setPosition(0); // return basket
        sleep(500); // this provides time for the basket to return
    }
    public void intakeSequence () {
        // ExtensionMotor out

//        PivotServo.setPosition(0.5);
//        intake.setPosition(0.47); // opens intake claw
//
//        extensionMotor.setTargetPosition(780); // extend extensionMotor
//        setExtensionMotor();
//        sleep(400);
//        sleep(500);

        // PivotServo drop
//        PivotServo.setPosition(0.5);
//        intake.setPosition(0.25); // closes intake claw
//        sleep(250);
//        intake.setPosition(0.43); // opens intake claw
//        sleep(300);
        PivotServo.setPosition(0);
        intake.setPosition(0.47);
        sleep(250);
//        sleep(400);
        // Drop Pivot Servo

        // PivotServo return and transfer to basket
//        intake.setPosition(0.25); // closes intake claw
        intake.setPosition(0.2); // closes intake claw
        sleep(300);
//        sleep(400);
        extensionMotor.setTargetPosition(0);
        setExtensionMotor();
        PivotServo.setPosition(.86); //raises claw to point to robot
//        sleep(600);
        // Return Pivot Servo
//        intake.setPosition(0.47); // opens intake claw
//        sleep(500);//1000);
//        intake.setPosition(0.25); // close intake
//        sleep(250);
//        PivotServo.setPosition(0.5); // make sure intake claw is not in the way
    }
    public void outtake () {
        intake.setPosition(1);
    }
}