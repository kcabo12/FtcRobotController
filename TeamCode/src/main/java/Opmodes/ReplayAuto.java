package Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.TemplateJanx;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.FileReader;

@Autonomous(name="ReplayAuto", group="Testing")
public class ReplayAuto extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {
    // ======================= MOTORS =======================
    public DcMotor leftFront   = null;
    public DcMotor leftBack   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightBack  = null;
    public DcMotor liftMotor = null;
    public DcMotor extensionMotor = null;
    public DcMotor ascentMotor1 = null;
    public DcMotor ascentMotor2 = null;

    // ======================= SERVOS =======================
    public Servo claw = null;
    public CRServo intake = null;
    public CRServo PivotServo = null;
    public Servo basketServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
//        TemplateJanx janx = new TemplateJanx(hardwareMap);
//        janx.wheelInit("rightFront", "rightBack", "leftBack", "leftFront");
//        leftFront = janx.fl;
//        rightFront = janx.fr;
//        rightBack = janx.br;
//        leftBack = janx.bl;

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        extensionMotor = hardwareMap.get(DcMotor.class, "extensionMotor");
        ascentMotor1 = hardwareMap.get(DcMotor.class, "ascentMotor1");
        ascentMotor2 = hardwareMap.get(DcMotor.class, "ascentMotor2");

        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        PivotServo = hardwareMap.get(CRServo.class, "PivotServo");
        basketServo = hardwareMap.get(Servo.class, "basketServo");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Only use these if necessary for configuration:
        ascentMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascentMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ascentMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        JSONArray movementLog = loadMovementLog();
        if (movementLog == null) {
            telemetry.addData("Error", "Failed to load movement log.");
            telemetry.update();
            return;
        }

        telemetry.addData("Status", "Replaying movements...");
        telemetry.update();

        int i = 0;
        for (;i < movementLog.length() && opModeIsActive(); i++) {
            try {
                JSONObject movement = movementLog.getJSONObject(i);

                leftFront.setPower(movement.getDouble("leftFrontVelocity"));
                rightFront.setPower(movement.getDouble("RightFrontVelocity"));
                leftBack.setPower(movement.getDouble("LeftBackVelocity"));
                rightBack.setPower(movement.getDouble("RightBackVelocity"));
//                liftMotor.setTargetPosition(movement.getInt("LiftMotorPosition"));
//                liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                liftMotor.setPower(0.5);
//                extensionMotor.setPower(movement.getDouble("extensionMotorPosition"));
//                extensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                extensionMotor.setPower(0.5);
                claw.setPosition(movement.getDouble("clawPosition"));
                basketServo.setPosition(movement.getDouble("basketPosition"));

                Thread.sleep(9); // Adjust delay as needed
            }
            catch (Exception e) {
                telemetry.addData("Error", "Failed to replay movement at index " + i);
                telemetry.update();
            }
        }

        telemetry.addData("movementLog.length", movementLog.length());
        telemetry.addData("i value", i);
        telemetry.update();
        sleep(3000);

        stopMotors();
        telemetry.addData("Status", "Replay complete!");
        telemetry.update();
    }

    private JSONArray loadMovementLog() {
        String filePath = "/data/user/0/com.qualcomm.ftcrobotcontroller/files/movementLog.json";

        try (FileReader file = new FileReader(filePath)) {
            StringBuilder jsonBuilder = new StringBuilder();
            int character;
            while ((character = file.read()) != -1) {
                jsonBuilder.append((char) character);
            }
            return new JSONArray(jsonBuilder.toString());
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to load JSON: " + e.getMessage());
            telemetry.update();
            sleep(1000);
            return null;
        }
    }

    private void stopMotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        liftMotor.setPower(0);
    }
}
