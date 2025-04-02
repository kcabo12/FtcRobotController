package Opmodes;

import static com.qualcomm.robotcore.util.Range.clip;

import android.icu.lang.UCharacter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.source.tree.BreakTree;

//import org.firstinspires.ftc.teamcode.TemplateJanx;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.FileWriter;
import java.io.IOException;
import java.text.BreakIterator;

@TeleOp(name="RecordTeleop", group="Testing")
public class RecordTeleOp extends OpMode {
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

    private JSONArray movementLog;

    // Arm positions
    private final int liftMotorUpPosition = 1500;
    private final int liftMotorRestPosition = 0;
    private boolean liftFlag = false;
    private boolean lastAState = false;

    @Override
    public void init() {
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

        movementLog = new JSONArray();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        controlClaw();
        controlLift();
        controlBasket();

        recordMovement();

        telemetry.addData("LiftMotor Position", liftMotor.getCurrentPosition());
        telemetry.addData("Claw Position", claw.getPosition());
        telemetry.update();
    }

//    @Override
//    public void stop() {
//        try (FileWriter file = new FileWriter("/sdcard/FIRST/movementLog.json")) {
//            file.write(movementLog.toString());
//            telemetry.addData("Status", "Movements saved to /sdcard/FIRST/movementLog.json");
//        } catch (IOException e) {
//            telemetry.addData("Error", "Failed to save movements: " + e.getMessage());
//        }
//        telemetry.update();
//    }

    @Override
    public void stop() {
        try {
            String filePath ="/data/user/0/com.qualcomm.ftcrobotcontroller/files/movementLog.json";
            FileWriter file = new FileWriter(filePath);
            file.write(movementLog.toString());
            file.close();
            telemetry.addData("Status", "Movements saved to " + filePath);
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to save movements: " + e.getMessage());
        }
        telemetry.update();
    }


    private void mecanumDrive(double LSY, double LSX, double RSX) {
        int speed = 1600; // Motor speed
        double lx = Math.pow(LSX, 3);
        double ly = -Math.pow(LSY, 3);
        double rx = Math.pow(RSX, 3);

        if (LSX != 0 || LSY != 0 || RSX != 0) {
            rightFront.setPower(speed * (clip((ly) - lx, -1, 1) - rx));
            leftFront.setPower(speed * (clip((ly) + lx, -1, 1) + rx));
            rightBack.setPower(speed * (clip((ly) + lx, -1, 1) - rx));
            leftBack.setPower(speed * (clip((ly) - lx, -1, 1) + rx));
        } else {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
        }
    }

    private void controlClaw() {
        if (gamepad2.left_bumper == true) {
            claw.setPosition(0.5);
        } else if (gamepad2.right_bumper == true) {
            claw.setPosition(1);
        }
    }

    private void controlBasket() {
        if (gamepad2.a) {
            basketServo.setPosition(0);
        } else if (gamepad2.y) {
            basketServo.setPosition(1);
        }
    }

//    private void controlintake() {
//        if (gamepad1.right_trigger > 0) {
//            intake.setPower(1);
//        } else if (gamepad1.left_trigger > 0) {
//            intake.setPower(-1);
//        }
//    }

    private void controlLift() {
        if (gamepad2.a && !lastAState) {
            liftFlag = !liftFlag;
        }
        lastAState = gamepad2.a;

        int targetPosition = liftFlag ? liftMotorUpPosition : liftMotorRestPosition;
        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.5);
    }

    private void recordMovement() {
        try {
            JSONObject movement = new JSONObject();
            movement.put("leftFrontVelocity", leftFront.getPower());
            movement.put("RightFrontVelocity", rightFront.getPower());
            movement.put("LeftBackVelocity", leftBack.getPower());
            movement.put("RightBackVelocity", rightBack.getPower());
//            movement.put("LiftMotorPosition", liftMotor.getCurrentPosition());
//            movement.put("extensionMotorPosition", extensionMotor.getCurrentPosition());
            movement.put("clawPosition", claw.getPosition());
            movement.put("basketPosition", basketServo.getPosition());
            movementLog.put(movement);
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to record movement");
        }
    }
}
