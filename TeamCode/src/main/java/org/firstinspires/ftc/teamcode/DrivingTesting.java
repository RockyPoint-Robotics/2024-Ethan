package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Driving Test")
public class DrivingTesting extends OpMode {

    public boolean isClimbing = false;
    public double servoPosition = 0.0;
    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;
    public DcMotor armMotor;
    public CRServo claw;
    public DcMotor linearMotor;
    public DcMotor tensionMotor;
    double craneSpeedMax = 1.0;
    double craneSpeedMin = 0.0;

    @Override
    public void init() {
        fLMotor = hardwareMap.get(DcMotor.class, "FL");
        fRMotor = hardwareMap.get(DcMotor.class, "FR");
        bLMotor = hardwareMap.get(DcMotor.class, "BL");
        bRMotor = hardwareMap.get(DcMotor.class, "BR");
        armMotor = hardwareMap.get(DcMotor.class, "ARM");
        claw = hardwareMap.get(CRServo.class, "CLAW");
        linearMotor = hardwareMap.get(DcMotor.class, "LINEAR");
        tensionMotor = hardwareMap.get(DcMotor.class, "TENSION");

        fLMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        fRMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double y = gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Calculates movement power

        if (gamepad1.right_bumper) {
            claw.setPower(-.25);
        }
        if (gamepad1.left_bumper) {
            claw.setPower(-1.5);
        }
        if (gamepad1.x) {
            isClimbing = !isClimbing;
            armMotor.setPower(-1);
        }

        tensionMotor.setPower(0);
        linearMotor.setPower(0);

        if (gamepad1.dpad_up) {
            linearMotor.setPower(1);
        }

        if (gamepad1.dpad_down) {
            linearMotor.setPower(-1);
        }

        if (gamepad1.dpad_right) {
            tensionMotor.setPower(1);
        }

        if (gamepad1.dpad_left) {
            tensionMotor.setPower(-1);
        }

        fLMotor.setPower(-y - x - rx);
        bLMotor.setPower(-y + x + rx);
        fRMotor.setPower(-y + x - rx);
        bRMotor.setPower(-y - x + rx);

        if (!isClimbing) {
            armMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        }

        telemetry.addData("Servo Position: ", claw.getPower());
    }
}
