package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Driving Test")
public class DrivingTesting extends OpMode {

    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;
    public DcMotor armMotor;
    double craneSpeedMax = 1.0;
    double craneSpeedMin = 0.0;

    @Override
    public void init() {
        fLMotor = hardwareMap.get(DcMotor.class, "FL");
        fRMotor = hardwareMap.get(DcMotor.class, "FR");
        bLMotor = hardwareMap.get(DcMotor.class, "BL");
        bRMotor = hardwareMap.get(DcMotor.class, "BR");
        armMotor = hardwareMap.get(DcMotor.class, "ARM");

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

        if (gamepad1.a) {
            fLMotor.setPower(1);
        } else {
            fLMotor.setPower(0);
        }
        if (gamepad1.b) {
            bLMotor.setPower(1);
        } else {
            bLMotor.setPower(0);
        }
        if (gamepad1.x) {
            fRMotor.setPower(1);
        } else {
            fRMotor.setPower(0);
        }
        if (gamepad1.y) {
            bRMotor.setPower(1);
        } else {
            bRMotor.setPower(0);
        }

        fLMotor.setPower(-y + x - rx);
        bLMotor.setPower(-y - x + rx);
        fRMotor.setPower(-y - x - rx);
        bRMotor.setPower(-y + x + rx);

        armMotor.setPower(gamepad1.right_trigger);
    }
}
