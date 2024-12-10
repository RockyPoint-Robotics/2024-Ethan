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
        // armMotor = hardwareMap.get(DcMotor.class, "ARM");

        fRMotor.setDirection(DcMotor.Direction.REVERSE);
        bRMotor.setDirection(DcMotor.Direction.FORWARD);
        fLMotor.setDirection(DcMotor.Direction.FORWARD);
        bLMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double y = gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Calculates movement power

        fLMotor.setPower(gamepad1.right_trigger);
        bLMotor.setPower(gamepad1.right_trigger);
        fRMotor.setPower(gamepad1.right_trigger);
        bRMotor.setPower(gamepad1.right_trigger);

        fLMotor.setPower(gamepad1.left_stick_x);
        bLMotor.setPower(gamepad1.left_stick_y);
        fRMotor.setPower(gamepad1.right_stick_x);
        bRMotor.setPower(gamepad1.right_stick_y);

        // armMotor.setPower(gamepad1.right_trigger);
    }
}
