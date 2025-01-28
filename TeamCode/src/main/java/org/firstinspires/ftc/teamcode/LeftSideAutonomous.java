package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="LeftSideAutonomous")
public class LeftSideAutonomous extends LinearOpMode {
    public void xyToMotorPower(double x, double y, double rx) {
        fLMotor.setPower(-y + x + rx);
        bLMotor.setPower(-y - x + rx);
        fRMotor.setPower(-y - x - rx);
        bRMotor.setPower(-y + x - rx);
    }

    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;
    public DcMotor armMotor;
    // public Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        fLMotor = hardwareMap.get(DcMotor.class, "FL");
        fRMotor = hardwareMap.get(DcMotor.class, "FR");
        bLMotor = hardwareMap.get(DcMotor.class, "BL");
        bRMotor = hardwareMap.get(DcMotor.class, "BR");
        armMotor = hardwareMap.get(DcMotor.class, "ARM");
        // claw = hardwareMap.get(Servo.class, "CLAW");

        fLMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        fRMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Moves Left to parking space

        xyToMotorPower(-0.5, 0, 0);
        Thread.sleep(2500);
        xyToMotorPower(0, 0, 0);
    }
}
