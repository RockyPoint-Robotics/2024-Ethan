package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.IMU;

import java.security.KeyPairGenerator;
import java.sql.Array;

@TeleOp(name="Tessaract")
public class TessaractTeleOp extends OpMode {

    // Motors
    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;
    public DcMotor armMotor;

    // IMU
    public IMU imu;
    public IMU.Parameters imuParams;
    public YawPitchRollAngles robotOrientation;

    // Configurations
    double armSpeed = 2;
    double angleThreshold = 5; // Angle that the robot snaps to
    double angleSnapping = 45;
    double basketHeight = 0; // Unknown, change later

    // Functions
    public double lerp(double start, double target, double alpha) {
        double output = start + (target - start) * alpha;
        return output;
    }

    // Controller Variables
    Vector2D LJoyVector = Vector2D.ZERO; // X will be the angle, Y will be the magnitude
    Vector2D RJoyVector = Vector2D.ZERO;

    @Override
    public void init() {
        fLMotor = hardwareMap.get(DcMotor.class, "FL");
        fRMotor = hardwareMap.get(DcMotor.class, "FR");
        bLMotor = hardwareMap.get(DcMotor.class, "BL");
        bRMotor = hardwareMap.get(DcMotor.class, "BR");
        armMotor = hardwareMap.get(DcMotor.class, "ARM");

        imu = hardwareMap.get(IMU.class, "imu");
        imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        // Make sure to change this when moving the control hub
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
    }

    @Override
    public void loop() {
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        double pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double roll = robotOrientation.getRoll(AngleUnit.DEGREES);

        imu.initialize(imuParams);
        imu.resetYaw();

        LJoyVector = new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) * (180/Math.PI);
        double magnitude = LJoyVector.getNorm();
        LJoyVector = new Vector2D(angle-yaw, magnitude);
        LJoyVector = new Vector2D(Math.sin(LJoyVector.getX()), Math.cos(LJoyVector.getX()));

        RJoyVector = new Vector2D(gamepad1.right_stick_x, gamepad1.right_stick_y);

        bLMotor.setPower(-LJoyVector.getY() - LJoyVector.getX() + RJoyVector.getX());
        fRMotor.setPower(-LJoyVector.getY() - LJoyVector.getX() - RJoyVector.getX());
        bRMotor.setPower(-LJoyVector.getY() + LJoyVector.getX() - RJoyVector.getX());
        fLMotor.setPower(-LJoyVector.getY() + LJoyVector.getX() + RJoyVector.getX());
    }
}
