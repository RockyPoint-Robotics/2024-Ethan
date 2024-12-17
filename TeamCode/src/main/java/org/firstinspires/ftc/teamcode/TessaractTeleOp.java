package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    double rotAngleThreshold = 5; // Angle that the robot snaps to rotation wise
    double posAngleThreshold = 5; // Angle that the robot snaps to movement wise
    double rotAngleSnapping = 45;
    double posAngleSnapping = 45;
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

        fLMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        fRMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bRMotor.setDirection(DcMotorSimple.Direction.REVERSE);


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
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double magnitude = 0;
        if (LJoyVector.getX() + LJoyVector.getY() != 0) {
            magnitude = LJoyVector.getNorm();
        }

        // Conversions from Degrees to Radians

        rotAngleThreshold = rotAngleThreshold * Math.PI / 180;
        posAngleThreshold = posAngleThreshold * Math.PI / 180;
        rotAngleSnapping = rotAngleSnapping * Math.PI / 180;
        posAngleSnapping = posAngleSnapping * Math.PI / 180;

        if (gamepad1.right_bumper) {
            // Grid Movement Snapping Mode

            if (posAngleThreshold < angle || angle < (posAngleSnapping - posAngleThreshold)) {
                // angle =
            }
        }

        LJoyVector = new Vector2D(Math.sin(angle-yaw) * magnitude, Math.cos(angle-yaw) * magnitude);

        RJoyVector = new Vector2D(gamepad1.right_stick_x, gamepad1.right_stick_y);

        bLMotor.setPower(-LJoyVector.getY() - LJoyVector.getX() + RJoyVector.getX());
        fRMotor.setPower(-LJoyVector.getY() - LJoyVector.getX() - RJoyVector.getX());
        bRMotor.setPower(-LJoyVector.getY() + LJoyVector.getX() - RJoyVector.getX());
        fLMotor.setPower(-LJoyVector.getY() + LJoyVector.getX() + RJoyVector.getX());

        // Telemetry

        telemetry.addData("Adjusted Angle", angle-yaw);
        telemetry.addData("Angle", angle);
        telemetry.addData("Magnitude", magnitude);

        telemetry.addData("Yaw", yaw);
        telemetry.addData("Pitch", pitch);
        telemetry.addData("Roll", roll);

    }
}
