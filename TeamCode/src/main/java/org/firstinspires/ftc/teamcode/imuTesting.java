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

@TeleOp(name="imuTesting")
public class imuTesting extends OpMode {

    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;
    public DcMotor armMotor;
    double yaw;
    public IMU imu;
    public IMU.Parameters imuParams;
    public YawPitchRollAngles robotOrientation;
    public Servo claw;
    public Vector2D lJoyPos = Vector2D.ZERO;
    double armSpeed = 2;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();
    }

    @Override
    public void loop() {
        double yaw = robotOrientation.getYaw(AngleUnit.RADIANS);

        telemetry.addData("Angle: ", yaw);
    }
}
