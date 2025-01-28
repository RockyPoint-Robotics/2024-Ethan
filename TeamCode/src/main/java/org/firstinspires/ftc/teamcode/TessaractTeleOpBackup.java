package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Backup")
public class TessaractTeleOpBackup extends OpMode {
    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;
    public DcMotor armMotor;
    public IMU imu;
    public IMU.Parameters imuParams;
    public YawPitchRollAngles robotOrientation;
    public Servo claw;
    public Vector2D lJoyPos = Vector2D.ZERO;
    double armSpeed = 2;
    double angleThreshold = 5; // Angle that the robot snaps to

    @Override
    public void init() {
        fLMotor = hardwareMap.get(DcMotor.class, "FL");
        fRMotor = hardwareMap.get(DcMotor.class, "FR");
        bLMotor = hardwareMap.get(DcMotor.class, "BL");
        bRMotor = hardwareMap.get(DcMotor.class, "BR");
        armMotor = hardwareMap.get(DcMotor.class, "ARM");
        // claw = hardwareMap.get(Servo.class, "CLAW");
    }

    @Override
    public void loop() {
        Vector2D lJoyPos = new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double y = gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double lJoyAngle = Math.atan2(y, x);
        double lJoyDistance = Math.sqrt(x * x + y * y);
        double rx = gamepad1.right_stick_x;
        double yaw; // YAW of robot
        yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        double calibratedYaw = lJoyAngle - yaw;
//        double anglePosX = Math.sin(LJoyAngle - yaw) * LJoyMagnitude;
//        double anglePosY = Math.cos(LJoyAngle - yaw) * LJoyMagnitude;

        double numbers[] = {0, 90, 180, 270};

        for(int i = 0; i == numbers.length - 1; i++){
            if(yaw >= numbers[i] - angleThreshold && yaw <= numbers[i] + angleThreshold && rx != 0){
                yaw = numbers[i];
            }
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        // Calculates movement power

        fLMotor.setPower(-y + x + rx);
        bLMotor.setPower(-y - x + rx);
        fRMotor.setPower(-y - x - rx);
        bRMotor.setPower(-y + x - rx);

        // Arm speed on dPad press

        if (gamepad1.dpad_up){
            armMotor.setPower(1 / armSpeed);
        } else if (gamepad1.dpad_down){
            armMotor.setPower(-1 / armSpeed);
        } else {
            armMotor.setPower(0);
        }

        // Claw position on button press

        if (gamepad1.a) {
            claw.setPosition(1);
        }
        if (gamepad1.b) {
            claw.setPosition(0);
        }

        if (gamepad1.y) {
            fLMotor.setPower(1);
            bLMotor.setPower(1);
            fRMotor.setPower(-1);
            bRMotor.setPower(-1);
        }
        telemetry.addData("Angle: ", yaw);
    }
}
