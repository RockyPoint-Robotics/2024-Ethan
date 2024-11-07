package org.firstinspires.ftc.teamcode;

import com.google.blocks.ftcrobotcontroller.runtime.ImuAccess;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.security.KeyPairGenerator;

@TeleOp(name="Tessaract")
public class TessaractTeleOp extends OpMode {

    public DcMotor fLMotor;
    public DcMotor fRMotor;
    public DcMotor bLMotor;
    public DcMotor bRMotor;
    public DcMotor armMotor;
    public Servo claw;
    public Vector2D lJoyPos = Vector2D.ZERO;
    double armSpeed = 2;
    private KeyPairGenerator imu;

    @Override
    public void init() {
        fLMotor = hardwareMap.get(DcMotor.class, "FL");
        fRMotor = hardwareMap.get(DcMotor.class, "FR");
        bLMotor = hardwareMap.get(DcMotor.class, "BL");
        bRMotor = hardwareMap.get(DcMotor.class, "BR");
        armMotor = hardwareMap.get(DcMotor.class, "ARM");
        claw = hardwareMap.get(Servo.class, "CLAW");
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
        Vector2D lJoyPos = new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double y = gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double lJoyAngle = Math.atan2(y, x);
        double lJoyDistance = Math.sqrt(x * x + y * y);
        double rx = gamepad1.right_stick_x;






        //double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);






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
    }
}
