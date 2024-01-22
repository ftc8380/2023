package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Park RB", group="Robot")
public class AutoParkRB extends LinearOpMode {
    // Assume these are defined earlier in your code
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    BNO055IMU imu;

    double scale = 0.55; // 1.0 speed is a bit too fast

    // Method to set motor powers
    void setMotorPowers(double frontLeft, double backLeft, double frontRight, double backRight) {
        motorFrontLeft.setPower(frontLeft);
        motorBackLeft.setPower(backLeft);
        motorFrontRight.setPower(frontRight);
        motorBackRight.setPower(backRight);
    }

    // Autonomous parking routine
    public void runOpMode() throws InterruptedException {
        // Initialize motors and IMU here
        motorFrontLeft = hardwareMap.dcMotor.get("LF");
        motorBackLeft = hardwareMap.dcMotor.get("LR");
        motorFrontRight = hardwareMap.dcMotor.get("RF");
        motorBackRight = hardwareMap.dcMotor.get("RR");
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        ElapsedTime runtime = new ElapsedTime();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Once opMode is active, the robot will start executing the following commands
        if (opModeIsActive()) {
            Thread.sleep(15000);
            // Move forward for 2 seconds
            double y = 1.0;
            double x = 0.0;
            double rx = 0.0;
            updateMotorPowers(x, y, rx);
            runtime.reset();
            while (opModeIsActive() && (runtime.milliseconds() < 1600)) {
                telemetry.addData("Path", "Moving Forward: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Strafe right for 4 seconds
            y = 0.0;
            x = 1.0;
            updateMotorPowers(x, y, rx);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 4.0)) {
                telemetry.addData("Path", "Strafing Right: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Stop the robot
            setMotorPowers(0, 0, 0, 0);
        }
    }

    // Update motor powers based on x, y, and rx values
    void updateMotorPowers(double x, double y, double rx) {
        double botHeading = -imu.getAngularOrientation().firstAngle;
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = scale * (rotY + rotX + rx) / denominator;
        double backLeftPower = scale * (rotY - rotX + rx) / denominator;
        double frontRightPower = scale * (rotY - rotX - rx) / denominator;
        double backRightPower = scale * (rotY + rotX - rx) / denominator;

        setMotorPowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }
}
