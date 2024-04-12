package org.firstinspires.ftc.robotcontroller.internal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class MainTeleOp extends OpMode {
    private DcMotor motorFlippy, motorSpinny, motorExtendy, motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    private Servo servoFlippyJr, servoShooty;
    private IMU imu;
    private BNO055IMU imubn;

    private int flippyPosition, extendyPosition;
    private double flippyJrPosition, driveScale;

    @Override
    public void init() {
        initializeHardware();
        initializeCamera();
        setInitialPositions();
    }

    private void initializeHardware() {
        motorFlippy = hardwareMap.dcMotor.get("flippy");
        motorFrontLeft = hardwareMap.dcMotor.get("LF");
        motorBackLeft = hardwareMap.dcMotor.get("LR");
        motorFrontRight = hardwareMap.dcMotor.get("RF");
        motorBackRight = hardwareMap.dcMotor.get("RR");
        motorExtendy = hardwareMap.dcMotor.get("extendy");
        motorSpinny = hardwareMap.dcMotor.get("spinny");
        servoFlippyJr = hardwareMap.servo.get("flippyJr");
        servoShooty = hardwareMap.servo.get("shooty");
        setInitialPositions();
        motorExtendy.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFlippy.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFlippy.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        driveScale = 0.55; // Default driving speed scale
    }

    private void initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
    }

    private void setInitialPositions() {
        flippyPosition = 0;
        flippyJrPosition = 0.03; // Set initial position for Flippy Jr.
        extendyPosition = 0;

        updateActuatorPositions();
    }

    @Override
    public void loop() {
        handleDriving();
        handleActuatorControls();
        logPositions();
    }

    private void handleDriving() {
        adjustDriveScaleBasedOnDpad();
        controlDriveMotors();
    }

    private void adjustDriveScaleBasedOnDpad() {
        if (gamepad1.dpad_down) {
            driveScale = 0.30; // Slow mode
        } else if (gamepad1.dpad_up) {
            driveScale = 0.75; // Fast mode
        } else {
            driveScale = 0.55; // Normal mode
        }
    }

    private void controlDriveMotors() {
        double y = gamepad1.left_stick_y; // Reverse Y-axis
        double x = gamepad1.left_stick_x * 1.1; // Adjust for strafing
        double rx = gamepad1.right_stick_x; // Rotation

        // Calculate rotation for omnidirectional movement
        double botHeading = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle - Math.PI/2;
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Scale power for each motor
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        motorFrontLeft.setPower(driveScale * (rotY + rotX + rx) / denominator);
        motorBackLeft.setPower(driveScale * (rotY - rotX + rx) / denominator);
        motorFrontRight.setPower(driveScale * (rotY - rotX - rx) / denominator);
        motorBackRight.setPower(driveScale * (rotY + rotX - rx) / denominator);
    }

    private void handleActuatorControls() {
        adjustFlippyPosition();
        adjustFlippyJrPosition();
        handleShooterServo();
        adjustExtendyPosition();
        handleSpinnyMotor();
        updateActuatorPositions();
    }

    private void adjustFlippyPosition() {
        final int sensitivity = 10; // Adjust sensitivity here

        if (gamepad2.dpad_up) {
            flippyPosition += sensitivity;
        } else if (gamepad2.dpad_down) {
            flippyPosition -= sensitivity;
        } else if (gamepad2.dpad_left) {
            flippyPosition = 1260;
        }

        // Ensure flippyPosition never goes below 0
        if (flippyPosition < 0) {
            flippyPosition = 0;
        }
    }

    private void adjustFlippyJrPosition() {
        if (gamepad2.right_bumper && flippyJrPosition > 0) {
            flippyJrPosition -= 0.002;
        } else if (gamepad2.left_bumper && flippyJrPosition < 1.0) {
            flippyJrPosition += 0.002;
        }
    }

    private void adjustExtendyPosition() {
        final int sensitivity = 50; // Adjust sensitivity here

        if (gamepad2.right_trigger > 0.8) {
            extendyPosition = 1150;
        } else if (gamepad2.left_trigger > 0.8) {
            flippyJrPosition = 0.35;
        } else if (gamepad2.x) {
            flippyJrPosition = 0.11;
            updateActuatorPositions();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            extendyPosition = 400;
            updateActuatorPositions();
            try {
                Thread.sleep(1500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            flippyJrPosition = 0.03;
            updateActuatorPositions();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            extendyPosition = 0;
            updateActuatorPositions();
        }

        // Ensure extendyPosition never goes below 0
        if (extendyPosition < 0) {
            extendyPosition = 0;
        }

        if (extendyPosition > 1150) {
            extendyPosition = 1150;
        }
    }

    private void handleShooterServo() {
        if (gamepad1.x) {
            servoShooty.setPosition(0.5); // Assume 0.6 is the position to shoot
        } else if (gamepad1.y) {
            servoShooty.setPosition(0.0); // Assume 0.0 is the reset position
        }
    }

    private void handleSpinnyMotor() {
        if (gamepad1.left_bumper) {
            motorSpinny.setPower(0.5);
        } else if (gamepad1.right_bumper) {
            motorSpinny.setPower(-0.69);
        } else {
            motorSpinny.setPower(0);
        }
    }

    private void updateActuatorPositions() {
        motorFlippy.setTargetPosition(flippyPosition);
        motorFlippy.setPower(1.0);
        servoFlippyJr.setPosition(flippyJrPosition);
        motorExtendy.setTargetPosition(extendyPosition);
        motorExtendy.setPower(0.5);
    }

    private void logPositions() {
        telemetry.addData("Flippy Position", flippyPosition);
        telemetry.addData("Flippy Jr. Position", flippyJrPosition);
        telemetry.addData("Extendy Position", extendyPosition);
        telemetry.addData("IMU", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle);
        telemetry.addData("IMU", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle);
        telemetry.addData("IMU", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle);
        telemetry.update();
    }
}