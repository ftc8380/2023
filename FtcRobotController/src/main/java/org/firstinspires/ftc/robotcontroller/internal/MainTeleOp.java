package org.firstinspires.ftc.robotcontroller.internal;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MainTeleOp extends OpMode {
    DcMotor flippy;
    int flippyPos;
    double grabbyPos, twistyPos, shootyPushPos;

    BNO055IMU imu;

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor shooty;

    Servo twisty, grabby, shootyPush;
    double scale = 0.55; // 1.0 speed is a bit too fast

    @Override
    public void init() {
        flippy = hardwareMap.dcMotor.get("flippy");
        flippyPos = flippy.getCurrentPosition();
        flippy.setTargetPosition(flippyPos);
        flippy.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flippy.setPower(1.0);

        motorFrontLeft = hardwareMap.dcMotor.get("LF");
        motorBackLeft = hardwareMap.dcMotor.get("LR");
        motorFrontRight = hardwareMap.dcMotor.get("RF");
        motorBackRight = hardwareMap.dcMotor.get("RR");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        twisty = hardwareMap.servo.get("twisty");
        grabby = hardwareMap.servo.get("grabby");
        shootyPush = hardwareMap.servo.get("shootyPushy");

        twisty.setDirection(Servo.Direction.REVERSE);

        twistyPos = -7.52;
        grabbyPos = 0.63;
        flippyPos = 202;
        shootyPushPos = 0.7;

        shooty = hardwareMap.dcMotor.get("shooty");

        // move to

        flippy.setTargetPosition(flippyPos);
        twisty.setPosition(twistyPos);

        shooty.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        Log.i("FlippyPos",String.valueOf(flippyPos));
        Log.i("TwistyPos",String.valueOf(twistyPos));
        Log.i("GrabbyPos",String.valueOf(grabbyPos));
        Log.i("ShootyPushPos",String.valueOf(shootyPushPos));

        // Slow mode
        if(gamepad1.dpad_down) {
            scale = 0.30;
        } else if(gamepad1.dpad_left) {
            scale = 0.55;
        } else if(gamepad1.dpad_right) {
            scale = 0.55;
        } else if(gamepad1.dpad_up) {
            scale = 0.75;
        }

        if (gamepad2.dpad_up) {
            flippyPos -= 5;
            flippy.setPower(1.0);
        } else if (gamepad2.dpad_down) {
            flippyPos += 5;
            flippy.setPower(1.0);
        }


        flippy.setTargetPosition(flippyPos);

        if (gamepad2.right_trigger > 0.70) {
            grabbyPos = 0.63;
        } else if (gamepad2.left_trigger > 0.70) {
            grabbyPos = 0.79;
        }

        if (gamepad2.right_bumper && twistyPos > 0) {
            twistyPos -= 0.01;
        } else if (gamepad2.left_bumper && twistyPos <= 1.0) {
            twistyPos += 0.01;
        }

        if (gamepad1.x) {
            shooty.setPower(1.0);
            try {
                Thread.sleep(450);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            shootyPushPos = 0.0;
        } else if(gamepad1.y){
            shooty.setPower(0.0);
        }
        if (gamepad1.right_bumper) {
            shootyPushPos +=0.05;
        } else if (gamepad1.left_bumper) {
            shootyPushPos -=0.05;
        }

        if (gamepad2.x) {
            //move to grab position
            flippyPos = -33;
            /*flippy.setPower(0.6);
            flippy.setTargetPosition(137);
            flippy.setPower(1.0);*/
            twistyPos = 0.589999999999;
            grabbyPos = 0.63;
        }

        if (gamepad2.y) {
            // move to put position
            flippyPos = -(2388 + 33);
            twistyPos = 0.18;
            grabbyPos = 0.63;
        }
        if (gamepad2.b) {
            flippyPos = -1778;
            twistyPos= 0.69;
        }
        twisty.setPosition(twistyPos);
        grabby.setPosition(grabbyPos);
        shootyPush.setPosition(shootyPushPos);

        double y = -gamepad1.left_stick_y; // Why is y still reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        double botHeading = -imu.getAngularOrientation().firstAngle;
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = scale * (rotY + rotX + rx) / denominator;
        double backLeftPower = scale * (rotY - rotX + rx) / denominator;
        double frontRightPower = scale * (rotY - rotX - rx) / denominator;
        double backRightPower = scale * (rotY + rotX - rx) / denominator;
        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
