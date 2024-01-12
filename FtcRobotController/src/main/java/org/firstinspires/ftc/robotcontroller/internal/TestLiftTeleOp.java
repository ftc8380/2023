package org.firstinspires.ftc.robotcontroller.internal;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestLiftTeleOp extends OpMode {
    DcMotor shooty;
    Servo shootyPushy;
    @Override
    public void init() {
        shooty = hardwareMap.dcMotor.get("shooty");
        shootyPushy = hardwareMap.servo.get("shootyPushy");
    }

    @Override
    public void loop() {
        double shootyPushyPos = 0.0;
        if (gamepad1.a) {
            shooty.setPower(1.0);
            shootyPushyPos += 0.5;
            shootyPushy.setPosition(shootyPushyPos);
        }
        if (gamepad1.b) {
            shooty.setPower(0.0);
        }
        if (gamepad1.options) {
            Log.i("breakpoint", "breakpoint");
        }
    }
}
