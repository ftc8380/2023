package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Sliders {
    DcMotor motorLiftLeft;
    DcMotor motorLiftRight;
    Servo servoIntake;
    DcMotor motorV4b;

    private double LIFT_TPR = 384.5;
    private double V4B_TPR = 5281.1;

    public Sliders(HardwareMap hm) {
        motorLiftLeft = hm.dcMotor.get("motor lift left");
        motorLiftRight = hm.dcMotor.get("motor lift right");
        servoIntake = hm.servo.get("servo intake");
        motorLiftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftLeft.setTargetPosition(0);
        motorLiftRight.setTargetPosition(0);
        motorLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorV4b = hm.dcMotor.get("motor lift");
        motorV4b.setDirection(DcMotorSimple.Direction.REVERSE);
        motorV4b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorV4b.setTargetPosition(0);
        motorV4b.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void openClaw() {
        servoIntake.setPosition(0.8);
    }

    public void closeClaw() {
        servoIntake.setPosition(0.7);
    }

    public void v4b(double rotations) {
        int ticks = (int) (V4B_TPR * rotations);
        motorV4b.setPower(0.7);
        motorV4b.setTargetPosition(ticks);
    }

    public void motorV4bUp() {
        v4b(0.5);
    }

    public void motorV4bDown() {
        v4b(0);
        closeClaw();
    }

    public void motorGrouping(double rotations){
        int ticks = (int) (rotations * LIFT_TPR);
        motorLiftLeft.setPower(0.7);
        motorLiftRight.setPower(0.7);
        motorLiftLeft.setTargetPosition(ticks);
        motorLiftRight.setTargetPosition(ticks + (int) (0.2 * LIFT_TPR));
    }

    public void floor(){
        motorGrouping(0);
        motorV4bDown();
    }
    public void highJunction(){
        motorGrouping(3);
        motorV4bUp();
    }
    //we might just use v4b arms for this
    public void lowJunction(){
        motorGrouping(0.07);
        motorV4bDown();
    }
    public void midJunction(){
        motorGrouping(1.2);
        motorV4bUp();
    }
}