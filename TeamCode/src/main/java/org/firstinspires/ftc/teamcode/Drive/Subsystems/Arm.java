package org.firstinspires.ftc.teamcode.Drive.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    public DcMotor armLift = null;
    public Servo armDropper = null;

    private String armLiftName = "armLift";
    private String armDropperName = "armClamp";

    private int armTarget;
    private double armSpeed;
    private double armSpeedMove = 0.75;

    private int[] positions = {
            -540, // High Position
            -360, // Middle Position
            0,    // Reset Position (Bottom)
    };

    public Arm(HardwareMap hardwareMap) {
        armLift = hardwareMap.get(DcMotor.class, armLiftName);
        armDropper = hardwareMap.get(Servo.class, armDropperName);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setArmTarget(int target) {
        armTarget = positions[target];
        armSpeed = armSpeedMove;
        armLift.setTargetPosition(armTarget);
        armLift.setPower(armSpeed);
        armDropper.setPosition(0.5);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void dump() {
        armDropper.setPosition(1.0);
    }
}
