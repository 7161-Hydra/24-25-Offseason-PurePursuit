package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Localizer {
    public double x, y, h;
    public IMU imu;
    private double perpPos, parPos;
    public DcMotor perpEncoder, parEncoder;
    public Localizer(DcMotor Fl, DcMotor Fr, double x, double y, double h, IMU inertial) {
        imu = inertial;

        perpEncoder = Fl;
        parEncoder = Fr;
        perpEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        parEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        perpPos = 0;
        parPos = 0;

        this.x = x;
        this.y = y;
        this.h = h;

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void update(){
        this.h = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        y += (Math.abs(perpEncoder.getCurrentPosition()-perpPos) * Math.cos(h)) + (Math.abs(parEncoder.getCurrentPosition()-parPos) * Math.sin(h));
        x += (Math.abs(perpEncoder.getCurrentPosition()-perpPos) * Math.sin(h)) + (Math.abs(parEncoder.getCurrentPosition()-parPos) * Math.cos(h));
        perpPos = perpEncoder.getCurrentPosition();
        parPos = parEncoder.getCurrentPosition();
    }

    public double getX(){
        return .625 * 2 * Math.PI * x / 8192;
    }
    public double getY(){
        return .625 * 2 * Math.PI * y / 8192;
    }
    public double getH(){
        return .625 * 2 * Math.PI * h / 8192;
    }
    public String returnLoc(){
        return "(" + getX() + " , " + getY() + " , " + getH() + ")";
    }
}