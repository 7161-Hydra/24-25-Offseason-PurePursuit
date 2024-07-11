package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Autonomous
public class TestOpMode extends OpMode {
    SampleMecanumDrive drive;
    Pose2d p1;
    ArrayList<CurvePoint> allPoints = new ArrayList();
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        p1 = new Pose2d(0,0,0);
        drive.setPoseEstimate(p1);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        allPoints.add(new CurvePoint(0.0, 0.0, 1.0, 1.0, 50.0, Math.toRadians(50.0), 1.0));
        allPoints.add(new CurvePoint(10, 10, .5, .3, 50, Math.toRadians(50), 1));
        allPoints.add(new CurvePoint(5, 20, .5, .3, 50, Math.toRadians(50), 1));
    }

    @Override
    public void loop() {
        ArrayList<Double> p = Movement.followCurve(allPoints, Math.toRadians(90.0), drive.getPoseEstimate());
        double x = p.get(0);
        double y = p.get(1);
        double rx = p.get(2);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flp = (y + x + rx) / denominator;
        double blp = (y - x + rx) / denominator;
        double frp = (y - x - rx) / denominator;
        double brp = (y + x - rx) / denominator;
        drive.setMotorPowers(flp, blp, frp, brp);
    }
}
