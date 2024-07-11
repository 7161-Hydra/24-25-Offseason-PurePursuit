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
    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        p1 = new Pose2d(0,0,0);
        drive.setPoseEstimate(p1);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        ArrayList<CurvePoint> allPoints = new ArrayList();
        allPoints.add(new CurvePoint(0.0, 0.0, 1.0, 1.0, 50.0, Math.toRadians(50.0), 1.0));
        allPoints.add(new CurvePoint(10, 10, .5, .3, 50, Math.toRadians(50), 1));
        allPoints.add(new CurvePoint(5, 20, .5, .3, 50, Math.toRadians(50), 1));
        ArrayList<Double> p = Movement.followCurve(allPoints, Math.toRadians(90.0), drive.getPoseEstimate());

    }
}
