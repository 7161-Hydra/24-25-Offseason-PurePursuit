package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PPConstants.*;
import static org.firstinspires.ftc.teamcode.pathBuilder.*;
import static org.firstinspires.ftc.teamcode.pathCalculuation.calculateCurvature;
import static org.firstinspires.ftc.teamcode.pathCalculuation.calculateVelocity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous
public class pptest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Waypoint[] waypoints = {
                new Waypoint(0, 0),
                new Waypoint(1, 1),
                new Waypoint(2, 0),
                new Waypoint(3, 1),
                new Waypoint(4, 0)
        };
        double maxVelocity = 50;
        waitForStart();
        waypoints = processWaypoints(waypoints, spacing, weightData, weightSmooth, tolerance);
        calculateCurvature(waypoints);
        calculateVelocity(waypoints, maxVelocity, maxAcceleration, k);

        telemetry.addLine("Waypoints Processed!");
        telemetry.update();
        while (!isStopRequested()) {

        }
    }
}
