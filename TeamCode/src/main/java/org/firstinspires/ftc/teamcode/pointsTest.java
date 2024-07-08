package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PPConstants.*;
import static org.firstinspires.ftc.teamcode.pathBuilder.processWaypoints;
import static org.firstinspires.ftc.teamcode.pathCalculuation.calculateCurvature;
import static org.firstinspires.ftc.teamcode.pathCalculuation.calculateVelocity;

public class pointsTest {
    public static void main(String[] args) {
        Waypoint[] waypoints = {
                new Waypoint(0, 0),
                new Waypoint(1, 1),
                new Waypoint(2, 0),
                new Waypoint(3, 1),
                new Waypoint(4, 0)
        };
        double maxVelocity = 50;

        waypoints = processWaypoints(waypoints,spacing,weightData,weightSmooth,tolerance);
        calculateCurvature(waypoints);
        calculateVelocity(waypoints, maxVelocity, maxAcceleration, k);


        for (Waypoint wp : waypoints) {
            System.out.println("X: " + wp.x + ", Y: " + wp.y + ", Curvature: " + wp.curvature + ", Velocity: " + wp.velocity);
        }
    }
}
