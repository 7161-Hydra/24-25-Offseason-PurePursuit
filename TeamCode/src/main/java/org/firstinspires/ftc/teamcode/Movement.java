package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PPMath.AngleWrap;
import static org.firstinspires.ftc.teamcode.PPMath.lineCircleIntersection;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.Range;
import org.opencv.core.Point;

import java.util.ArrayList;

public class Movement {
    public static ArrayList<Double> goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed, Pose2d p1){
        double absouluteAngleToTarget = Math.atan2(y-p1.getY(), x-p1.getX());
        double relativeAngleToPoint = AngleWrap(absouluteAngleToTarget-p1.getHeading());
        double distanceToTarget = Math.hypot(x-p1.getX(), y-p1.getY());
        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeXToPoint));
        double movement_X = movementXPower * movementSpeed;
        double movement_Y = movementYPower * movementSpeed;
        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        double movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

        if(distanceToTarget < 10){
            movement_turn = 0;
        }
        ArrayList<Double> powers = new ArrayList<>();
        powers.add(movement_Y);
        powers.add(movement_X);
        powers.add(movement_turn);
        return powers;
    }
    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, double xPos, double yPos, double followRadius, Pose2d p1){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));
        for(int i = 0; i < pathPoints.size() -1; i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);
            ArrayList<Point> intersections = lineCircleIntersection(new Point(xPos,yPos), followRadius, startLine.toPoint(), endLine.toPoint());
            double closestAngle = 1000000000;
            for(Point thisIntersection: intersections){
                double angle = Math.atan2(thisIntersection.y-p1.getY(), thisIntersection.x)-p1.getX();
                double deltaAngle = Math.abs(AngleWrap(angle - p1.getHeading()));
                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }
    public static ArrayList<Double> followCurve(ArrayList<CurvePoint> allPoints, double followAngle, Pose2d p1){
        CurvePoint followMe = getFollowPointPath(allPoints, p1.getX(), p1.getY(), allPoints.get(0).followDistance, p1);
        ArrayList<Double> p = goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed, p1);
        return p;
    }
}
