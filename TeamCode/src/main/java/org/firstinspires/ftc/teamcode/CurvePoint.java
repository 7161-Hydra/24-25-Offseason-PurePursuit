package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed, turnSpeed, followDistance, pointLength, slowDownTurnRadians, slowDownTurnAmount;
    public CurvePoint(double X, double Y, double mS, double tS, double fD, double sDTR, double sDTA){
        x = X;
        y = Y;
        moveSpeed = mS;
        turnSpeed = tS;
        followDistance = fD;
        slowDownTurnRadians = sDTR;
        slowDownTurnAmount = sDTA;
    }
    public CurvePoint(CurvePoint tP){
        x = tP.x;
        y = tP.y;
        moveSpeed = tP.moveSpeed;
        turnSpeed = tP.turnSpeed;
        followDistance = tP.followDistance;
        slowDownTurnRadians = tP.slowDownTurnRadians;
        slowDownTurnAmount = tP.slowDownTurnAmount;
        pointLength = tP.pointLength;
    }
    public Point toPoint(){
        return new Point(x, y);
    }
    public void setPoint(Point point){
        x = point.x;
        y = point.y;
    }
}