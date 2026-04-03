package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

public class Zone {
    public static class Point {
        public double x, y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
    private Point a, b, c;

    // bounding box del triángulo
    private double minX, maxX, minY, maxY,radius;

    public Zone(Point a, Point b, Point c, double radius) {
        this.a = a;
        this.b = b;
        this.c = c;

        this.minX = Math.min(a.x, Math.min(b.x, c.x));
        this.maxX = Math.max(a.x, Math.max(b.x, c.x));
        this.minY = Math.min(a.y, Math.min(b.y, c.y));
        this.maxY = Math.max(a.y, Math.max(b.y, c.y));

        this.radius = radius;
    }

    public boolean isRobotInZone(Pose pose) {
        double px = pose.getX();
        double py = pose.getY();

        if (px + radius < minX || px - radius > maxX ||
                py + radius < minY || py - radius > maxY) {
            return false;
        }

        if (isInsideTriangle(px, py)) return true;

        if (distanceToSegment(px, py, a.x, a.y, b.x, b.y) <= radius) return true;
        if (distanceToSegment(px, py, b.x, b.y, c.x, c.y) <= radius) return true;
        return distanceToSegment(px, py, c.x, c.y, a.x, a.y) <= radius;
    }

    private boolean isInsideTriangle(double px, double py) {
        double areaABC = area(a.x, a.y, b.x, b.y, c.x, c.y);
        double areaPAB = area(px, py, a.x, a.y, b.x, b.y);
        double areaPBC = area(px, py, b.x, b.y, c.x, c.y);
        double areaPCA = area(px, py, c.x, c.y, a.x, a.y);

        return Math.abs((areaPAB + areaPBC + areaPCA) - areaABC) < 1e-6;
    }

    private double area(double x1, double y1,
                        double x2, double y2,
                        double x3, double y3) {
        return Math.abs(
                (x1 * (y2 - y3) +
                        x2 * (y3 - y1) +
                        x3 * (y1 - y2)) * 0.5
        );
    }

    private double distanceToSegment(double px, double py,
                                     double x1, double y1,
                                     double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;

        if (dx == 0 && dy == 0) {
            return Math.hypot(px - x1, py - y1);
        }

        double t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);

        if (t < 0) t = 0;
        else if (t > 1) t = 1;

        double closestX = x1 + t * dx;
        double closestY = y1 + t * dy;

        return Math.hypot(px - closestX, py - closestY);
    }
}