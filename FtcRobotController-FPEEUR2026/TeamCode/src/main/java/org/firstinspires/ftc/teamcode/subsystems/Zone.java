package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

public class Zone {

    public static class Point {
        public final double x, y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    private final Point a, b, c;
    private final double minX, maxX, minY, maxY, radius;

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

        // Bounding box check
        if (px - radius > maxX || px + radius < minX ||
                py - radius > maxY || py + radius < minY) {
            return false;
        }

        // Inside triangle
        if (isInsideTriangle(px, py)) return true;

        // Close to edges
        return distanceToSegment(px, py, a, b) <= radius ||
                distanceToSegment(px, py, b, c) <= radius ||
                distanceToSegment(px, py, c, a) <= radius;
    }

    private boolean isInsideTriangle(double px, double py) {
        double denominator = ((b.y - c.y)*(a.x - c.x) + (c.x - b.x)*(a.y - c.y));
        double alpha = ((b.y - c.y)*(px - c.x) + (c.x - b.x)*(py - c.y)) / denominator;
        double beta  = ((c.y - a.y)*(px - c.x) + (a.x - c.x)*(py - c.y)) / denominator;
        double gamma = 1.0 - alpha - beta;
        return alpha >= 0 && beta >= 0 && gamma >= 0;
    }

    private double distanceToSegment(double px, double py, Point p1, Point p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;

        if (dx == 0 && dy == 0) return Math.hypot(px - p1.x, py - p1.y);

        double t = ((px - p1.x) * dx + (py - p1.y) * dy) / (dx*dx + dy*dy);
        t = Math.max(0, Math.min(1, t));

        double closestX = p1.x + t * dx;
        double closestY = p1.y + t * dy;

        return Math.hypot(px - closestX, py - closestY);
    }
}