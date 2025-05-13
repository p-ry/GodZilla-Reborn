package frc.robot;
public class Vector2D {
    public final double x, y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double dot(Vector2D other) {
        return this.x * other.x + this.y * other.y;
    }

    public double cross(Vector2D other) {
        return this.x * other.y - this.y * other.x;
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public Vector2D normalize() {
        double mag = magnitude();
        return (mag == 0) ? new Vector2D(0, 0) : new Vector2D(x / mag, y / mag);
    }
}
