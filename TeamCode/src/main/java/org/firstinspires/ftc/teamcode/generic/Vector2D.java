package org.firstinspires.ftc.teamcode.generic;

/// A 2D vector object with doubles for its components
public class Vector2D {
	public double x;
	public double y;

	public Vector2D (double x, double y) {
		this.x = x;
		this.y = y;
	}

	public Vector2D () {
		this.x = 0.0;
		this.y = 0.0;
	}

	/// Adds two vectors together, returning the result
	public Vector2D add(Vector2D other) {
		return new Vector2D(this.x + other.x, this.y + other.y);
	}

	/// Subtracts other from this, returning the result
	public Vector2D sub(Vector2D other) {
		return new Vector2D(this.x - other.x, this.y - other.y);
	}

	/// Multiplies both components by a scalar value
	public Vector2D mul_by(double scalar) {
		return new Vector2D(this.x * scalar, this.y * scalar);
	}

	/// Divides both components by a scalar value
	public Vector2D div_by(double scalar) {
		return new Vector2D(this.x / scalar, this.y / scalar);
	}

	/// Returns the scalar product of both vectors
	public double scalarProduct(Vector2D other) {
		return this.x * other.x + this.y * other.y;
	}

	/// Returns self rotated rads counterclockwise
	public Vector2D rotateCCWFor(double rads) {

		double sin_theta = Math.sin(rads);
		double cos_theta = Math.cos(rads);

		return new Vector2D(this.x * cos_theta - this.y * sin_theta, this.x * sin_theta + y * cos_theta);
	}

	/// Returns self rotated rads clockwise
	public Vector2D rotateCWFor(double rads) {
		return rotateCCWFor(-rads);
	}

	/// Returns the length of this vector
	public double length() {
		return Math.sqrt(this.x * this.x + this.y * this.y);
	}

	/// Makes the length of this vector 1 and returns the result
	public Vector2D normalize() {
		double length = length();

		return new Vector2D(this.x / length, this.y / length);
	}
}
