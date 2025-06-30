package org.chillout1778;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Utils {
    public static double deadZone(double inpt, double zone) {
        // Fancy deadzone: interpolate values from 0.1 to 1.0 into the
        // range 0.0 to 1.0.
        // https://web.archive.org/web/20181021234413/http://www.gamasutra.com/blogs/JoshSutphin/20130416/190541/Doing_Thumbstick_Dead_Zones_Right.php
        if (Math.abs(inpt) < zone) {
            return 0.0;
        } else if (inpt > 1.0) {
            return 1.0;
        } else if (inpt < -1.0) {
            return -1.0;
        } else {
            return ((inpt - zone) / (1.0 - zone));
        }
    }

    public static double unclampedDeadzone(double inpt, double zone) {
        return Math.abs(inpt) < zone ? 0.0 : inpt;
    }

    public static double wrapTo0_2PI(double a) {
        double asdf = MathUtil.angleModulus(a); // wraps from -PI to +PI
        return asdf < 0 ? asdf + 2 * Math.PI : asdf;
    }

    public static boolean isInsideField(Translation2d translation) {
        return translation.getX() > 0.0 && translation.getY() > 0.0 &&
                translation.getX() < Constants.Field.FIELD_X_SIZE &&
                translation.getY() < Constants.Field.FIELD_Y_SIZE;
    }

    public static Translation2d mirror(Translation2d translation) {
        return new Translation2d(Constants.Field.FIELD_X_SIZE - translation.getX(), translation.getY());
    }

    public static Rotation2d mirror(Rotation2d rotation) {
        return Rotation2d.kPi.minus(rotation);
    }

    public static Pose2d mirror(Pose2d pose) {
        return new Pose2d(mirror(pose.getTranslation()), mirror(pose.getRotation()));
    }

    public static Translation2d mirrorIfRed(Translation2d translation) {
        return Robot.isRedAlliance() ? mirror(translation) : translation;
    }

    public static Rotation2d mirrorIfRed(Rotation2d rotation) {
        return Robot.isRedAlliance() ? mirror(rotation) : rotation;
    }

    public static Pose2d mirrorIfRed(Pose2d pose) {
        return Robot.isRedAlliance() ? mirror(pose) : pose;
    }

    public static void addClosedLoopProperties(String name, TalonFX motor, SendableBuilder builder) {
        builder.addDoubleProperty(name + " voltage (V)", () -> motor.getMotorVoltage().getValueAsDouble(), (value) -> {
        });
        // builder.addDoubleProperty(name + " supply Current (As)", () ->
        // motor.getSupplyCurrent().getValueAsDouble(), (value) -> {});
        // builder.addDoubleProperty(name + " stator Current (A)", () ->
        // motor.getStatorCurrent().getValueAsDouble(), (value) -> {});
        builder.addDoubleProperty(name + " raw velocity", () -> motor.getVelocity().getValueAsDouble(), (value) -> {
        });
        builder.addDoubleProperty(name + " raw acceleration", () -> motor.getAcceleration().getValueAsDouble(),
                (value) -> {
                });
        // builder.addDoubleProperty(name + " position", () ->
        // motor.getPosition().getValueAsDouble(), (value) -> {});
        // builder.addDoubleProperty(name + " motion magic setpoint*360", () ->
        // motor.getClosedLoopReference().getValueAsDouble() * 360.0, (value) -> {});
    }
}
