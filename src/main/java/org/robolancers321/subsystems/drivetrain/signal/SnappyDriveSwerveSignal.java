package org.robolancers321.subsystems.drivetrain.signal;

import org.robolancers321.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SnappyDriveSwerveSignal extends DriveSwerveSignal {
    private Rotation2d snapAngle = Rotation2d.fromDegrees(0); 

    @Override
    public ChassisSpeeds apply(Drivetrain drivetrain) {
        withRotationVelocity(drivetrain.getSwerveDrive().swerveController.getRawTargetSpeeds(0, 0, snapAngle.getRadians(), drivetrain.getPose().getRotation().getRadians()).omegaRadiansPerSecond); 
        return super.apply(drivetrain); 
    }

    public SnappyDriveSwerveSignal withSnapAngle(Rotation2d angle) {
        this.snapAngle = angle; 
        return this; 
    }
}
