package org.robolancers321.subsystems.drivetrain.signal;

import org.robolancers321.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveSwerveSignal extends SwerveSignal {
    private double forward = 0; 
    private double strafe = 0; 
    private double rotate = 0; 
    private boolean fieldRelative = true; 

    @Override
    public ChassisSpeeds apply(Drivetrain drivetrain) {
        ChassisSpeeds speeds = new ChassisSpeeds(forward, strafe, rotate); 
        if (fieldRelative) speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drivetrain.getPose().getRotation()); 
        return speeds; 
    }

    public DriveSwerveSignal withForwardVelocity(double velo) {
        this.forward = velo; 
        return this; 
    }

    public DriveSwerveSignal withStrafeVelocity(double velo) {
        this.strafe = velo; 
        return this; 
    }
    
    public DriveSwerveSignal withRotationVelocity(double velo) {
        this.rotate = velo; 
        return this; 
    }

    public DriveSwerveSignal withFieldRelative(boolean fieldRelative) {
        this.fieldRelative = fieldRelative; 
        return this; 
    }
    
}
