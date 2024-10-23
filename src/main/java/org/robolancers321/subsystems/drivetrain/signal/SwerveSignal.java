package org.robolancers321.subsystems.drivetrain.signal;

import java.util.Arrays;

import org.robolancers321.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class SwerveSignal {
    public abstract ChassisSpeeds apply(Drivetrain drivetrain); 

    public static SwerveSignal fuse(SwerveSignal... signals) {
        return new SwerveSignal() {
            public ChassisSpeeds apply(Drivetrain drivetrain) {
                return Arrays.asList(signals).stream().map(e -> e.apply(drivetrain)).reduce(new ChassisSpeeds(), (a, b) -> new ChassisSpeeds(
                    a.vxMetersPerSecond + b.vxMetersPerSecond, 
                    a.vyMetersPerSecond + b.vyMetersPerSecond, 
                    a.omegaRadiansPerSecond + b.omegaRadiansPerSecond
                    )
                ); 
            }
        };
    }

    public static SwerveSignal empty() {
        return new DriveSwerveSignal(); 
    }

    public SwerveSignal with(SwerveSignal anotherSignal) {
        return fuse(this, anotherSignal); 
    }
}
