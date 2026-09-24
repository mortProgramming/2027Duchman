package org.mort11.library.hardware.brands.kauailabs;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import org.mort11.library.hardware.imu.IMUIntf;

import com.studica.frc.Navx;

// StudicaLib 2026 replaced AHRS with Navx, which talks to a NavX3 over CAN or USB.
// The old MXP-SPI NavX2 this class used to construct is not supported by that library,
// so ID is now the CAN device id instead of being ignored.
public class NavX2IMU implements IMUIntf {

    public Navx imu;

    public int ID;

    public NavX2IMU(int ID) {
        this.ID = ID;

        imu = new Navx(ID);

        // Replaces the old isMagnetometerCalibrated()/getFusedHeading() pair: magnetometer
        // fusion is now a mode on the IMU, and getYaw() returns the fused value once it's on.
        imu.enable9DYaw(true);
    }

    public void setCanivore (String canivore) {
        System.out.println("Why are you here?");
    }

    public double getAngle() {
		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise
		// makes the angle increase.
		return 360.0 - imu.getYaw().in(Degrees);
	}

    public double getRate() {
        // {x, y, z} deg/s — z is the yaw rate the old getRate() returned.
        return imu.getAngularVel()[2].in(DegreesPerSecond);
    }

    public void reset () {
        imu.resetYaw();
    }
    
    public Rotation2d getRotation2d() {
        return imu.getRotation2d();
    }

    public Rotation3d getRotation3d() {
        return imu.getRotation3d();
    }

    // public double getAccelerationX() {
    //     return imu.get
    // }

    // public double getAccelerationY() {
    //     return imu.getAccelerationY().getValueAsDouble();
    // }

    // public double getAccelerationZ() {
    //     return imu.getAccelerationZ().getValueAsDouble();
    // }

    // public double getAcceleration() {
    //     return Math.sqrt(
    //         getAccelerationX() * getAccelerationX() +
    //         getAccelerationY() * getAccelerationY() +
    //         getAccelerationZ() * getAccelerationZ()
    //     );
    // }

    public Navx getIMU() {
        return imu;
    }
}
