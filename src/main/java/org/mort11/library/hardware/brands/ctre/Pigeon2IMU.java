package org.mort11.library.hardware.brands.ctre;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import org.mort11.library.hardware.imu.IMUIntf;

import com.ctre.phoenix6.hardware.Pigeon2;

public class Pigeon2IMU implements IMUIntf {

    public Pigeon2 imu;

    public int ID;

    public Pigeon2IMU(int ID) {
        this.ID = ID;

        imu = new Pigeon2(ID);
    }

    public void setCanivore(String canivore) {
        imu = new Pigeon2(ID, canivore);
    }

    public double getAngle() {
        // Phoenix 6 v26 removed Pigeon2.getAngle(); it was -getYaw(), negated because
        // getAngle() is cw+ while the Pigeon reports ccw+.
        return -imu.getYaw().getValueAsDouble();
    }

    public double getRate() {
        // Likewise getRate() was -getAngularVelocityZWorld(), cw+ vs ccw+.
        return -imu.getAngularVelocityZWorld().getValueAsDouble();
    }

    public double getAccelerationX() {
        return imu.getAccelerationX().getValueAsDouble();
    }

    public double getAccelerationY() {
        return imu.getAccelerationY().getValueAsDouble();
    }

    public double getAccelerationZ() {
        return imu.getAccelerationZ().getValueAsDouble();
    }

    public double getAcceleration() {
        return Math.sqrt(
            getAccelerationX() * getAccelerationX() +
            getAccelerationY() * getAccelerationY() +
            getAccelerationZ() * getAccelerationZ()
        );
    }

    public void reset () {
        imu.reset();
    }
    
    public Rotation2d getRotation2d() {
        return imu.getRotation2d();
    }

    public Rotation3d getRotation3d() {
        return imu.getRotation3d();
    }

    public Pigeon2 getIMU() {
        return imu;
    }

}
