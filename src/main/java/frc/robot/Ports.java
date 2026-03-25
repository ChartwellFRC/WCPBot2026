package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kCANivoreCANBus = new CANBus("main");
    public static final CANBus kRoboRioCANBus = kCANivoreCANBus;

    // Talon FX IDs
    public static final int kIntakePivot = 52;
    public static final int kIntakeRollers = 50 ;
    public static final int kFloor = 53;
    public static final int kFeeder = 13;
    public static final int kShooterLeft = 14;
    public static final int kShooterMiddle = 15;
    public static final int kShooterRight = 16;
    public static final int kHanger = 18;

    // PWM Ports
    public static final int kHoodLeftServo = 3;
    public static final int kHoodRightServo = 4;
}
