#include "command/subsystem.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

class Drive : public Subsystem {
private:
    // motors
    pros::MotorGroup* leftMotors;
    pros::MotorGroup* rightMotors;
    pros::Imu* imu;

    // lemlib components (as pointers for lazy initialization)
    lemlib::Drivetrain* drivetrain = nullptr;
    lemlib::ControllerSettings* linearController = nullptr;
    lemlib::ControllerSettings* angularController = nullptr;
    pros::Rotation* horizontalEnc = nullptr;
    pros::Rotation* verticalEnc = nullptr;
    lemlib::TrackingWheel* horizontal = nullptr;
    lemlib::TrackingWheel* vertical = nullptr;
    lemlib::OdomSensors* sensors = nullptr;
    lemlib::ExpoDriveCurve* throttleCurve = nullptr;
    lemlib::ExpoDriveCurve* steerCurve = nullptr;
    lemlib::Chassis* chassis = nullptr;

public:
    /**
     * Construct a new Drive subsystem with left and right motor groups
     *
     * @param leftMotorGroup motor group for the left side of the drivetrain
     * @param rightMotorGroup motor group for the right side of the drivetrain
     * @param imu IMU sensor for the drivetrain
     */
    explicit Drive(pros::MotorGroup& leftMotorGroup, pros::MotorGroup& rightMotorGroup, pros::Imu& imuSensor)
    : leftMotors(&leftMotorGroup), rightMotors(&rightMotorGroup), imu(&imuSensor) {
        
        // Initialize tracking wheel encoders
        horizontalEnc = new pros::Rotation(6);
        verticalEnc = new pros::Rotation(-5);
        
        // Initialize tracking wheels
        horizontal = new lemlib::TrackingWheel(horizontalEnc, 2.75, -3.25);
        vertical = new lemlib::TrackingWheel(verticalEnc, 2.75, -0.375);
        
        // Initialize drivetrain
        drivetrain = new lemlib::Drivetrain(leftMotors,
                                            rightMotors,
                                            10,
                                            lemlib::Omniwheel::NEW_275,
                                            450,
                                            2);
        
        // Initialize controllers
        linearController = new lemlib::ControllerSettings(10, 0, 3, 3, 1, 100, 3, 500, 20);
        angularController = new lemlib::ControllerSettings(2, 0, 10, 3, 1, 100, 3, 500, 0);
        
        // Initialize sensors
        sensors = new lemlib::OdomSensors(vertical, nullptr, horizontal, nullptr, imu);
        
        // Initialize drive curves
        throttleCurve = new lemlib::ExpoDriveCurve(3, 10, 1.019);
        steerCurve = new lemlib::ExpoDriveCurve(3, 10, 1.019);
        
        // Create chassis
        chassis = new lemlib::Chassis(*drivetrain, *linearController, *angularController, *sensors, throttleCurve, steerCurve);
    }

    lemlib::Chassis* getChassis() {
        return chassis;
    }
    
    void periodic() override {
        // EX: debugging tasks
    }

    // Free any additional resources that are needed.
    ~Drive() override {
        delete chassis;
        delete drivetrain;
        delete linearController;
        delete angularController;
        delete horizontalEnc;
        delete verticalEnc;
        delete horizontal;
        delete vertical;
        delete sensors;
        delete throttleCurve;
        delete steerCurve;
    }
};