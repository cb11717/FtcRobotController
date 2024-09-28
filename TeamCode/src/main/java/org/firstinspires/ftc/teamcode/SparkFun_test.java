/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates how to use the SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS)
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_otos".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.sparkfun.com/products/24904
 */
@Autonomous(name = "SparkFun straight", group = "Sensor")
//@Disabled
public class SparkFun_test extends LinearOpMode {
    // Create an instance of the sensor
    SparkFunOTOS myOtos;
    private DcMotor LBdrive;
    private DcMotor RBdrive;
    private DcMotor LFdrive;
    private DcMotor RFdrive;
    private final double wheelDiameter = 3.94;
    private final double circumference =  wheelDiameter * 3.14;
    private double encoderTickCount = 288;
    private final int gearRatio = 2;

    @Override
    public void runOpMode() throws InterruptedException {

        encoderTickCount = encoderTickCount/gearRatio;

        // Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        LBdrive = hardwareMap.get(DcMotor.class, "LBdrive");
        RBdrive = hardwareMap.get(DcMotor.class, "RBdrive");
        LFdrive = hardwareMap.get(DcMotor.class, "LFdrive");
        RFdrive = hardwareMap.get(DcMotor.class, "RFdrive");

        RFdrive.setDirection(DcMotor.Direction.REVERSE);
        RBdrive.setDirection(DcMotor.Direction.REVERSE);
        LFdrive.setDirection(DcMotor.Direction.FORWARD);
        LBdrive.setDirection(DcMotor.Direction.FORWARD);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

        // Wait for the start button to be pressed
        waitForStart();

        // Loop until the OpMode ends
        while (opModeIsActive()) {
            // Get the latest position, which includes the x and y coordinates, plus the
            // heading angle
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            // Reset the tracking if the user requests it
            //if (gamepad1.y) {
                //myOtos.resetTracking();
            //}

            // Re-calibrate the IMU if the user requests it
            //if (gamepad1.x) {
                //myOtos.calibrateImu();
            //}

            // Inform user of available controls
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();

            // Log the position to the telemetry
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);

            // Update the telemetry on the driver station
            telemetry.update();
            moveStraightWithVelocity(24, 200, 1);
            SpinRobot(180, "RIGHT");
            moveStraightWithVelocity(24, 200, 1);

            break;
        }

    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }



    private void SpinRobot(int degreesToTurn, String directionToTurn) {

        int turningRight = 0;
        int turningLeft = 0;

        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (directionToTurn.equals("RIGHT")){
            degreesToTurn = -1 * degreesToTurn;
        }

        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        double currentOrientation = pos.h;

        degreesToTurn = (int)currentOrientation + degreesToTurn;

       /* if( directionToTurn.equals("RIGHT")){
            degreesToTurn = (int)currentOrientation - degreesToTurn;
            telemetry.addData("DEGREES_TO_TURN", degreesToTurn);
            telemetry.addData("CURRENT ORIENTATION", currentOrientation);
            telemetry.update();
            sleep(2000);

        }else if (directionToTurn.equals("LEFT")){
            degreesToTurn = (int)currentOrientation + degreesToTurn;
            telemetry.addData("DEGREES_TO_TURN", degreesToTurn);
            telemetry.addData("CURRENT ORIENTATION", currentOrientation);
            telemetry.update();
            sleep(2000);
        } */
        while (opModeIsActive()){
            pos = myOtos.getPosition();
            telemetry.addData("CURRENT ORIENTATION", Math.floor(pos.h));

            if( degreesToTurn < Math.floor(pos.h)  && directionToTurn.equals("RIGHT")){
                telemetry.addData("CURRENT ORIENTATION", currentOrientation);
                telemetry.update();

                // ensure that power to the wheels is set only once per turn
                if(turningRight == 0) {

                    telemetry.addData("SPINNING RIGHT", "FULL POWER");
                    telemetry.addData("CURRENT ORIENTATION", Math.floor(pos.h));

                    telemetry.update();
                    ((DcMotorEx) LBdrive).setVelocity(200);
                    ((DcMotorEx) RBdrive).setVelocity(-200);
                    ((DcMotorEx) LFdrive).setVelocity(200);
                    ((DcMotorEx) RFdrive).setVelocity(-200);
                    turningRight = 1;
                }
            } else if (degreesToTurn > Math.floor(pos.h) && directionToTurn.equals("LEFT")) {
                telemetry.addData("CURRENT ORIENTATION", Math.floor(pos.h));
                telemetry.update();
                // ensure that power to the wheels is set only once per turn
                if (turningLeft == 0){
                    telemetry.addData("SPINNING LEFT", "FULL POWER");
                    telemetry.update();
                    ((DcMotorEx) LBdrive).setVelocity(-200);
                    ((DcMotorEx) RBdrive).setVelocity(200);
                    ((DcMotorEx) LFdrive).setVelocity(-200);
                    ((DcMotorEx) RFdrive).setVelocity(200);
                    turningLeft = 1;
                }
            } else {
                LBdrive.setPower(0);
                RBdrive.setPower(0);
                LFdrive.setPower(0);
                RFdrive.setPower(0);
                break;
            }

        }

    }
    /**
     * this function will move the robot in a straight line to
     * the distance specified by"distanceInInches" input variable
     */
    private void moveStraightWithVelocity(double distanceInInches, int velocityInTPS, int speedFactor) {
        int newVelocity;

        telemetry.addData("MOVING STRAIGHT", distanceInInches);
        telemetry.update();
        // InputVariable : distanceInInches - amount to move in straight line
        // InputVariable : velocityInTPS - velocity value in motor tics per second - should be integer value
        // Input Variable : speedFactor - factor to control speed - this function will multiply the velocity with SpeedFactor
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double rotationsNeeded =  (distanceInInches / circumference);
        int driveTargetPosition = (int) (rotationsNeeded * encoderTickCount);
        LBdrive.setTargetPosition(driveTargetPosition);
        RBdrive.setTargetPosition(driveTargetPosition);
        LFdrive.setTargetPosition(driveTargetPosition);
        RFdrive.setTargetPosition(driveTargetPosition);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        newVelocity = velocityInTPS * speedFactor;
        ((DcMotorEx) LBdrive).setVelocity(newVelocity);
        ((DcMotorEx) RBdrive).setVelocity(newVelocity);
        ((DcMotorEx) LFdrive).setVelocity(newVelocity);
        ((DcMotorEx) RFdrive).setVelocity(newVelocity);
        while (LBdrive.isBusy() || RBdrive.isBusy() || LFdrive.isBusy() || RFdrive.isBusy()) {
            telemetry.addData("MOVING STRAIGHT", distanceInInches);
            telemetry.update();
        }
        LBdrive.setPower(0);
        RBdrive.setPower(0);
        LFdrive.setPower(0);
        RFdrive.setPower(0);
    }


}
