package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;
	private int filterControl;
	private static final int FILTER_OUT = 15;

	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)
		this.distance = distance;

		// ERROR = measured distance - controlled distance
		int ERROR=distance-bandCenter;

		/* if the difference between the distance measured and the distance set is a negative value
		 * meaning it is too close to the wall
		 * if too close to the wall, right(outer wheel) faster, left (inner) wheel goes slower */
		if (ERROR<-bandwidth) {
			filterControl = 0;

			/*if too close to the wall, less than the distance set, 
			meaning the robot will not be able to make the turn without hitting the wall
			set robot to go backwards in a straight line, and when distance is large enough, will go forward */
			if(distance<=14) {
				WallFollowingLab.rightMotor.setSpeed(motorHigh);
				WallFollowingLab.leftMotor.setSpeed(motorHigh);
				WallFollowingLab.rightMotor.backward();
				WallFollowingLab.leftMotor.backward();
			}

			else {
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
				/*the rightMotor has been set to a speed of motorHigh+motorLow, where motorLow could have been any constant
				 * this is to counter the fact that the right side of the robot is heavier, therefore the right side has more adherence
				 * which is why it will speed faster to make a quicker turn */
				WallFollowingLab.rightMotor.setSpeed(motorHigh+motorLow);
				WallFollowingLab.leftMotor.setSpeed(motorLow);
			}
		}

		/* if the difference between the distance measured and the distance set is a negative value
		 * if too far from the wall, speed up the outer wheel, and lower inner wheel*/
		else if (ERROR>bandwidth) {
			filterControl++;
			if(filterControl>FILTER_OUT) {

				/* condition that if the robot is too far from the wall, it has to go forward
				 * this prevents the robot to follow the wall in reverse mode */
				if(distance>30) {
					WallFollowingLab.rightMotor.forward();
					WallFollowingLab.leftMotor.forward();
				}

				WallFollowingLab.rightMotor.setSpeed(motorLow);
				WallFollowingLab.leftMotor.setSpeed(motorHigh);
			}
		}

		//    On correct heading and moves forward
		else {
			filterControl=0;
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.setSpeed(motorHigh);
		}

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
