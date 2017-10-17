/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometrylab;

import java.util.LinkedList;

import javax.swing.plaf.ColorUIResource;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class OdometryCorrection extends Thread {
	
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	//lastest adding
	EV3ColorSensor colorSensor=new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	SampleProvider colorRGBSensor = colorSensor.getRedMode();
	float [] sample = new float[colorRGBSensor.sampleSize()];
	private LinkedList<Float> recent = new LinkedList<Float>();
	//----------------------------------------------------

	//added from test
	//difference from yellow space that needed to be identiied as a line
	private static int LINE_VALUE=20;
	int counter =0;

	//count that robot cross two lines
	private int xLine=0;
	private int yLine=0;

	//offset of light sensor 
	private static final double SENSOR_OFFSET=6.8; //distance from wheel to sensor

	//variables of the color value
	private double baseline=0;
	private double calibT=0;//change value to 0

	boolean calibrated;
	boolean lineReset;

	// constructor
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;
		sample[0]=0;
		lineReset=true;
		calibrated=false;
	}

	// run method (required for Thread)
	public void run() {
		
		long correctionStart, correctionEnd;
		
		while (true) {
			
			correctionStart = System.currentTimeMillis();
			colorRGBSensor.fetchSample(sample, 0);
			
			/* first collect 20 datas that identifies the yellow space other than black line
			 * set calibrated to true if 20 datas collected
			 */
			if(!calibrated)
			{
				//have not collected 20 datas yet
				if(counter<20)
				{
					//add most recent value to sample
					calibT +=sample[0]*100;
					counter++;
				}

				else {
					
					//20 datas collected,baseline as indication of yellow field other than black line
					baseline=calibT/20;
					calibrated=true;
				}
			}
			//if sample list is not full, add sample to list
			if(recent.size()<2)
			{
				recent.addLast(sample[0]*100);
			}
			//if list is full, remove first element and add latest sample to last position
			else {
				recent.removeFirst();
				recent.addLast(sample[0]*100);
			}

			//if sample average is close to when robots start, no black line detected, reset line detection.
			if(Math.abs(baseline-getAverage(recent))<10) {
				lineReset=true;
			}
			
			
			else if(Math.abs(baseline-getAverage(recent))> LINE_VALUE && baseline !=0 && lineReset)
			{
				//does sound when line is detected
				LocalEV3.get().getAudio().systemSound(0);

				//current angle of robot in degree, get theta method return an angle in degree.
				Double theta = odometer.getTheta() ;
				
				//avoid line detection multiple times
				lineReset=false;
				
				//detecting
				if(theta %180 <15 || theta%180>165)
				{//facing y direction
					correctY(theta);
				
				}
				else {
					//facing x direction
					correctX(theta);
				}
			}
			

			//TODO Place correction implementation here

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}
//a method to correct y
	public void correctY(double theta) {
		
		if(theta % 360 < 25 || theta % 360 >335)
		{//face increasing y direction, increasing counter because black line detexted
			yLine++;
			
			//when first detect black line, set that line to y=0, actual position of robot should be 0-sensoroffset
			if(yLine==1) {
				
				odometer.setY(-SENSOR_OFFSET);
			}
			else{
				//other cases, set y position to calculated value
				odometer.setY((yLine-1) * 30 - SENSOR_OFFSET); 
			}
			}
	
		else {
			//facing decreasing y direction, when yline=1, sensor is at y=0, so actual position of robot is at sensor offset
			if(yLine==1) {
				odometer.setY(+SENSOR_OFFSET);
				}
			else
				//other cases, calculate right position
			{odometer.setY((yLine-1)* 30 + SENSOR_OFFSET); //y*30-15+offset
			}
			yLine--;
		}
		}
	
//this is a method to correct x
	public void correctX(double theta) {
		if(theta % 360 > 70 && theta % 360 < 110)
		{
		//facing increasing x direction, first time detect black line, set that line to be x=0
			xLine++;
			if(xLine==1) {
				odometer.setX(-SENSOR_OFFSET);
			}
			//other case, calculate
			else {
				odometer.setX((xLine-1)*30-SENSOR_OFFSET);}
		}
		else {
			//facing decreasing x direction, counter=0,when xline=0, it reaches x=0.
			if(xLine==0) {
				odometer.setX(SENSOR_OFFSET);
			}
			else {
			odometer.setX((xLine-1)*30 +SENSOR_OFFSET);
			}
			xLine--;
		}
	}

	/* this method return a double of the average for the values collected
	 * this permits to have a smoother curve */
	public double getAverage(LinkedList<Float> value) {
		double averageValue=0;
		if(value.isEmpty()) {
			averageValue=0;
		}
		else {
			for(Float i : value)
			{
				averageValue=averageValue+i;
			}
			averageValue=averageValue/(value.size());
		}
		return averageValue;
	}
}
