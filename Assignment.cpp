#include <stdio.h>
#include <stdlib.h>
#include "Aria.h"
#include <list>
#include <iostream>
#include <math.h>

ArRobot robot;
ArSensorReading *sonarReading;
ArLaser *myLaser;

double dist, angle;
int l = 200, r = 200, stage = 1;

ArGripper *myGripper = NULL;


FILE *fpData;

// The camera (Cannon VC-C4).
ArVCC4 vcc4(&robot);
// ACTS, for tracking blobs of color.
ArACTS_1_2 acts;

// Define a new class named Chase.
class Chase
{
public:
	// Constructor.
	Chase(ArRobot *robot, ArACTS_1_2 *acts);
	// Destructor.
	~Chase(void);
	// The chase action.
	void ChaseAction();
	// Height and width of pixels from frame-grabber.
	enum {
		WIDTH = 160,
		HEIGHT = 120
	};
protected:
	ArRobot *myRobot;
	ArACTS_1_2 *myActs;
	int myChannel;
};

// Constructor: Initialize the chase action.
Chase::Chase(ArRobot *robot, ArACTS_1_2 *acts)
{
	myRobot = robot;
	myActs = acts;
	myChannel = 1;
}

// Destructor.
Chase::~Chase(void) {}

// The chase action.
void Chase::ChaseAction()
{
	ArACTSBlob blob;
	ArACTSBlob largestBlob;
	int numberOfBlobs;
	int blobArea = 10;
	double xRel, yRel;
	bool flag = false;
	numberOfBlobs = myActs->getNumBlobs(myChannel);

	// Get largest blob.
	if (numberOfBlobs != 0)
	{
		for (int i = 0; i < numberOfBlobs; i++)
		{
			myActs->getBlob(myChannel, i + 1, &blob);
			if (blob.getArea() > blobArea)
			{
				flag = true;
				blobArea = blob.getArea();
				largestBlob = blob;
			}
		}
	}

	if (flag == true)
	{
		// Determine where the largest blob's center of gravity is relative to the center of the camera and adjust xRel.
		xRel = (double)(largestBlob.getXCG() - WIDTH / 2.0) / (double)WIDTH - 0.15;
		yRel = (double)(largestBlob.getYCG() - HEIGHT / 2.0) / (double)HEIGHT;
		// Set the heading and velocity for the robot.
		if (ArMath::fabs(xRel) < .1)
		{
			myRobot->setDeltaHeading(0);
		}
		else
		{
			if (ArMath::fabs(xRel) <= 1)
				myRobot->setDeltaHeading(-xRel * 5);
			else if (-xRel > 0)
				myRobot->setDeltaHeading(5);
			else
				myRobot->setDeltaHeading(-5);
		}
		myRobot->setVel(60);
	}
}

// Use the Chase class defined above to declare an object named chase.
Chase chase(&robot, &acts);

void update(void);
ArGlobalFunctor updateCB(&update);

void update(void)
{
	if (stage == 7) {
		chase.ChaseAction();
	}
	else {
		robot.setVel2(l, r); // Set the velocity of the wheels.
	}
	if (stage == 3)
		Sleep(1150), l = 200, r = 200, stage = 4;

	if (stage == 2)
		Sleep(500), r = 0, l = 0, stage = 3;
	
	sonarReading = robot.getSonarReading(3);
	if (sonarReading->getRange() < 1200 & stage == 1)
		r=-200, stage = 2;

	sonarReading = robot.getSonarReading(7);
	if (sonarReading->getRange() < 500 & stage == 4)
		stage = 5;
	
	sonarReading = robot.getSonarReading(4);
	if (sonarReading->getRange() < 2000 & stage == 5)
		r = 111, stage = 6;

	sonarReading = robot.getSonarReading(0);
	if (sonarReading->getRange() < 500 & stage == 6)
		r = 200, myGripper->gripOpen(), stage = 7;

	if (stage == 7 & myGripper->getBreakBeamState() == 3) {
		chase.~Chase(), robot.setVel2(0, 0), stage = 8, myGripper->gripClose();
		Sleep(400);
		myGripper->gripStop();
		r = 150, l = 100;
	}
	sonarReading = robot.getSonarReading(4);
	if (sonarReading->getRange() < 500 & stage == 8) 
		r = 0, l = 0, myGripper->gripOpen(), stage = 9;

	if (stage == 9)
		r = -200, l = -200, stage = 10;

	sonarReading = robot.getSonarReading(4);
	if (sonarReading->getRange() > 1000 & stage == 10)
		r = 0, l = 0, myGripper->gripClose();

	// Open a file to store sonar, laser and odometry readings.
	fpData = fopen("Data.txt", "a");
	if (fpData == NULL)
		std::cout << "File cannot be opened." << std::endl;

	// Get positional data
	fprintf(fpData, "%.2f,%.2f,", robot.getX(), robot.getY());

	// Get sonar reading.
	for (int i = 0; i<8; i++)
	{
		sonarReading = robot.getSonarReading(i);
		double xp = sonarReading->getRange()*cos(sonarReading->getSensorTh()*3.14 / 180) + sonarReading->getSensorX();
		double yp = sonarReading->getRange()*sin(sonarReading->getSensorTh()*3.14 / 180) + sonarReading->getSensorY();
		double xr = xp*cos(robot.getTh()*3.14 / 180) - yp*sin(robot.getTh()*3.14 / 180);
		double yr = xp*sin(robot.getTh()*3.14 / 180) + yp*cos(robot.getTh()*3.14 / 180);
		double x = xr + robot.getX();
		double y = yr + robot.getY();
		if (y < 0)
			y = 0;
		if (y > 4300)
			y = 4300;
		if (x < 0)
			x = 0;
		if (x > 4300)
			x = 4300;
		fprintf(fpData, "%.2f,%.2f,", x, y);
	}

	// Get laser reading.
	int angle1 = -90;
	int angle2 = -85;
	for (int i = 0; i<12; i++)
	{
		dist = myLaser->currentReadingPolar(angle1, angle2, &angle);
		double xp = dist*cos(angle*3.14 / 180);
		double yp = dist*sin(angle*3.14 / 180);
		double xr = xp*cos(robot.getTh()*3.14 / 180) - yp*sin(robot.getTh()*3.14 / 180);
		double yr = xp*sin(robot.getTh()*3.14 / 180) + yp*cos(robot.getTh()*3.14 / 180);
		double x = xr + robot.getX();
		double y = yr + robot.getY();
		if (y > -100 & x > -100) {
			fprintf(fpData, "%.2f,%.2f,", x, y);
		}
		else {
			fprintf(fpData, ",,");
		}
		angle1 = angle1 + 15;
		angle2 = angle2 + 15;
	}

	//Get speed data
	fprintf(fpData, "%.2f,%.2f,", robot.getLeftVel(), robot.getRightVel());

	fprintf(fpData, "\n");

	fclose(fpData);	// Close the file.
}

int main(int argc, char **argv)
{
	// Initialisation
	Aria::init();

	// Open a connection to ACTS.
	acts.openPort(&robot);
	// Initialize the camera.
	vcc4.init();
	// Wait for a little while.
	ArUtil::sleep(2000);

	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();

	ArRobotConnector robotConnector(&argParser, &robot);
	ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);

	// Always try to connect to the first laser:
	argParser.addDefaultArgument("-connectLaser");

	if (!robotConnector.connectRobot())
	{
		ArLog::log(ArLog::Terse, "Could not connect to the robot.");
		if (argParser.checkHelpAndWarnUnparsed())
		{
			// -help not given, just exit.
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	// Trigger argument parsing
	if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
	}

	myGripper = new ArGripper(&robot);
	myGripper->gripClose();

	// Add sonar.
	ArSonarDevice sonar;
	robot.addRangeDevice(&sonar);

	// Connect laser.
	if (!laserConnector.connectLasers())
	{
		ArLog::log(ArLog::Terse, "Could not connect to configured laser.");
		Aria::logOptions();
		Aria::exit(1);
	}
	myLaser = robot.findLaser(1);

	ArPose space(3800, 3500, 270); // Initial robot's odometry.
	robot.moveTo(space); //Moves the robot's idea of its position to this position.

						 // Tilt the camera down 45 degrees to make it find the ball easier.
	vcc4.tilt(-45);
	ArUtil::sleep(1000);

						 // turn on the motors, turn off amigobot sounds
	robot.enableMotors();

	robot.addUserTask("update", 50, &updateCB);
	//robot.setCycleTime(100);
	robot.runAsync(true);
	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();

	Aria::exit(0);
}