#include "robot.h"

#include "log.h"
#include "cluster.h"
#include "constants.h"
#include "frame_transform.h"

#include <cmath>
#include <unistd.h> // linux sleep
#include <optional>

#define DISP_IMG_WIDTH 1696
#define DISP_IMG_HEIGHT 960
#define SCALE_FACTOR 0.25

void addStateLabel(cv::Mat& image, const std::string & text) {
	// Define the text properties
	double fontScale = 2.0;
    int thickness = 2;
    int fontFace = cv::FONT_HERSHEY_DUPLEX ; 
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);

    // Calculate the position to center the text
    cv::Point textOrg(DISP_IMG_WIDTH/2 - textSize.width / 2, DISP_IMG_HEIGHT + 50 + textSize.height / 2);

    // Write the text on the image
	cv::Scalar color(255, 255, 255); // White color
    cv::putText(image, text, textOrg, fontFace, fontScale, color, thickness);
}

void addStateInfo(cv::Mat& image, const std::string & text) {
	// Define the text properties
	double fontScale = 1.5;
    int thickness = 2;
    int fontFace = cv::FONT_HERSHEY_DUPLEX ; 
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);

    // Calculate the position to center the text
    cv::Point textOrg(DISP_IMG_WIDTH/2 - textSize.width / 2, DISP_IMG_HEIGHT + 135 + textSize.height / 2);

    // Write the text on the image
	cv::Scalar color(255, 255, 255); // White color
    cv::putText(image, text, textOrg, fontFace, fontScale, color, thickness);
}

Robot::Robot(Camera & camera) 
    : camera(camera){
    
	initialize_log();
	i2c_bus_file = open_i2c(); 

	// initial scan image 
	cv::Mat state_img = cv::Mat::zeros(cv::Size(DISP_IMG_WIDTH, DISP_IMG_HEIGHT + 200), CV_8UC3);
	addStateLabel(state_img, std::string("INITIALIZED"));
	cv::Mat scan_img = cv::Mat::zeros(cv::Size(DISP_IMG_WIDTH, DISP_IMG_HEIGHT), CV_8UC3);
	cv::resize(state_img, state_img, cv::Size(), SCALE_FACTOR, SCALE_FACTOR);
	cv::imwrite("./display/state.jpg", state_img);
	cv::imwrite("./display/scan.jpg", scan_img);

	// arm starts in upright position
	armPosition = Point3D(0, 0, CARTESIAN_Z_MAX);
	debug_log("Initialized Robot");
}

Point2D Robot::getRobotPosition() {
	debug_log(std::string("Robot Position - x:") 
	+ std::to_string(robotPosition.x) 
	+ std::string(", y: ")
	+ std::to_string(robotPosition.y));
	return robotPosition;
}

Point3D Robot::getArmPosition() {
	debug_log(std::string("Arm Position - x:") 
	+ std::to_string(armPosition.x) 
	+ std::string(", y: ")
	+ std::to_string(armPosition.y)
	+ std::string(", z: ")
	+ std::to_string(armPosition.z));
	return armPosition;
}

// responsible for 
void Robot::setArmPosition(uint8_t motor_num, double pos) {
	pos = pos * 1000; // convert from [m] to [mm]
	uint16_t sat_pos;
	if (pos > 6535){
		sat_pos = 0xFFFF;
	} else {
		sat_pos = (uint16_t) (pos*10);
	}
	uint8_t data[3] = {0};
	data[0] = motor_num;
	data[1] = (sat_pos & 0xFF00) >> 8;
	data[2] = (sat_pos & 0xFF);
	write_i2c(i2c_bus_file, MCU_E, CMD_E_MOVE_AXIS, data, 3);
	sleep(1);
	log(std::string("Move stepper motor: ") + std::to_string(motor_num) +
		std::string("to position: ") + std::to_string(pos) + 
		std::string("by sending value: ") + std::to_string(data[1]) 
		+ std::string(" ") + std::to_string(data[2]) );

	uint8_t resp = 0xFF; // Not a valid resp
	// wait for done
	while (resp != S_E_ACTION_COMPLETE && resp != S_E_ACTION_ENDED_W_LIMIT) {
		read_i2c(i2c_bus_file, MCU_E, CMD_E_WRITE_STATUS, &resp, 1);
		sleep(1);
	}

	// interpret response
	if (resp == S_E_ACTION_COMPLETE) {
		// action done
		armPosition[motor_num] = pos / 1000; // convert back to m
		log(std::string("i2c: read S_E_ACTION_COMPLETE from MCU_E.\n"));
	} else if (resp == S_E_ACTION_ENDED_W_LIMIT) {
		if (pos > armPosition[motor_num]) { // hit fwd limit
			armPosition[motor_num] = limit_fwd[motor_num];
		} else {
			armPosition[motor_num] = limit_bkwd[motor_num];
		}
		log(std::string("i2c: read S_E_ACTION_ENDED_W_LIMIT from MCU_E.\n"));
	}
}

void Robot::driveForwards(uint8_t pwm_speed, float seconds) {
	// clear state image
	cv::Mat state_img = cv::Mat::zeros(cv::Size(DISP_IMG_WIDTH, DISP_IMG_HEIGHT + 200), CV_8UC3);
	addStateLabel(state_img, std::string("DRIVE"));
	cv::Mat scan_img = cv::Mat::zeros(cv::Size(DISP_IMG_WIDTH, DISP_IMG_HEIGHT), CV_8UC3);
	cv::resize(state_img, state_img, cv::Size(), SCALE_FACTOR, SCALE_FACTOR);
	cv::imwrite("./display/state.jpg", state_img);
	cv::imwrite("./display/scan.jpg", scan_img);

	uint8_t data[5] = {0};
	data[0] = pwm_speed;
	// get float as array
	uint8_t *float_array = reinterpret_cast<uint8_t*>(&seconds);
	data[1] = float_array[0];
	data[2] = float_array[1];
	data[3] = float_array[2];
	data[4] = float_array[3];
	write_i2c(i2c_bus_file, MCU_M, CMD_M_FWD, data, 5);
	sleep(1);
	log(std::string("Going forward (pwm speed: ") + std::to_string(pwm_speed) +
		std::string(", time: ") + std::to_string(seconds) + std::string(" [s]"));

	uint8_t resp = 0xFF; // Not a valid resp
	// wait for done
	while (resp != S_M_ACTION_COMPLETE) {
		read_i2c(i2c_bus_file, MCU_M, CMD_M_WRITE_STATUS, &resp, 1);
		if (resp == S_M_ACTION_COMPLETE) {
			// action done
			log(std::string("i2c: read S_M_ACTION_COMPLETE from MCU_M.\n"));
			return;
		}
		sleep(1);
	}
}

void Robot::driveBackwards(uint8_t pwm_speed, float seconds) {
	// clear state image
	cv::Mat state_img = cv::Mat::zeros(cv::Size(DISP_IMG_WIDTH, DISP_IMG_HEIGHT + 200), CV_8UC3);
	addStateLabel(state_img, std::string("DRIVE"));
	cv::Mat scan_img = cv::Mat::zeros(cv::Size(DISP_IMG_WIDTH, DISP_IMG_HEIGHT), CV_8UC3);
	cv::resize(state_img, state_img, cv::Size(), SCALE_FACTOR, SCALE_FACTOR);
	cv::imwrite("./display/state.jpg", state_img);
	cv::imwrite("./display/scan.jpg", scan_img);

	uint8_t data[5] = {0};
	data[0] = pwm_speed;
	// get float as array
	uint8_t *float_array = reinterpret_cast<uint8_t*>(&seconds);
	data[1] = float_array[0];
	data[2] = float_array[1];
	data[3] = float_array[2];
	data[4] = float_array[3];
	write_i2c(i2c_bus_file, MCU_M, CMD_M_BKWD, data, 5);
	sleep(1);
	log(std::string("Going backwards (pwm speed: ") + std::to_string(pwm_speed) +
		std::string(", time: ") + std::to_string(seconds) + std::string(" [s]"));

	uint8_t resp = 0xFF; // Not a valid resp
	// wait for done
	while (resp != S_M_ACTION_COMPLETE) {
		read_i2c(i2c_bus_file, MCU_M, CMD_M_WRITE_STATUS, &resp, 1);
		if (resp == S_M_ACTION_COMPLETE) {
			// action done
			log(std::string("i2c: read S_M_ACTION_COMPLETE from MCU_M.\n"));
			return;
		}
		sleep(1);
	}
}

void Robot::resetServoArm() {
	uint8_t data[1];
	write_i2c(i2c_bus_file, MCU_E, CMD_E_RESET, data, 0);
	usleep(10000);

	uint8_t resp = 0xFF; // Not a valid command

	// wait for done
	log(std::string("i2c: waiting for S_E_ACTION_COMPLETE from MCU_E.\n"));
	while(resp != S_E_ACTION_COMPLETE) {
		read_i2c(i2c_bus_file, MCU_E, CMD_E_WRITE_STATUS, &resp, 1);
		sleep(1);
	}
	log(std::string("i2c: read S_E_ACTION_COMPLETE from MCU_E.\n"));

	armPosition = Point3D(0, 0, CARTESIAN_Z_MAX);
}

void Robot::pollinate() { 
	uint8_t data[1];
	write_i2c(i2c_bus_file, MCU_E, CMD_E_POLLINATE, data, 0);
	usleep(10000);

	uint8_t resp = 0xFF; // Not a valid command

	// wait for done
	log(std::string("i2c: waiting for S_E_ACTION_COMPLETE from MCU_E.\n"));
	while(resp != S_E_ACTION_COMPLETE) {
		read_i2c(i2c_bus_file, MCU_E, CMD_E_WRITE_STATUS, &resp, 1);
		sleep(1);
	}
	log(std::string("i2c: read S_E_ACTION_COMPLETE from MCU_E.\n"));
}

std::vector<Point3D> Robot::scan() {
	std::vector<Point3D> flowersToVisit;
	std::vector<Point2D> scanLoc{
		Point2D(CARTESIAN_X_MAX/3, CARTESIAN_Y_MAX/3 - 0.05), 
		Point2D(2*CARTESIAN_X_MAX/3, CARTESIAN_Y_MAX/3 - 0.05), 
		Point2D(2*CARTESIAN_X_MAX/3, 2*CARTESIAN_Y_MAX/3 - 0.05), 
		Point2D(CARTESIAN_X_MAX/3, 2*CARTESIAN_Y_MAX/3 - 0.05)};
	for (int i = 0; i < 4; i++){
			log(std::string("Scanning location: ") 
				+ std::to_string( scanLoc.at(i).x )
				+ std::string(", ")
				+ std::to_string( scanLoc.at(i).y ));
			setArmPosition(x, scanLoc.at(i).x);
			setArmPosition(y, scanLoc.at(i).y);
			usleep(1000000);
			std::vector<Point3D> newFlowers = findFlowers(i);
			flowersToVisit.insert(flowersToVisit.end(), newFlowers.begin(), newFlowers.end());
	}
	for (auto flowerToVisit : flowersToVisit){
		log(std::string("flower: ") + std::to_string(flowerToVisit.x) + std::string(", ") + std::to_string(flowerToVisit.y)
		 + std::string(", ") + std::to_string(flowerToVisit.z));
	}
	sleep(2);
	return avgClusterCenters(flowersToVisit, 0.06);
}

void write_scan_image(cv::Mat & scanPlot, cv::Mat & image, int i, Point3D armPosition){
	int startX = 0;
	int startY = 0;
	switch(i){
		case 0: 
			startY = DISP_IMG_HEIGHT/2-1;
		break;
		case 1 :
			startX = DISP_IMG_WIDTH/2-1;
			startY = DISP_IMG_HEIGHT/2-1;
		break;
		case 2 : 
			startX = DISP_IMG_WIDTH/2-1;
		break;	
	}
	cv::Rect roi(startX, startY, DISP_IMG_WIDTH/2, DISP_IMG_HEIGHT/2);

    // Get the ROI in the larger image
    cv::Mat largerROIRect = scanPlot(roi);

    // Write the smaller image to the ROI in the larger image
	cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    image.copyTo(largerROIRect);
	cv::imwrite("./display/scan.jpg", scanPlot);

	cv::Mat state_img = cv::Mat::zeros(cv::Size(DISP_IMG_WIDTH, DISP_IMG_HEIGHT + 200), CV_8UC3);
	cv::Rect state_roi(0, 0, DISP_IMG_WIDTH, DISP_IMG_HEIGHT);
	cv::Mat state_largerROIRect = state_img(state_roi);
    scanPlot.copyTo(state_largerROIRect);
	addStateLabel(state_img, std::string("SCAN"));
	addStateInfo(state_img, std::string("Arm Position [mm] - x:") +
	std::to_string((int)(armPosition.x * 1000)) +
	std::string(", y: ") +
	std::to_string((int)(armPosition.y * 1000)) +
	std::string(", z: ") +
	std::to_string((int)(armPosition.z * 1000))) ;
	cv::resize(state_img, state_img, cv::Size(), SCALE_FACTOR, SCALE_FACTOR);
	cv::imwrite("./display/state.jpg", state_img);
}

std::vector<Point3D> Robot::findFlowers(int index){
	camera.storeSnapshot();
	cv::Mat image = camera.getColorImage();
	std::string tag = getFormattedTimeStamp();
	if (DEBUG) {
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
		cv::imwrite("./plots/" + tag + "_image.png", image);
		cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
	}
	std::vector<Point2D> flowerCenters = findFlowerCenters(image, tag);
	if (DEBUG) {
		int width = image.cols;
		int height = image.rows;
		for (Point2D const& blob : flowerCenters) {
			float x = blob.x/width;
			float y = blob.y/height;
			log(std::string("depth val: ") + std::to_string(camera.getDepthVal(x, y)));
		}
	}
	flowerCenters = avgClusterCenters(flowerCenters, 25);
	
	for (Point2D const& blob : flowerCenters) {
		cv::circle(image, cv::Point((int)blob.x, (int)blob.y), 20, { 255, 0, 255 }, 5);
	}
	cv::Mat scanPlot = cv::imread("./display/scan.jpg");
	write_scan_image(scanPlot, image, index, getArmPosition());
	
	std::vector<Point3D> cam3DPoints = camera.getDeprojection(flowerCenters);
	Point3D armPosition = getArmPosition();
	return camera2robot(cam3DPoints, armPosition.x, armPosition.y);
}

void update_pollinate(cv::Mat & scanPlot, int numFlowers, Point3D armPosition) {
	cv::Mat state_img = cv::Mat::zeros(cv::Size(DISP_IMG_WIDTH, DISP_IMG_HEIGHT + 200), CV_8UC3);
	cv::Rect state_roi(0, 0, DISP_IMG_WIDTH, DISP_IMG_HEIGHT);
	cv::Mat state_largerROIRect = state_img(state_roi);
    scanPlot.copyTo(state_largerROIRect);
	addStateLabel(state_img, std::string("POLLINATE"));
	addStateInfo(state_img, std::string("Number of Flowers: ") + 
	std::to_string(numFlowers ) + 
	std::string(" | Arm Position [mm] - x:") +
	std::to_string((int)armPosition.x * 1000) +
	std::string(", y: ") +
	std::to_string((int)armPosition.y * 1000) +
	std::string(", z: ") +
	std::to_string((int)armPosition.z * 1000)) ;
	cv::resize(state_img, state_img, cv::Size(), SCALE_FACTOR, SCALE_FACTOR);
	cv::imwrite("./display/state.jpg", state_img);
}

void Robot::pollinate_all_in_zone(std::vector<Point3D> flowerCenters) {
	cv::Mat scanPlot = cv::imread("./display/scan.jpg");
	update_pollinate(scanPlot, flowerCenters.size(), getArmPosition());
	for (auto const & flowerCenter : flowerCenters) {

		// check if point is within bounds
		if (flowerCenter.x > CARTESIAN_X_MAX || flowerCenter.x < CARTESIAN_X_MIN ||
			flowerCenter.y > CARTESIAN_Y_MAX || flowerCenter.y < CARTESIAN_Y_MIN ||
			flowerCenter.z > CARTESIAN_Z_MAX || flowerCenter.z < CARTESIAN_Z_MIN) {
			std::stringstream ss;
			ss << "INFO robot: ignoring out of bound flower at: (" 
				<< flowerCenter.x << ", " << flowerCenter.y << ", " << flowerCenter.x << ").";
			log(ss.str());
			continue;
		}

		log(std::string("move arm to point: ") 
		+ std::to_string(flowerCenter.x) + std::string(", ") 
		+ std::to_string(flowerCenter.y) + std::string(", ")
		+ std::to_string(flowerCenter.z));

		sleep(1);

		setArmPosition(x, flowerCenter.x);
		setArmPosition(y, flowerCenter.y);
		setArmPosition(z, flowerCenter.z);

		update_pollinate(scanPlot, flowerCenters.size(), getArmPosition());
		pollinate();

		sleep(1);
		// reset arm to top
		setArmPosition(z, CARTESIAN_Z_MAX + 0.1);

	}
}
