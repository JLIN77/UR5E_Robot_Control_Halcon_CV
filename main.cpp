// 
//Discription: opencv & halcon
//Createy by Lin
//Update: 21,Sept,2020 - update staticXForce,staticYForce,staticZForce online
//        
//Functions: #1 read a Halcon template(with a binary form)
//           #2 opencv grab the realtime frame and transform it to Halcon form
//           #3 Halcon matches the object in the frame
//           #4 speedl is useed
//           #5 force feedbacks are used in x-direction,ydirection and z-direction
//        
//Issue: #1 with the same up and down time, the up-distance is shorter than down-distance
//       #2 slight virbration in static position, mainly caused by the matching method (which may be solved with a accuracy template-making or a more stable platform or a circle template) 
//       #3 the supposed tool board is not fixed
//       #4 the measured the staticXForce, staticYForce, staticZForce are quite variable.(fixed by updating the 3 values online)
//       #5 when transform the Frame to BinaryFrame, it will easy lose the object(unstable)

//

#include <iostream>
#include<opencv2/opencv.hpp>
#include<algorithm>
#include <fstream>

#include "windows.h"

#include "robot_data.h"
#include "robot_control.h"
#include "ur_robot_adapter.h"
#include "PIDControl.h"
//#include "fanuc_robot_adapter.h"
#include "platform/platform.h"
#include "CUR_Control.h"

#include "ImageProcessing.h"
#include "Halconcpp.h"


using namespace HalconCpp;

using namespace std;
using namespace cv;

Mat Frame;
Mat FrameBinary;

Point point1, point2;
Point ObjectCenter = Point(0, 0);

SYSTEMTIME t;

// vector of TCPForce
vector<double> TCPForce;
double XForce = 0;
double YForce = 0;
double ZForce = 0;

// ur_control::CUR_Control URRobotObj;
hrobot::URRobotAdapter* robot_control_ptr = new hrobot::URRobotAdapter();


double GetTickCountA()
{
	__int64 Freq = 0;
	__int64 Count = 0;
	if (QueryPerformanceFrequency((LARGE_INTEGER*)&Freq)
		&& Freq > 0
		&& QueryPerformanceCounter((LARGE_INTEGER*)&Count))
	{
		//乘以1000，把秒化为毫秒
		return (double)Count / (double)Freq * 1000.0;
	}
	return 0.0;
}



HObject Mat2HObject(const cv::Mat& image)
{
	HObject Hobj = HObject();
	int hgt = image.rows;
	int wid = image.cols;
	int i;
	//  CV_8UC3  
	if (image.type() == CV_8UC3)
	{
		vector<cv::Mat> imgchannel;
		split(image, imgchannel);
		cv::Mat imgB = imgchannel[0];
		cv::Mat imgG = imgchannel[1];
		cv::Mat imgR = imgchannel[2];
		uchar* dataR = new uchar[hgt * wid];
		uchar* dataG = new uchar[hgt * wid];
		uchar* dataB = new uchar[hgt * wid];
		for (i = 0; i < hgt; i++)
		{
			memcpy(dataR + wid * i, imgR.data + imgR.step * i, wid);
			memcpy(dataG + wid * i, imgG.data + imgG.step * i, wid);
			memcpy(dataB + wid * i, imgB.data + imgB.step * i, wid);
		}
		GenImage3(&Hobj, "byte", wid, hgt, (Hlong)dataR, (Hlong)dataG, (Hlong)dataB);
		delete[]dataR;
		delete[]dataG;
		delete[]dataB;
		dataR = NULL;
		dataG = NULL;
		dataB = NULL;
	}
	//  CV_8UCU1  
	else if (image.type() == CV_8UC1)
	{
		uchar* data = new uchar[hgt * wid];
		for (i = 0; i < hgt; i++)
			memcpy(data + wid * i, image.data + image.step * i, wid);
		GenImage1(&Hobj, "byte", wid, hgt, (Hlong)data);
		delete[] data;
		data = NULL;
	}
	return Hobj;
}

Mat threshold_demo(Mat Frame) {
	Mat binary, gray;
	cvtColor(Frame, gray, COLOR_RGB2GRAY);
	threshold(gray, binary, 0, 255, THRESH_BINARY | THRESH_TRIANGLE);
	//imshow("binary", binary);
	return binary;
}


vector<double> GetTCPForceThread()
{
	vector<double> tcpForce;
	double xForce = 0;
	double yForce = 0;
	double zForce = 0;
	for (int i = 0; i < 5;i++)
	{
		TCPForce = robot_control_ptr->robot_control_.m_pUrDiver->m_pRt_Interface->m_pRobotState->getTcpForce();
		xForce += TCPForce[0];
		yForce += TCPForce[1];
		zForce += TCPForce[2];
		Sleep(2);
	}
	xForce = xForce / 5;
	yForce = yForce / 5;
	zForce = zForce / 5;

	tcpForce.push_back(xForce);
	tcpForce.push_back(yForce);
	tcpForce.push_back(zForce);

	return tcpForce;
}

int main() {

	// Create a txt
	fstream RobotCoordinate("C:/Users/Administrator/Desktop/Robot Coordinate.txt", ios::out);
	RobotCoordinate << "Time                     Robot_x        Robot_y        Center_x        Center_y        Matching time" << endl;

	PIDController PID;

	// define: 
	// the up-position, down-position 
	// down-speed, up-speed
	// down-time, up-time
	// direction: only the forward direction, we do the down and up operation
	double downPositionX = 0.1755;
	double downPositionY = 0.3592;
	double upPositionX = 0.5717;
	double upPositionY = 0.0023;
	double postionErr = 0.01;

	double downSpeedZ = -0.005;
	double upSpeedZ = 0.005;

	double downTime = 2000;
	double followTime = 6000;
	double upTime = 2350;

	double PrePositionX = 0.0686;
	double PrePositionY = 0.4243;
	bool forwardFlag = false;

	



	// static force of X,Y,Z-direction. It is highly recommended that you should remeasure the staticXForce,staticXForce and staticZForce once you 
	// restart the project, mainly because the measured 3 values are quite different.
	double staticXForce = 0;
	double staticYForce = 0;
	double staticZForce = 0;

	double XForceThreshold = 3;
	double YForceThreshold = 3;
	double ZForceThreshold = 5;

	double XForceErr = 0;
	double YForceErr = 0;
	double ZForceErr = 0;

	// The reason that P_XForce, P_YForce bigger than P_ZForce is we want the XForce and YForce control 
	double P_XForce = 0.003;
	double P_YForce = 0.003;
	double P_ZForce = 0.002;
	
	
	// Initialize Robot 
	hrobot::ToolPosition current_position;
	ur_pose::ToolPosition temp_position;
	ur_pose::ToolPosition temp_speed;

	
	//robot_control_ptr->robot_contorl_ = &URRobotObj;
	if (!robot_control_ptr->StartRobot()) {
		std::cout << "failed to connected to robot!" << std::endl;
		delete robot_control_ptr;
		return 1;
	}

	Sleep(1000);
	current_position = robot_control_ptr->GetRobotTCPPosition();

	std::cout << "x = " << current_position.x_ << " "
		<< "y = " << current_position.y_ << " "
		<< "z = " << current_position.z_ << std::endl;
	std::cout << "rx = " << current_position.rx_ << " "
		<< "ry = " << current_position.ry_ << " "
		<< "rz = " << current_position.rz_ << std::endl;

	if (current_position.x_ == 0 && current_position.y_ == 0 && current_position.z_ == 0)
	{
		std::cout << "failed to connected to robot!" << std::endl;
		delete robot_control_ptr;
		return 0;
	}

    //update staticXForce,staticYForce,staticZForce online
	for (int i = 0;i < 10; i++) {
		TCPForce = robot_control_ptr->robot_control_.m_pUrDiver->m_pRt_Interface->m_pRobotState->getTcpForce();
		staticXForce += TCPForce[0];
		staticYForce += TCPForce[1];
		staticZForce += TCPForce[2];
		Sleep(2);   // UR5E update time 2s
	}
	staticXForce = staticXForce / 10;
	staticYForce = staticYForce / 10;
	staticZForce = staticZForce / 10;
	cout << "staticXForce  staticYForce  staticZForce  " << staticXForce << " " << staticYForce << " " << staticZForce << endl;
	

	// Step 1 ---------------------------------------------- Halcon read template

	// Local iconic variables
	HObject  ho_Image, ho_ModelRegion, ho_TemplateImage;
	HObject  ho_ModelContours, ho_TransContours;

	// Local control variables
	HTuple  hv_AcqHandle, hv_ModelID, hv_ModelRegionArea;
	HTuple  hv_RefRow, hv_RefColumn, hv_HomMat2D, hv_Row, hv_Column;
	HTuple  hv_Angle, hv_Score, hv_I, hv_Scale, hv_ScaleRow, hv_ScaleColumn;
	HTuple  width0, height0, width1, height1, width2, height2, width3, height3;
	HTuple  windowid0, windowid1, windowid2, windowid3, windowid4;
	HTuple hv_UsedThreshold;
	int matching_number;
	double ref_row = 512;
	double ref_column = 640;
	double current_row;
	double current_column;


	int found_counter = 0;
	
	double time_tick00, time_tick0, time_tick1, time_tick2, time_tick3, time_tick4, time_tick5, time_tick6;
	double time_tick7 = 0;
	double time_tick10 = 0;
	double time_tick11 = 0;
	double matching_time;
	double dt1=0, dt2=0, dt = 0;
	double delta_robot_x = 0;
	double delta_robot_y = 0;
	double delta_robot_z = 0;
	
	

	//
 //Matching 01: ************************************************
 //Matching 01: BEGIN of generated code for model initialization
 //Matching 01: ************************************************

   //
  //Matching 01: Obtain the model image
	ReadImage(&ho_Image, "C:/Users/Administrator/Desktop/pic0.tif");
	//
	//Matching 01: Build the ROI from basic regions
	GenCircle(&ho_ModelRegion, 428, 716, 154.175);
	//
	//Matching 01: Reduce the model template
	ReduceDomain(ho_Image, ho_ModelRegion, &ho_TemplateImage);
	//
	//Matching 01: Create the shape model
	CreateShapeModel(ho_TemplateImage, 7, HTuple(0).TupleRad(), HTuple(10).TupleRad(),
		HTuple(0.7912).TupleRad(), (HTuple("none").Append("no_pregeneration")), "use_polarity",
		((HTuple(54).Append(56)).Append(8)), 4, &hv_ModelID);
	//
	//Matching 01: Get the model contour for transforming it later into the image
	GetShapeModelContours(&ho_ModelContours, hv_ModelID, 1);
	//
	//Matching 01: Get the reference position
	AreaCenter(ho_ModelRegion, &hv_ModelRegionArea, &hv_RefRow, &hv_RefColumn);
	VectorAngleToRigid(0, 0, 0, hv_RefRow, hv_RefColumn, 0, &hv_HomMat2D);
	AffineTransContourXld(ho_ModelContours, &ho_TransContours, hv_HomMat2D);
	//
	////Matching 01: Display the model contours
	//if (HDevWindowStack::IsOpen())
	//	DispObj(ho_Image, HDevWindowStack::GetActive());
	//if (HDevWindowStack::IsOpen())
	//	SetColor(HDevWindowStack::GetActive(), "green");
	//if (HDevWindowStack::IsOpen())
	//	SetDraw(HDevWindowStack::GetActive(), "margin");
	//if (HDevWindowStack::IsOpen())
	//	DispObj(ho_ModelRegion, HDevWindowStack::GetActive());
	//if (HDevWindowStack::IsOpen())
	//	DispObj(ho_TransContours, HDevWindowStack::GetActive());

	// Step 2 ------------------------------------------------  opencv read camera, Halcon match
	

	VideoCapture capture(0);
	capture.set(CAP_PROP_FPS, 90);
	capture.set(CAP_PROP_FRAME_WIDTH, 1280.0);
	capture.set(CAP_PROP_FRAME_HEIGHT, 1024.0);
	//capture.set(CAP_PROP_HUE, 50);//色调 50
	//capture.set(CAP_PROP_CONTRAST, 40);//对比度 40
	//capture.set(CAP_PROP_SATURATION, 100);//饱和度 50

	////HObject show
	//
	//OpenWindow(0, 0, width0, height0, 0, "", "", &windowid1);
	//HDevWindowStack::Push(windowid1);
	//DispObj(ho_Image, HDevWindowStack::GetActive());
	//waitKey(1);

	time_tick0 = GetTickCountA();
	time_tick00 = GetTickCountA();
	while (capture.isOpened()) {
		
		//opencv grab image
		time_tick1 = GetTickCountA();
		capture >> Frame;
		//waitKey(1);

		double time_tick2 = GetTickCountA();
		//Frame = threshold_demo(Frame);
		//waitKey(5);
		
		//imshow("frame", Frame);
		//waitKey(1);
		time_tick3 = GetTickCountA();
		//cout << "show time" << time_tick3 - time_tick2 << endl;

		////help to create a Binary Template
		//imwrite("0.jpg", Frame);
		//waitKey(10000);

		//Mat -> HObject
		ho_Image = Mat2HObject(Frame);
		time_tick4 = GetTickCountA();
		cout << "opencv to Halcon time:" << time_tick4 - time_tick3 << endl;
		
		//Matching 01: Find the model
		FindShapeModel(ho_Image, hv_ModelID, HTuple(0).TupleRad(), HTuple(10).TupleRad(),
			0.5, 0, 0.5, "least_squares", (HTuple(7).Append(1)), 0.75, &hv_Row, &hv_Column,
			&hv_Angle, &hv_Score);
		cout << "hv_Score" << hv_Score[0].D() << endl;

		time_tick5 = GetTickCountA();
		matching_time = time_tick5 - time_tick4;
		cout << "Halcon matching time:" << matching_time << endl;
		

		
		matching_number = hv_Score.TupleLength().I();

		time_tick6 = GetTickCountA();
		if (matching_number >= 1)
		{
		
			cout << "index: " << ++found_counter << "  time tick before and after: " << time_tick6 - time_tick7 << endl;
			time_tick7 = GetTickCountA();

			current_row = hv_Row[0].D();
			current_column = hv_Column[0].D();


			//// method 2 : P Control
			//current_position = robot_control_ptr->GetRobotTCPPosition();
			//PID.InputParameters(current_column, current_row, dt);
			//delta_robot_x = PID.PControl_X();
			//delta_robot_y = PID.PControl_Y();
			//cout << "P CONTROL!" << endl;

			 //method 3 : P ControlPI Control
	         //wait for resetting 2s

			cout << "center point pixel:" << current_column <<"," << current_row << endl;
			
			//initialize the robot position
			if(time_tick2 - time_tick00 < 2000)  
			{
				cout << "P CONTROL!" << endl;
				PID.InputParameters(current_column, current_row, dt);
				delta_robot_x = PID.PControl_X();
				delta_robot_y = PID.PControl_Y();

				current_position = robot_control_ptr->GetRobotTCPPosition();
				temp_position.x = current_position.x_;
				temp_position.y = current_position.y_;
				temp_position.z = 0.2728;

				temp_position.rx = current_position.rx_;
				temp_position.ry = current_position.ry_;
				temp_position.rz = current_position.rz_;
				robot_control_ptr->robot_control_.m_cArmCtrlobj.ServoJMoveToTCP(temp_position, 0.05, 0.03, 800);
				//Sleep(100);
				cout << "current_position.z_" << current_position.z_ << endl;

				dt1 = GetTickCountA();
			}
			else
			{
				cout << "PI CONTROL!" << endl;
				dt2 = GetTickCountA();
				dt = (dt2 - dt1)/1000;
				cout << "--dt--" << dt << endl;
				//dt = 1;
				PID.InputParameters(current_column, current_row, dt);
				delta_robot_x = PID.PIControl_X();
				delta_robot_y = PID.PIControl_Y();
				dt1 = dt2;
			}
			//if (delta_robot_x <= 0.0005)
			//{
			//	delta_robot_x = 0;
			//}
			//if (delta_robot_y <= 0.0005)
			//{
			//	delta_robot_y = 0;
			//}

			// forward way or backward way
			current_position = robot_control_ptr->GetRobotTCPPosition();
			if (current_position.x_ - PrePositionX > 0)
			{
				forwardFlag = true;
			}
			else
			{
				forwardFlag = false;
			}
			PrePositionX = current_position.x_;

			//tool down, only the forward way
			if (fabs(current_position.x_ - downPositionX)<postionErr && fabs(current_position.y_ - downPositionY) < postionErr && forwardFlag)
			{
				delta_robot_z = downSpeedZ;
				time_tick10 = GetTickCountA();
				cout << "down triggered ------------------------------ " << endl;
			}
			
			time_tick11 = GetTickCountA();
			//tool down follow
			if (time_tick11 - time_tick10 > downTime && time_tick11 - time_tick10 < (downTime + followTime))
			{
				delta_robot_z = 0;
				cout << "down following ------------------------------ " << endl;
			}
			//tool up
			else if(time_tick11 - time_tick10 > (downTime + followTime) && time_tick11 - time_tick10 < (downTime + followTime + upTime))
			{
				delta_robot_z = upSpeedZ;
				cout << "up triggered ------------------------------ " << endl;
			}
			//tool follow
			else
			{
				if(delta_robot_z != downSpeedZ)
				    delta_robot_z = 0;
			}
			
	
			cout << "center point xy:" << current_position.x_ << "," << current_position.y_ << endl;
			//current_position.x_ += delta_robot_x;
			//current_position.y_ += delta_robot_y;

			//temp_position.x = current_position.x_;
			//temp_position.y = current_position.y_;
			//temp_position.z = 0.23676;
			//
			//temp_position.rx = current_position.rx_;
			//temp_position.ry = current_position.ry_;
			//temp_position.rz = current_position.rz_;

			cout << "v_x-" << "v_y-" << "v_z   " << delta_robot_x << delta_robot_y << delta_robot_z << endl;
			 temp_speed.x = delta_robot_x;
			 temp_speed.y = delta_robot_y;
			 temp_speed.z = delta_robot_z;
			
			 temp_speed.rx = 0;
			 temp_speed.ry = 0;
			 temp_speed.rz = 0;


			////get TCP Force
			TCPForce = robot_control_ptr->robot_control_.m_pUrDiver->m_pRt_Interface->m_pRobotState->getTcpForce();
			//return TCPForce
			XForceErr = TCPForce[0] - staticXForce;
			YForceErr = TCPForce[1] - staticYForce;
			ZForceErr = TCPForce[2] - staticZForce;
			cout << " XForceErr " << XForceErr << " YForceErr " << YForceErr << " ZForceErr " << ZForceErr << endl;
			//X-Direction
			if (XForceErr > XForceThreshold)
			{
				 temp_speed.x += P_XForce * XForceErr;
			}
			else if (XForceErr < -XForceThreshold)
			{
				 temp_speed.x += P_XForce * XForceErr;
			}
			else
			{
				 temp_speed.x =  temp_speed.x;
			}
			//Y-Direction
			if (YForceErr > YForceThreshold)
			{
				 temp_speed.y += P_YForce * YForceErr;
			}
			else if (YForceErr < -YForceThreshold)
			{
				 temp_speed.y += P_YForce * YForceErr;
			}
			else
			{
				 temp_speed.y =  temp_speed.y;
			}
			//Z-Direction
			if (ZForceErr > ZForceThreshold)
			{
				 temp_speed.z += P_ZForce * ZForceErr;
			}
			else if(ZForceErr < -ZForceThreshold)
			{
				 temp_speed.z += P_ZForce * ZForceErr;
			}
			else
			{ 
				 temp_speed.z =  temp_speed.z;
			}

			

			//robot_control_ptr->robot_control_.m_cArmCtrlobj.ServoJMoveToTCP(temp_position, 0.3, 0.05, 200);
			//robot_control_ptr->robot_control_.m_cArmCtrlobj.ToolMove(temp_position, 20, 0.5);
			//robot_control_ptr->robot_control_.m_cArmCtrlobj.ServoJMoveToTCP(temp_position, 0.1, 0.03, 300);   // basic (temp_position, 0.15, 0.03, 500)
			robot_control_ptr->robot_control_.m_cArmCtrlobj.Speedl(temp_speed, 1, 1);   //(temp_speed, 0.3, 1)			
			
			
		/*	GetLocalTime(&t);
			cout << setiosflags(ios::fixed) << setprecision(4);
			RobotCoordinate << t.wYear << "-" << t.wMonth << "-" << t.wDay << "-" << t.wHour << "-"
				<< t.wMinute << "-" << t.wSecond << "-" << t.wMilliseconds << "      " <<
				temp_position.x << "      " << temp_position.y << "      " << delta_robot_x << "      " << delta_robot_y << "      " << current_column << "      " <<
				current_row << "      " << matching_time << endl;*/

		}
		else {
			 temp_speed.x = 0;
			 temp_speed.y = 0;
			 temp_speed.z = 0;

			 temp_speed.rx = 0;
			 temp_speed.ry = 0;
			 temp_speed.rz = 0;
			robot_control_ptr->robot_control_.m_cArmCtrlobj.Speedl(temp_speed, 1, 1);
			cout << "failed to match the object!" << endl;
		}
	}
	capture.release();
	destroyAllWindows();

	robot_control_ptr->StopRobot();
	delete robot_control_ptr;

	RobotCoordinate.close();
	return 0;

}

