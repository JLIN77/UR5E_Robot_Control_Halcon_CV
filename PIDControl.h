#pragma once

#include <iostream>
#include <math.h>

#define TARGET_COL 640;
#define TARGET_ROW 512;

//class PIDController
//{
//public:
//	double P = 0,I = 0, D = 0;
//	float Integ = 0;
//	float Deriv = 0;
//	float Target, Measure;
//	float Error, PreError;
//	float dt;
//	float Output;
//
//	PIDController(double p_in, double i_in, double d_in) {
//		P = p_in;
//		I = i_in;
//		D = d_in;
//	};
//
//	virtual void GetParameter(double tarX, double meaX, double dt_in) {};
//	virtual double PControl() { return Output; };
//	virtual double PDControl() { return Output; };
//
//	~PIDController() {};
//	
//private:
//
//};
//
//
//
//class PIDControllerX : public PIDController
//{
//public:
//	PIDControllerX(double p_in, double i_in, double d_in) : PIDController(p_in,i_in,d_in) {};
//	void GetParameter(double tarX, double meaX, double dt_in);
//	double PDControl();
//
//};
//
//void PIDControllerX::GetParameter(double tarX, double meaX, double dt_in)
//{
//	Target = tarX;
//	Measure = meaX;
//	dt = dt_in;
//}
//
//double PIDControllerX::PDControl()
//{
//	Error = Target - Measure;
//	Deriv = (Error - PreError) / dt;
//	Output = P * Error + D * Deriv;
//
//	PreError = Error;
//	return Output;
//}
//
//
//
//class PIDControllerY : public PIDController
//{
//public:
//	PIDControllerY(double p_in, double i_in, double d_in) : PIDController(p_in, i_in, d_in) {};
//	void GetParameter(double tarX, double meaX, double dt_in);
//	double PDControl();
//
//};
//
//void PIDControllerY::GetParameter(double tarY, double meaY, double dt_in)
//{
//	Target = tarY;
//	Measure = meaY;
//	dt = dt_in;
//}
//
//double PIDControllerY::PDControl()
//{
//	Error = Target - Measure;
//	Deriv = (Error - PreError) / dt;
//	Output = P * Error + D * Deriv;
//
//	PreError = Error;
//	return Output;


//}

class PIDController
{
public:
	double P = 0.000067*10, I = 0.000067*20, D = 0.000067*2;   // P = 3/10310.0, I = 0.0003
	double integration_x_value_limit = 500, integration_y_value_limit = 500;
	double integration_x_value, integration_y_value;
	double OutputX_Threshold = 0.07, OutputY_Threshold = 0.07;
	double Deriv;
	double MeasureColumn, MeasureRow;
	double TargetColumn = 640, TargetRow = 512;

	double ErrorX, PreErrorX = 0;
	double ErrorY, PreErrorY = 0;
	double OutputX = 0, OutputY = 0;
	double DT = 1;
	double SpeedX = 0;
	double SpeedY = 0;
	double delta_speedX = 0;
	double delta_speedY = 0;


	bool PI_FirstFrame = true;

	PIDController();

	void InputParameters(double meaColumn, double meaRow, double dt);
	
	double PControl_X();
	double PControl_Y();

	double PIControl_X();
	double PIControl_Y();

	double PDControl_X();
	double PDControl_Y();

	double PIDControl_X();
	double PIDControl_Y();

	~PIDController() {};

private:

};

PIDController::PIDController() {
	integration_x_value = 0;
	integration_y_value = 0;
	Deriv = 0;
};

void PIDController::InputParameters(double meaColumn, double meaRow, double dt)
{
	MeasureColumn = meaColumn;
	MeasureRow = meaRow;
	DT = dt;
}

///////////////////////////////////////// P control
double PIDController::PControl_X()
{
	ErrorX = MeasureColumn - TargetColumn;
	OutputX = P * ErrorX;
	std::cout << "OutputX: " << OutputX << std::endl;

	return OutputX;
}


double PIDController::PControl_Y()
{
	ErrorY = MeasureRow - TargetRow;
	OutputY = P * ErrorY;
	std::cout << "OutputY: " << OutputY << std::endl;

	return OutputY;
}

///////////////////////////////////////// P control
//double PIDController::PControl_X()
//{
//	//if (fabs(MeasureColumn - TargetColumn) < 5) {
//	//	std::cout << "----------------Followed object!" << std::endl;
//	//	return 0;
//	//}
//	ErrorX = MeasureColumn - TargetColumn;
//	delta_speedX = P * ErrorX;
//	SpeedX += delta_speedX;
//
//	OutputX = SpeedX;
//	std::cout << "OutputX: " << OutputX << std::endl;
//
//	return OutputX;
//}
//
//
//double PIDController::PControl_Y()
//{
//	//if (fabs(MeasureRow - TargetRow) < 5) {
//	//	std::cout << "----------------Followed object!" << std::endl;
//	//	return 0;
//	//}
//	ErrorY = MeasureRow - TargetRow;
//	delta_speedY = P * ErrorY;
//	SpeedY += delta_speedY;
//
//	OutputY = SpeedY;
//	std::cout << "OutputY: " << OutputY << std::endl;
//
//	return OutputY;
//}
//
/////////////////////////////////////////// PI control
//double PIDController::PIControl_X()
//{
//	
//	ErrorX = MeasureColumn - TargetColumn;
//	delta_speedX = P * ErrorX;
//	SpeedX += delta_speedX;
//
//	OutputX = SpeedX;
//	std::cout << "OutputX: " << OutputX << std::endl;
//
//	return OutputX;
//}
//
//
//double PIDController::PControl_Y()
//{
//	//if (fabs(MeasureRow - TargetRow) < 5) {
//	//	std::cout << "----------------Followed object!" << std::endl;
//	//	return 0;
//	//}
//	ErrorY = MeasureRow - TargetRow;
//	delta_speedY = P * ErrorY;
//	SpeedY += delta_speedY;
//
//	OutputY = SpeedY;
//	std::cout << "OutputY: " << OutputY << std::endl;
//
//	return OutputY;
//}

/////////////////////////////////////////// PD control
//double PIDController::PDControl_X()
//{
//	//if (fabs(MeasureColumn - TargetColumn) < 5) {
//	//	std::cout << "----------------Followed object!" << std::endl;
//	//	return 0;
//	//}
//	ErrorX = MeasureColumn - TargetColumn;
//	delta_speedX = P * ErrorX;
//	SpeedX += delta_speedX;
//
//	OutputX = SpeedX + D * (ErrorX - PreErrorX);
//	PreErrorX = ErrorX;
//	std::cout << "OutputX: " << OutputX << std::endl;
//
//	return OutputX;
//}
//
//
//double PIDController::PDControl_Y()
//{
//	//if (fabs(MeasureRow - TargetRow) < 5) {
//	//	std::cout << "----------------Followed object!" << std::endl;
//	//	return 0;
//	//}
//	ErrorY = MeasureRow - TargetRow;
//	delta_speedY = P * ErrorY;
//	SpeedY += delta_speedY;
//
//	OutputY = SpeedY + D * (ErrorY - PreErrorY);
//	PreErrorY = ErrorY;
//	std::cout << "OutputY: " << OutputY << std::endl;
//
//	return OutputY;
//}

///////////////////////////////////////// PI control
//double PIDController::PIControl_X()
//{
//	Error = MeasureColumn - TargetColumn;
//	Integ = (Error - PreError) * DT;
//	OutputX = P * Error + I * Integ;
//
//	PreError = Error;
//	
//	return OutputX;
//}
//
//double PIDController::PIControl_Y()
//{
//	Error = MeasureRow - TargetRow;
//	Integ = (Error - PreError) * DT;
//	OutputY = P * Error + I * Integ;
//
//	PreError = Error;
//	
//	return OutputY;
//}

double PIDController::PIControl_X()
{
	// For the first frame, PI cannot get the correct dt
	if(PI_FirstFrame)
	{
		PI_FirstFrame = false;
		return 0;
	}

	if (fabs(OutputX) <= OutputX_Threshold)
	{
		ErrorX = MeasureColumn - TargetColumn;
		if (fabs(integration_x_value) <= integration_x_value_limit)
		{
			integration_x_value += ErrorX * DT;
		}
		OutputX = P * ErrorX + I * integration_x_value;
	}
	else if(OutputX > OutputX_Threshold)
	{
		OutputX = OutputX_Threshold;
		std::cout << "******************* X+ Big Move *********************" << std::endl;
	}
	else if (OutputX < -OutputX_Threshold)
	{
		OutputX = -OutputX_Threshold;
		std::cout << "******************* X- Big Move *********************" << std::endl;
	}
	else
	{
		OutputX = 0;
	}

	std::cout << "integration_x_value= " << integration_x_value << std::endl;
	std::cout << "OutputX= " << OutputX << std::endl;
	return OutputX;
}

double PIDController::PIControl_Y()
{
	if (PI_FirstFrame)
	{
		PI_FirstFrame = false;
		return 0;
	}

	if (fabs(OutputY) < OutputY_Threshold)
	{
		ErrorY = MeasureRow - TargetRow;
		if (fabs(integration_y_value) <= integration_y_value_limit)
		{
			integration_y_value += ErrorY * DT;
		}
		OutputY = P * ErrorY + I * integration_y_value;
	}
	else if (OutputY > OutputY_Threshold)
	{
		OutputY = OutputY_Threshold;
		std::cout << "******************* Y+ Big Move ********************" << std::endl;
	}
	else if (OutputY < -OutputY_Threshold)
	{
		OutputY = -OutputY_Threshold;
		std::cout << "******************* Y- Big Move*********************" << std::endl;
	}
	else
	{
		OutputY = 0;
	}
	std::cout << "integration_y_value= " << integration_y_value << std::endl;
	std::cout << "OutputY= " << OutputY << std::endl;

	return OutputY;
}

//////////////////////////////////////////// PD control
//double PIDController::PDControl_X()
//{
//	Error = MeasureColumn - TargetColumn;
//	Deriv = (Error - PreError) / DT;
//	OutputX = P * Error + D * Deriv;
//
//	PreError = Error;
//	return OutputX;
//}
//
//
//double PIDController::PDControl_Y()
//{
//	Error = MeasureRow - TargetRow;
//	Deriv = (Error - PreError) / DT;
//	OutputY = P * Error + D * Deriv;
//
//	PreError = Error;
//	return OutputY;
//}
//
//////////////////////////////////////////// PID control
//double PIDController::PIDControl_X()
//{
//	Error = MeasureColumn - TargetColumn;
//	integration_x_value = (Error - PreError) * DT;
//	Deriv = (Error - PreError) / DT;
//	OutputX = P * Error + I * integration_x_value + D * Deriv;
//
//	PreError = Error;
//	return OutputX;
//}
//
//
//double PIDController::PIDControl_Y()
//{
//	Error = MeasureRow - TargetRow;
//	integration_y_value = (Error - PreError) * DT;
//	Deriv = (Error - PreError) / DT;
//	OutputY = P * Error + I * integration_y_value + D * Deriv;
//
//	PreError = Error;
//	return OutputY;
//}
