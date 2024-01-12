#ifndef PID_H
#define PID_H

#include <vector>

class PID_Ctrl {
	double pterm;
	double iterm;
	double dterm;

	double integratorClamp;

	double integratorSum = 0;
	double last_error = 0;

public:
	PID_Ctrl(double pterm, double iterm, double dterm, double integratorClamp);
	double update_ctrl_signal(double error, double timestep);
	void reset_ctrller();
};

#endif //h