#include <iostream>
#include <fstream>

#include "PID_Ctrl.h"

int main(){
    PID_Ctrl pid_ctrl(10, 1, 2, 5);
    
    std::ofstream myfile;
    myfile.open ("testing.csv");
    for (int i = 20; i > 0; i--){
        myfile << pid_ctrl.update_ctrl_signal(i, 0.1) << ",";
    }
    myfile.close();

}