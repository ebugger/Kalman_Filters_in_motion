#include <iostream>
#include <vector>
#include "eigen3/Eigen/Dense"

using std::cout;
using std::endl;
using std::vector;
//using Eigen::VectorXd;
//using Eigen::MatrixXd;
//using namespace Eigen;

//Kalman Filter varibales:
Eigen::VectorXd x; //object state
Eigen::MatrixXd P; //uncertainty covariance
Eigen::MatrixXd F; //state transition function
Eigen::VectorXd u; //external motion
Eigen::MatrixXd H; //measurement Function
Eigen::MatrixXd R; //measurement covariance(Noise)
Eigen::MatrixXd I; //identity matrix
Eigen::MatrixXd Q; //process covariance

vector<Eigen::VectorXd> measurements;

void filter(Eigen::VectorXd &x, Eigen::MatrixXd &P);

int main() {
    //1D motion design
    x = Eigen::VectorXd(2);
    x <<0., 0.;

    P = Eigen::MatrixXd(2,2);
    P << 1000., 0., 0., 1000.;

    u = Eigen::VectorXd(2);
    u << 0., 0.;

    F = Eigen::MatrixXd(2,2);
    F << 1., 1., 0., 1.;

    H = Eigen::MatrixXd(1,2);  //1-row, 2-cols
    H << 1., 0;

    R = Eigen::MatrixXd(1,1);  //1-row, 1-col
    R << 1.;

    I = Eigen::MatrixXd::Identity(2, 2);

    Q = Eigen::MatrixXd(2,2); //2 by 2
    Q << 0., 0., 0., 0.;

    //create list of measurements
    Eigen::VectorXd single_meas(1);
    single_meas << 1;
    measurements.push_back(single_meas);
    single_meas << 2;
    measurements.push_back(single_meas);    
    single_meas << 3;
    measurements.push_back(single_meas);

    //call filter
    filter(x, P);

    return 0;
}

void filter(Eigen::VectorXd &x, Eigen::MatrixXd &P) {
    for(unsigned int n=0; n < measurements.size(); n++) {
        Eigen::VectorXd z = measurements[n];
        //measurement update
        Eigen::VectorXd y = z - H * x;       
        Eigen::MatrixXd Ht = H.transpose();
        Eigen::MatrixXd S = H * P * Ht + R;
        Eigen::MatrixXd St = S.inverse();
        Eigen::MatrixXd K = P * Ht * St;

        //new state
        x = x + (K * y);
        P = (I - K * H) * P;

        //prediction
        x = F * x + u;
        Eigen::MatrixXd Ft = F.transpose();
        P = F * P * Ft + Q;

        cout<<"x = "<<endl<< x <<endl;
        cout<<"P = "<<endl<< P <<endl;
    }
}


