
#include <bits/stdc++.h>
using namespace std;
#define phi 3.14159265359
#define rad (phi / 180.0)

class RobotOmniwheel {
    public:
    double matrix[4][3];
    int translational_velocity[3];
    double wheel_velocity[4];
    double angular_speed[4];
    double radius;

    void inputWheel(fstream &file) {

        file << "Invers Kinematic Omni-Wheel" << endl;
        file << endl;

        cout << "Input X speed: ";
        file << "Input X speed:";
        cin >> translational_velocity[0];
        file << translational_velocity[0] << endl;

        cout << "Input Y speed: ";
        file << "Input Y speed:";
        cin >> translational_velocity[1];
        file << translational_velocity[1] << endl;

        cout << "Input Theta: ";
        file << "Input Theta:";
        cin >> translational_velocity[2];
        file << translational_velocity[2] << endl;

        cout << "Input Radius: ";
        file << "Input Radius: ";
        cin >> radius;
        file << radius << endl;

        cout << endl;

        matrix[0][0] = sinf(45.0 * rad);
        matrix[0][1] = cosf(45.0 * rad);
        matrix[0][2] = radius;

        matrix[1][0] = sinf(135.0 * rad);
        matrix[1][1] = cosf(135.0 * rad);
        matrix[1][2] = radius;

        matrix[2][0] = sinf(225.0 * rad);
        matrix[2][1] = cosf(225.0 * rad);
        matrix[2][2] = radius;

        matrix[3][0] = sinf(315.0 * rad);
        matrix[3][1] = cosf(315.0 * rad);
        matrix[3][2] = radius;

        file << endl;
    }

    void motor_speed(fstream &file) {

        for (int i = 0; i < 4; i++) {
            wheel_velocity[i] = 0;
        }

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                wheel_velocity[i] += matrix[i][j] * translational_velocity[j];
                angular_speed[i] = wheel_velocity[i] / matrix[i][2];
            }
        }

        for (int i = 0; i < 4; i++) {
            file << fixed << setprecision(2) << "Wheel Velocity " << i << " = " << wheel_velocity[i] << " m/s" << endl;
            cout << fixed << setprecision(2) << "Wheel Velocity " << i << " = " << wheel_velocity[i] << " m/s" << endl;
        }   

        file << endl;
        cout << endl;

        for (int i = 0; i < 4; i++) {
            cout << fixed << setprecision(2) << "Angular Speed " << i << " = " << angular_speed[i] << " rad/s" << endl;
            file << fixed << setprecision(2) << "Angular Speed " << i << " = " << angular_speed[i] << " rad/s" << endl;
        }

        file << endl;
        cout << endl;
    }
};

class RobotArm {
    public:
    double x;
    double y;
    double l1;
    double l2;
    double theta1;
    double theta2;

    void inputArm(fstream &file) {

        file << "Invers Kinematic Robot's Arm 2-DOF" << endl;
        file << endl;

        cout << "Input x target coordinate: ";
        file << "Input x target coordinate: ";
        cin >> x;
        file << x << endl;
        cout << endl;

        cout << "input y target coordinate: ";
        file << "input y target coordinate: ";
        cin >> y;
        file << y << endl;
        cout << endl;

        cout << "input robot's arm length 1: ";
        file << "input robot's arm length 1: ";
        cin >> l1;
        file << l1 << endl;
        cout << endl;

        cout << "input robot's arm length 2: ";
        file << "input robot's arm length 2: ";
        cin  >> l2;
        file << l2 << endl;
        cout << endl; 

        file << endl;
    }

    void theta(fstream &file) {
        double distance = sqrt((x*x) + (y*y));
        if (distance > l1 + l2) {
            cout << "Target is out of range";
            file << "Target is out of range";
        }
        else {
            double cos_theta2 = (x*x + y*y - l1*l1 - l2*l2)/(2 * l1 * l2);
            if (cos_theta2 < -1) cos_theta2 = -1;
            else if (cos_theta2 > 1) cos_theta2 = 1;

            theta2 = acos(cos_theta2);
            theta1 = atan2(y,x) - atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));

            theta1 = theta1 * 180.0 / phi;
            theta2 = theta2 * 180.0 / phi;


            cout << "Joint 1 angle: " << fixed << setprecision(2) << theta1 << " degree" << endl;
            file << "Joint 1 angle: " << fixed << setprecision(2) << theta1 << " degree" << endl;

            cout << "Joint 2 angle: " << fixed << setprecision(2) << theta2 << " degree" << endl;
            file << "Joint 2 angle: " << fixed << setprecision(2) << theta2 << " degree" << endl;
        }

    }

};

int main() {
    fstream file;
    file.open("modul_2_robocon.txt", ios::out);
    file << "Tugas Modul 2 Internship ITS ROBOCON\n";
    file << endl;
    
    RobotOmniwheel robot1;
    robot1.inputWheel(file);
    robot1.motor_speed(file);

    RobotArm robot2;
    robot2.inputArm(file);
    robot2.theta(file);

    file.close();

}