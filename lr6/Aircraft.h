#pragma once
#include "SNS.h"
#include "INS.h"
#include <vector>
#include "Transition.h"
#include <mutex>
#include <iostream>
#include <fstream>
#include "ASP.h"
#include <list>
#include "integrator.h";	
#include <string>
#include <sstream>

class Aircraft {
private:

	Transition tr;
	std::mutex mutex;
	std::ofstream file1;
	std::stringstream ss;
	const double PI = 3.141592653589793;
	double roll, pitch, yaw;		// ���� ������ ��������
	double longitude, latitude;		// ������� ������
	double A, theta;				// ���� ����� � �����
	double V, Vx, Vy, Vz, Dist;
	// std::vector <double> startSK = { 0,10000,0 };
	std::vector <double> thetaList = { 0 };
	std::vector <double> AList = { 0 };
	std::vector <ASP> bombs = {};
	int curIndexBomb = 0;
	int curRoundIndex = -1;
	int indexLA;   //������ ��� �������� ������ UDP

	SOCKET _s;
	sockaddr_in _destAddr;

	bool END = false;
	//std::list <ASP> bombs;
	//list <ASP> ::iterator it = bombs.begin();

	std::vector <std::vector<double>> coordinates; // ���������� �� � ��������� ��
	std::vector <double> coordinatesG; // ���������� �� � �������������� ��
	std::vector <std::vector<double>> PPMs;	// ���������� ����� � ��������� ��
	std::vector <std::vector<double>> PPMsG; // ���������� ����� � �������������� ��
	std::vector <std::vector<double>> PPMsRound;		//���������� ����� �� ����������

	std::vector <double> Xpr = { 0, 0 }; // ���������� �� ��������
	int index = 0; // ������ �������� �� ���
	double yawmax_pr = 0.14; // ����������� �� A (���� �����)
	std::vector <double> theta_pr = {0.02}; // ����������� �� theta (���� �����)
	int countPPM = 0;							//���������� �����
	long countOperation = 0;						//���������� ��������

	double t = 0;
	std::vector<double> distSP = { 3777201.83, 0 };	// � �� ���. ������ �� ������� �����

public:
	INS ins;
	SNS sns;

	std::string filenamef;

	std::vector <double> startSK = { 0,10000,0 }; //
	bool islanding = false; //
	bool islanded = false; //

	Aircraft();
	Aircraft(std::vector<double> x0, double V0, double A0, int number);
	Aircraft(double longitude, double latitude, double V0, double A0);

	void run3();
	void run2();
	void OPS2();
	void startBomb();
	void fillSNS(std::vector<double> vec);
	void fillINS(std::vector<double> vec);
	void run_asp(ASP& asp);
	void calculateEllisp(std::vector<double> x0, std::vector<std::vector<double>> glissade);
	void initRound();
	void initRound(double R, double verticalShift);
	int setPPMs(int indexCur);
	int bindPort(SOCKET s, sockaddr_in destAddr);
	void sendMsg(std::vector<double> vec, int countOP);
};
