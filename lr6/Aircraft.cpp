#pragma once
#include "Aircraft.h"
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>
#include "Timer.cpp"
#include <list>
#include "integrator.h";


Aircraft::Aircraft(double longitude, double latitude, double V0, double A0) {

	this->longitude = longitude;
	this->latitude = latitude;
	V = V0;
	A = A0;
	roll = 0; //крен
	pitch = 0; //тангаж
	yaw = 0; //рысканьет
	Vx = 0;
	Vy = 0;
	Vz = 0;
	Dist = 0;
	theta = 0;
	tr.setDefault(longitude, latitude);
	PPMs.insert(PPMs.end(), std::vector<double>{20000, 9000, 20000});
	PPMs.insert(PPMs.end(), std::vector<double>{20000, 12000, 30000});
	PPMs.insert(PPMs.end(), std::vector<double>{10000, 11000, 10000});
	PPMs.insert(PPMs.end(), std::vector<double>{-10000, 10000, 15000});
	PPMsG.insert(PPMsG.end(), tr.fromStart2Geogr(std::vector<double>{20000, 0, 20000}));
	PPMsG.insert(PPMsG.end(), tr.fromStart2Geogr(std::vector<double>{20000, 0, 50000}));
	PPMsG.insert(PPMsG.end(), tr.fromStart2Geogr(std::vector<double>{10000, 0, 100000}));
	PPMsG.insert(PPMsG.end(), tr.fromStart2Geogr(std::vector<double>{-10000, 0, 150000}));
	tr.WritePPM(file1, std::string("PPM1.kml"), PPMsG[0]);
	tr.WritePPM(file1, std::string("PPM2.kml"), PPMsG[1]);
	tr.WritePPM(file1, std::string("PPM3.kml"), PPMsG[2]);
	tr.WritePPM(file1, std::string("PPM4.kml"), PPMsG[3]);
	ASP bomb0(startSK, std::vector <double> {Vx, 0, Vz});
	ASP bomb1(startSK, std::vector <double> {Vx, 0, Vz});
	ASP bomb2(startSK, std::vector <double> {Vx, 0, Vz});
	ASP bomb3(startSK, std::vector <double> {Vx, 0, Vz});
	bombs.push_back(bomb0);
	bombs.push_back(bomb1);
	bombs.push_back(bomb2);
	bombs.push_back(bomb3);

	SNS sns(Val("высота", 10, 20, 65536),
		Val("HDOP", 10.0, 15, 512),
		Val("VDOP", 10.0, 15, 512),
		Val("путевой угол", 5.0, 15, 90),
		Val("текущая широта ", 55, 20, 90),
		Val("текущая широта (точно)", 3, 11, 0.000085830),
		Val("текущая долгота", 35, 20, 90),
		Val("текущая долгота (точно)", 4, 11, 0.000085830),
		Val("задержка выдачи", 13.3, 20, 512),
		Val("текущее время UTC (старшие разряды)", 6.0, 6, 32),
		Val("текущее время UTC (младшие разряды)", 2.0, 20, 512),
		Val("вертикальная скорость", 1.0, 15, 16384)
	);

	INS ins(Val("широта", 28, 20, 90),
		Val("долгота", 55, 20, 90),
		Val("высота", 130, 19, 19975.3728),
		Val("курс истинный", 15.3, 16, 90),
		Val("тангаж", 3.5, 16, 90),
		Val("крен", 6.3245, 16, 90),
		Val("скорость север юг", 400, 19, 1053.5822),
		Val("скорость восток запад", 200, 19, 1053.5822),
		Val("скорость вертикальная инерциальная", 83, 19, 83.2307),
		Val("ускорение продольное", 0, 12, 19.62),
		Val("ускорение поперечное", 0, 12, 19.62),
		Val("ускорение нормальное", 0, 12, 2)
	);

	tr.fillFile(file1, "123.kml", 1, std::vector<double> {0, 0, 0}, countOperation, index, PPMs.size());
}

Aircraft::Aircraft(std::vector<double> x0, double V0, double A0, int number)
{
	this->startSK = x0;
	this->V = V0;
	this->A = A0;

	tr.setDefault(34.98728944195329, 56.14352396617182);
	

	this->initRound();
	this->curRoundIndex = this->setPPMs(-1);

	ss << "Plane" << number << ".kml";
	filenamef = ss.str();

	tr.fillFile(file1, filenamef, 1, std::vector<double> {0, 0, 0}, countOperation, index, PPMs.size());
}

Aircraft::Aircraft()
{ 

}

void Aircraft::run2()
{
	double dt = 0.1;

	coordinates.insert(coordinates.end(), startSK);
	if (index < PPMs.size())
	{
		Dist = tr.getDistance(startSK, PPMs[index]);
		mutex.lock();
		A = A + Xpr[0] * dt;
		theta = theta + Xpr[1] * dt;
		Vx = V * cos(A);
		Vy = V * (PPMs[index][1] - startSK[1]) / Dist;
		Vz = V * sin(A);
		startSK[0] = startSK[0] + Vx * dt;
		startSK[1] = startSK[1] + Vy * dt;
		startSK[2] = startSK[2] + Vz * dt;
		coordinatesG = tr.fromStart2Geogr(startSK);
		latitude = coordinatesG[0];
		longitude = coordinatesG[1];

		if (tr.getDistance(std::vector<double> {startSK[0], startSK[2]}, std::vector<double>{PPMs[index][0], PPMs[index][2]}) < 10)
		{
			index += 1;
			std::cout << "+1PPM"<<std::endl;
		}
		countOperation += 1;
		mutex.unlock();
	}
	std::vector<double> SNS_vec = { A, longitude, latitude };
	std::vector<double> INS_vec = { latitude, longitude, A, pitch, roll, Vx, Vz };
	fillINS(INS_vec);
	fillSNS(SNS_vec);

	tr.fillFile(file1, "123.kml", 2, std::vector<double> {latitude, longitude, startSK[1]}, countOperation, index, PPMs.size());
	if (index == PPMs.size())
		index = 1000;
}

void  Aircraft::run3()
{
	double dt = 0.1;
	coordinates.insert(coordinates.end(), startSK);
	if (index < PPMs.size())
	{
		Dist = tr.getDistance(startSK, PPMs[index]); //
		mutex.lock();
		A = A + Xpr[0] * dt;
		theta = theta + Xpr[1] * dt;
		Vx = V * cos(A);
		Vy = V * (PPMs[index][1] - startSK[1]) / Dist; //
		Vz = V * sin(A);
		startSK[0] = startSK[0] + Vx * dt;
		startSK[1] = startSK[1] + Vy * dt; //
		startSK[2] = startSK[2] + Vz * dt;
		coordinatesG = tr.fromStart2Geogr(startSK);
		latitude = coordinatesG[0];
		longitude = coordinatesG[1];

		sendMsg(startSK);

		if (tr.getDistance(std::vector<double> {startSK[0], startSK[2]}, std::vector<double>{PPMs[index][0], PPMs[index][2]}) < 200)
		{
			index += 1;
			if (!islanding)
			{
				curRoundIndex = setPPMs(curRoundIndex);
				std::cout << curRoundIndex << std::endl;
			}
			else {
				std::cout << index << std::endl;
			}
		}
		countOperation += 1;
		mutex.unlock();
	}

	tr.fillFile(file1, filenamef, 2, std::vector<double> {latitude, longitude, startSK[1]}, countOperation, index, PPMs.size());
	if (index == PPMs.size())
	{
		this->islanding = false;
		this->islanded = true;
		index = 1000;
	}

}

void Aircraft::OPS2()
{
	mutex.lock();
	if (index < PPMs.size())
	{		
		double theta_delta = asin(startSK[1] / tr.getDistance(startSK, PPMs[index])); // требуемый угол места
		int i = 0;
		if (abs(theta_delta - theta) <= 0.010)
		{
			Xpr[1] = 0;
			thetaList.insert(thetaList.end(), tr.fromRad2Grad(theta));
		}

		else if (theta_delta > theta)
		{
			Xpr[1] = theta_pr[i];
			thetaList.insert(thetaList.end(), tr.fromRad2Grad(theta));
		}

		else if (theta_delta < theta)
		{
			Xpr[1] = -theta_pr[i];
			thetaList.insert(thetaList.end(), tr.fromRad2Grad(theta));
		}

		std::vector<double> ort = { 0, 0 };
		ort[0] = (distSP[0] - startSK[0]) / tr.getDistance(distSP, startSK);
		ort[1] = (distSP[1] - startSK[1]) / tr.getDistance(distSP, startSK);

		double delta = tr.getAngleFromScalars(ort, std::vector<double> {PPMs[index][0] - startSK[0], PPMs[index][2] - startSK[2]}); // требуемый угол курса
		if (abs(delta - A) <= 0.005)
		{
			Xpr[0] = 0;
			AList.insert(AList.end(), tr.fromRad2Grad(A));
		}

		else if (delta > A)
		{
			Xpr[0] = yawmax_pr;
			AList.insert(AList.end(), tr.fromRad2Grad(A));
		}

		else if (delta < A)
		{
			Xpr[0] = -yawmax_pr;
			AList.insert(AList.end(), tr.fromRad2Grad(A));
		}
	}
	mutex.unlock();
}

void Aircraft::calculateEllisp(std::vector<double> x0, std::vector<std::vector<double>> glissade)
{
	mutex.lock();
	PPMs.clear();
	this->index = 0;
	int n = 5;     // количество точек ППМ по эллипсу
	int coeff = 1; //рассматриваем только верхнюю часть эллипса
	std::vector<double> gliss = glissade[0];
	double a_z = tr.getDistance(std::vector<double> {x0[0], 0, x0[2]}, std::vector<double> {gliss[0], 0, gliss[2]});  // расстояние в плоскости XoZ
	double a_y = x0[1] - gliss[1];  // вертикальное расстояние
	double a_xyz = tr.getDistance(x0, gliss);    // расстояние 
	std::vector<double> ort_z = { 0,1 };   // орт на восток
	std::vector<double> dist = { x0[0] - gliss[0],  x0[2] - gliss[2] };  // получаем вектор a перемещенный в начало координат
	double beta = tr.getAngleFromScalars(dist, ort_z);   //угол поворота фигуры
	if (dist[0] < 0)
	{
		beta = -beta;    //Поворот фигуры в другую сторону
	}
	double gamma = tr.getAngleFromScalars(std::vector<double> {x0[0], 0, x0[2]}, std::vector<double> {gliss[0], 0, gliss[2]}); // угол между глиссадой и расстоянием от конца глиссады до н.т ЛА
	if (gamma > 3.14 / 2)
	{
		coeff = -1;   //рассматриваем только нижнюю часть эллипса
	}
	double b = a_z / 2 * sin(gamma);  //примерная функция зависимости b
	double k = b / pow(a_z, 2);
	std::vector<std::vector<double>> parabola;
	std::vector<std::vector<double>> ellips;
	double a = a_z / 2;
	std::vector<double> shift = { (x0[0] - gliss[0]) / 2 + gliss[0], (x0[1] - gliss[1]) / 2, (x0[2] - gliss[2]) / 2 + gliss[2] }; //сдвиг ск в центр вектора расстояния от положения ЛА до глиссады
	for (int i = 0; i <= n; i++)
	{
		double z = -a_z / 2 + i * a_z / n;
		double y = x0[1] - i * a_y / n;
		double x = coeff * sqrt(pow(b, 2) * (1 - pow(z, 2) / pow(a, 2)));  // Координаты эллиипса в центре связанной ск
		double u = x * cos(beta) + z * sin(beta) + shift[0];
		double v = -x * sin(beta) + z * cos(beta) + shift[2];			// Афинное преобразование координат эллипса
		ellips.insert(ellips.end(), std::vector<double> { x, y, z });
		PPMs.insert(PPMs.end(), std::vector<double> { u, y, v });
	}
	if (tr.getDistance(PPMs[0], x0) > tr.getDistance(PPMs[n - 1], x0))
	{
		//PPMs.pop_back();
		std::reverse(PPMs.begin(), PPMs.end());							// Если при повороте эллипса начальные точки оказались в противоположных местах
		for (int i = 0; i < PPMs.size(); i++)									// Не лучший способ пересчета высоты
		{
			double y = x0[1] - i * a_y / n;
			PPMs[i][1] = y;
		}
		std::reverse(PPMs.begin(), PPMs.end());
		PPMs.pop_back();
		std::reverse(PPMs.begin(), PPMs.end());
	}

	for (int i = 1; i < glissade.size(); i++)
	{
		PPMs.insert(PPMs.end(), glissade[i]);
	}
	PPMs.insert(PPMs.end(), std::vector<double> {0,0,0});

	//Кошмар
	file1.open("PPMs.kml", std::ios::out);
	tr.WriteFile(file1, 1, std::vector<double> {0, 0, 0});
	for (int i = 0; i < PPMs.size(); i++)
	{
		PPMsG.insert(PPMsG.end(), tr.fromStart2Geogr(PPMs[i]));
		tr.WriteFile(file1, 2, PPMsG[i]);
	}
	tr.WriteFile(file1, 3, std::vector<double> {0, 0, 0});
	file1.close();
	mutex.unlock();
}

void Aircraft::initRound()
{
	double r = tr.getDistance(std::vector<double> {startSK[0], startSK[2]}, std::vector<double> {0, 0});  //радиус окружности
	double l = 2 * 3.14 * r;
	double dist = 2000;
	int n = int(l / dist);  //количество точек, так что, каждые 2 км добаляем ППМ
	double x = -r;
	while (x <= r)
	{
		double y = -sqrt(pow(r, 2) - pow(x, 2));
		std::vector<double> point = { y, startSK[1], x };
		PPMsRound.insert(PPMsRound.end(), point);
		x += dist;
	}
	x = r;
	while (x >= -r)
	{
		double y = sqrt(pow(r, 2) - pow(x, 2));
		std::vector<double> point = { y, startSK[1], x };
		PPMsRound.insert(PPMsRound.end(), point);
		x -= dist;
	}

}

int Aircraft::setPPMs(int indexCur)
{
	int index = 0;
	if (indexCur == -1)
	{
		double dist = DBL_MAX;
		double distMin = 2000;
		std::vector<double> pos;
		for (int i = 0; i < PPMsRound.size(); i++)
		{
			pos = std::vector<double>{ PPMsRound[i][0], PPMsRound[i][2] };
			double curDist = tr.getDistance(pos, std::vector<double> {startSK[0], startSK[2]});
			if (startSK[0] <= 0)
			{
				if (curDist < dist && curDist > distMin && pos[1] > startSK[2])
				{
					dist = curDist;
					index = i;
				}
			}
			else if (startSK[0] >= 0)
			{
				if (curDist < dist && curDist > distMin && pos[1] < startSK[2])
				{
					dist = curDist;
					index = i;
				}
			}

		}
		PPMs.insert(PPMs.end(), PPMsRound[index]);
		return index;
	}
	else {

		if (indexCur == PPMsRound.size() - 1)
		{
			index = 0;
			PPMs.insert(PPMs.end(), PPMsRound[index]);
			return index;
		}
		else {
			index = indexCur;
			PPMs.insert(PPMs.end(), PPMsRound[index + 1]);
			return index + 1;
		}
	}

}

void Aircraft::startBomb()
{
	mutex.lock();
	Timer timer;
	if ((curIndexBomb < bombs.size()) && bombs[curIndexBomb].flag)
	{
		ASP bomb = bombs[curIndexBomb];
		bomb.setX(startSK);
		bomb.setV(std::vector<double> {V* cos(A), 0, V* sin(A)});
		bomb.Ab_f();
		if ((abs(tr.getDistance(std::vector<double> {startSK[0], startSK[2]}, std::vector<double>{PPMs[index][0], PPMs[index][2]}) - bomb.A)
			< 100) && (index == curIndexBomb))
		{
			//bomb.tr.setDefault(coordinatesG[0], coordinatesG[1]);
			std::vector<float> initial{ (float)startSK[0], (float)startSK[1], (float)startSK[2], (float)Vx, 0, (float)Vz };
			bomb.setInitialConditions(initial);
			bomb.setSampIncrement(0.1);
			bomb.setT_st(0);
			bomb.setT_fin(bomb.T_h);

			dormandPrinceIntgrator* dp_integrator = new dormandPrinceIntgrator();

			//bomb.setName(curIndexBomb);
			dp_integrator->run(&bomb);
			bomb.insertGeogr();
			bomb.setName(index);
			tr.WriteBomb(file1, bomb.name, bomb.coordinatesG);
			//bomb.run();

			bombs[curIndexBomb].flag = false;
			
			
			if ((curIndexBomb == 3) && (bomb.coordinatesG[bomb.coordinatesG.size()-1][2] < 50))
			{
				END = true;
				// curIndexBomb = 1000;
			}

			if (curIndexBomb < bombs.size() - 1)
			{
				curIndexBomb += 1;
			}

		}
	}
	if (END == true) // ((END == true) && (index == PPMs.size()))
	{
		tr.writeAngles(file1, std::string("Angles.txt"), thetaList, AList);
		curIndexBomb = 1000;
		END = false;
	}

	mutex.unlock();

}

void Aircraft::run_asp(ASP& asp)
{
	asp.run();
}

void Aircraft::fillSNS(std::vector<double> vec)
{
	sns.mutex.lock();
	sns.H.value = 10000;
	sns.trackAngle.value = vec[0];
	sns.curLatitude.value = vec[1];
	sns.curLongitude.value = vec[2];
	sns.mutex.unlock();
}

void Aircraft::fillINS(std::vector<double> vec)
{
	ins.mutex.lock();
	ins.H.value = 10000;
	ins.Latitude.value = vec[0];
	ins.Longitude.value = vec[1];
	ins.CourseTrue.value = vec[2];
	ins.Tungazh.value = vec[3];
	ins.List.value = vec[4];
	ins.VelocityNS.value = vec[5];
	ins.VelocityEW.value = vec[6];
	ins.mutex.unlock();
}

int Aircraft::bindPort(SOCKET s, sockaddr_in destAddr) {
	_s = s;
	_destAddr = destAddr;
	return 1;
}

void Aircraft::sendMsg(std::vector<double> vector)
{
	char* buffer;
	std::string msg = std::to_string(vector[0]);
	msg.append(" ");
	msg.append(std::to_string(vector[1]));
	msg.append(" ");
	msg.append(std::to_string(vector[2]));
	buffer = new char[sizeof(msg)];
	memset(buffer, 0, sizeof(msg));
	memcpy(buffer, &msg, sizeof(msg));
	char s = *buffer;
	//sendto(_s, &buffer[0], sizeof(buffer), 0,
	//	(sockaddr*)&_destAddr, sizeof(_destAddr));

	int n_bytes = ::sendto(_s, msg.c_str(), msg.length(), 0, reinterpret_cast<sockaddr*>(&_destAddr), sizeof(_destAddr));
	std::cout << n_bytes << " bytes sent" << std::endl;
}