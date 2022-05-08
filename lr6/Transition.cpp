#pragma once
#include "Transition.h"
#include <vector>
#include <cmath>
#include <map>
#include <iostream>
#include <fstream>

Transition::Transition()
{
	lambda0 = 34.98728944195329; // долгота аэродрома Орловка
	phi0 = 56.14352396617182; // широта
	
}

Transition::Transition(double lambda, double phi)
{
	lambda0 = lambda;
	phi0 = phi;
}

std::vector<double> Transition::fromStart2Geogr(std::vector<double> vec)
{
	double x, z, lambda, dlambda, phi, dphi, psi, dx, dz;
	x = 0; z = 0;
	lambda = lambda0;  phi = phi0;
	dx = 111000; // шаг по долготе
	while ((abs(vec[0] - x) > 300) || (abs(vec[2] - z) > 300))
	{
		psi = atan((vec[2] - z) / (vec[0] - x)); // текущий угол курса
		dlambda = floor(lambda) + 1 - lambda; // разница между текущей долготой и следующей (в градусах)
		dphi = floor(phi) + 1 - phi; // разница между текущей широтой и следующуй (в градусах)

		// шаг по широте: 
		if (dphi < 0.5)
			dz = 1000 * table[int(floor(phi) + 1)];
		else
			dz = 1000 * table[int(floor(phi))];

		if ((vec[0] - x) < dx and (vec[2] - z) < dz)
		{
			lambda = lambda + ((vec[2] - z) / dz);
			phi = phi + ((vec[0] - x) / dx);
			x = x + (vec[0] - x);
			z = z + (vec[2] - z);
		}
		else
		{
			if (dlambda * dz < dphi * dx) // т.е. следующая широта ближе чем следующая долгота
			{
				x = x + dphi * dx;
				z = z + (tan(psi) * (dphi * dx));
				phi = phi + dphi;
				lambda = lambda + ((tan(psi) * (dphi * dx)) / dz); // считаем на сколько градусов передвинулись по широте, т.е. чему равна текущая долгота
			}
			else
			{
				z = z + dlambda * dz;
				x = x + (atan(psi) * (dlambda * dz));
				lambda = lambda + dlambda;
				phi = phi + ((atan(psi) * (dlambda * dz)) / dx);
			}
		}
	}
	std::vector<double> res = { lambda, phi, vec[1] };
	return res;
}

double Transition::getAngleFromGeogr(std::vector<double> vec1, std::vector<double> vec2) // угол курса при движении от vec1 к vec2. на вход векторы с географическими координатами (долгота, широта, высота)
{
	double x, z, angle;
	// вычисляем перемещение по оси z
	if (vec1[0] == vec2[0]) // долгота не меняется -> по оси z не двигаемся
		z = 0;
	else
		z = abs(vec1[0] - vec2[0]);

	// вычисляем перемещение по оси x:
	if (vec1[1] == vec2[1]) // широта не меняется -> по оси x не двигаемся
		x = 0;
	else
		x = abs(vec1[1] - vec2[1]);

	if (z == 0 || x == 0) // если идём по какой-то из осей
		angle = 0;
	else // если прошли И по оси x, И по оси z
	{
		angle = atan(z / x);
		angle = fromRad2Grad(angle);
	}
	
	// угол при движении только по оси x
	if (x == 0)
	{
		if (vec1[0] < vec2[0])
			angle = 90;
		else
			angle = 270;
	}

	// угол при движении только по оси z
	if (z == 0)
	{
		if (vec1[1] < vec2[1])
			angle = 0;
		else
			angle = 180;
	}

	// движение на восток под углом:
	if (vec1[0] < vec2[0]) // курс (0, 180)
	{
		if (vec1[1] < vec2[1]) //курс (0, 90)
			angle = angle;
		else // курс (90, 180)
			angle = 180 - angle;
	}
	// движение на запад под углом:
	else // курс (180, 360)
	{
		if (vec1[1] < vec2[1]) //курс (180, 270)
			angle = 180 + angle;
		else // курс (270, 360)
			angle = 360 - angle;
	}
	return angle;
}

//std::vector<double> Transition::getDistanceFromGeogr(std::vector<double> vec1, std::vector<double> vec2) //
//{
//	double x, z, lambda, dlambda, phi, dphi, psi, dx, dz;
//	x = 0; z = 0;
//	lambda = vec1[0]; phi = vec1[1];
//	dx = 111000; // шаг по долготе
//
//	if (vec2[1] == 90) // считаем расстояние до северного полюса
//	{
//		while (phi < 90)
//		{
//			dphi = floor(phi) + 1 - phi;
//			if (dphi < 1)
//			{
//				x = x + dx * (dphi / 1);
//				phi = floor(phi) + 1;
//			}
//			else
//			{
//				x = x + dx;
//				phi = phi + 1;
//			}
//		}
//	}
//	else if ((phi < vec2[1]) && (lambda < vec2[0])) // движемся вверх вправо
//	{
//		while ((abs(phi - vec1[1])) || (abs(lambda - vec1[0])))
//		{
//
//		}
//		dphi = floor(phi) + 1 - phi;
//		if (dphi < 0.5)
//			dz = 1000 * table[int(floor(phi) + 1)];
//		else
//			dz = 1000 * table[int(floor(phi))];
//
//		if (vec2[1] - floor(phi) < 1)
//		{
//
//		}
//		z = z + dz;
//
//	}
//	else if ((phi < vec2[1]) && (lambda > vec2[0])) // движемся вверх влево
//	{
//
//	}
//	else if ((phi > vec2[1]) && (lambda < vec2[0])) // движемся вниз вправо
//	{
//
//	}
//	else if ((phi > vec2[1]) && (lambda > vec2[0])) // движемся вниз влево
//	{
//		
//	}
//	else if ((phi < vec2[1]) && (lambda = vec2[0])) // движемся строго вверх
//	{
//		while (phi < vec2[1])
//		{
//			
//			if (vec2[1] - phi < 1)
//			{
//				dphi = vec2[1] - phi;
//				x = x + dx * (dphi / 1);
//				phi = vec2[1];
//				break;
//			}
//			else 
//				dphi = floor(phi) + 1 - phi;
//			if (dphi < 1)
//			{
//				x = x + dx * (dphi / 1);
//				phi = floor(phi) + 1;
//			}
//			else
//			{
//				x = x + dx;
//				phi = phi + 1;
//			}
//		}
//	}
//	else if ((phi < vec2[1]) && (lambda = vec2[0])) // движемся строго вниз
//	{
//
//	}
//	else if ((lambda < vec2[0]) && (phi = vec2[1])) // движемся строго вправо
//	{
//
//	}
//	else if ((lambda > vec2[0]) && (phi = vec2[1])) // движемся строго влево
//	{
//
//	}
//
//	std::vector<double> res = { }; //расстояние по прямой, движение по вертикали, движение по горизотали
//	return res;
//}

//std::vector<double> Transition::fromGeogr2Start(std::vector<double> coor)
//{
//	double Az, lambda, dlambda, phi, dphi, distZ, distX, Z, dz, X, dx, B; //
//	Az = getAngleFromGeogr({ lambda0, phi0, 0 }, coor);
//	lambda = coor[0];
//	phi = coor[1];
//	dx = 111000;
//	if (Az == 0)
//	{
//
//	}
//	else if (Az == 90)
//	{
//
//	}
//	else if (Az == 180)
//	{
//
//	}
//	else if (Az == 270)
//	{
//		
//	}
//	else
//	{
//		if (Az < 90)
//		{
//			B = 90 - Az;
//			while ((lambda != lambda0) && (phi != phi0))
//			{
//				// считаем шаг по широте
//				if (phi - floor(phi) < 0.5)
//					dz = 1000 * table[int(floor(phi))];
//				else
//					dz = 1000 * table[int(floor(phi)) + 1];
//
//				// считаем расстояние до следующей долготы
//				if (floor(lambda) < lambda0)
//					dlambda = lambda - lambda0;
//				else if (lambda == floor(lambda))
//					dlambda = 1;
//				else
//					dlambda = lambda - floor(lambda);
//
//				// считаем расстояние до следующей широты
//				if (floor(phi) < phi0)
//					dphi = phi - phi0;
//				else if (phi == floor(phi))
//					dphi = 1;
//				else
//					dphi = phi - floor(phi);
//
//				distX = dx * dphi / sin(B);// сколько пройдём по текущему направлению, если пойдём до следующей широты
//				distZ =  // сколько пройдём по текущему направлению, если пойдём до следующей долготы
//			}
//		}
//		else if (Az < 180)
//		{
//			B = 180 - Az;
//			while ((lambda != lambda0) && (phi != phi0))
//			{
//
//			}
//		}
//		else if (Az < 270)
//		{
//			B = Az - 180;
//			while ((lambda != lambda0) && (phi != phi0))
//			{
//
//			}
//		}
//		else
//		{
//			B = Az - 270;
//			while ((lambda != lambda0) && (phi != phi0))
//			{
//
//			}
//		}
//	}
//	return std::vector<double>();
//}

std::vector<std::vector<double>> Transition::getGlissade(double dist, int flag)
{
	std::vector<double> or_coor, coor; // or_coor, coor - географические координаты
	std::vector<std::vector<double>> res; // res - стартовая СК
	double theta, Az, X, Y, Z, dist_t;
	int n;

	or_coor = {lambda0, phi0, 0};
	theta = fromGrad2Rad(3); // угол подъёма глиссады
	n = 5;
	dist_t = dist / n;

	if (flag == 1) // летим с запада
	{
		
		coor = { 34.98409801930414, 56.14287646318206, 0 }; // координаты для расчёта угла
		//Az = getAngleFromGeogr(coor, or_coor); // азимут глиссады
		Az = 65.75;
		for (int i = 0; i < n; i++)
		{
			Y = sin(theta) * dist;
			X = cos(fromGrad2Rad(Az)) * dist;
			Z = sin(fromGrad2Rad(Az)) * dist;
			std::vector<double> r = { -(X + 75.93), Y, -(Z + 198.39) };
			res.insert(res.end(), r);
			//res[i] = r;
			dist = dist - dist_t;
		}
	}
	else // летим с востока
	{
		coor = { 34.98728944195329,56.14352396617182,0 };
		//Az = getAngleFromGeogr(or_coor, coor); // азимут глиссады
		Az = 65.75;
		for (int i = 0; i <= n; i++)
		{
			Y = sin(theta) * dist;
			X = cos(fromGrad2Rad(Az)) * dist;
			Z = sin(fromGrad2Rad(Az)) * dist;
			std::vector<double> r = { X + 155.25, Y, Z + 387.86 };
			res.insert(res.end(), r);
			//res[i] = r;
			dist = dist - dist_t;
		}
	}
	return res;
}



void Transition::setDefault(double lambda, double phi)
{
	lambda0 = lambda;
	phi0 = phi;
}

double Transition::getAngleFromScalars(std::vector<double> x1, std::vector<double> x2)
{
	if (x1.size() == 3)
	{
		double x1len = sqrt(pow(x1[0], 2) + pow(x1[1], 2) + pow(x1[2], 2));
		double x2len = sqrt(pow(x2[0], 2) + pow(x2[1], 2) + pow(x2[2], 2));
		return  acos((x1[0] * x2[0] + x1[1] * x2[1] + x1[2] * x2[2]) / (x1len * x2len));
	}
	else if (x1.size() == 2)
	{
		double x1len = sqrt(pow(x1[0], 2) + pow(x1[1], 2));
		double x2len = sqrt(pow(x2[0], 2) + pow(x2[1], 2));
		if (x2[1] < 0)
			return  -acos((x1[0] * x2[0] + x1[1] * x2[1]) / (x1len * x2len));
		else if (x2[1] > 0)
			return  acos((x1[0] * x2[0] + x1[1] * x2[1]) / (x1len * x2len));
		else if (x2[1] == 0)
			return 0;
	}
}

double Transition::getDistance(std::vector<double> x1, std::vector<double> x2)
{
	if (x1.size() == 3)
	{
		double a = sqrt(pow(x1[0] - x2[0], 2) + pow(x1[1] - x2[1], 2) + pow(x1[2] - x2[2], 2));
		return a;
	}
	if (x1.size() == 2)
	{
		double a = sqrt(pow(x1[0] - x2[0], 2) + pow(x1[1] - x2[1], 2));
		return a;
	}
}


void Transition::WriteFile(std::ofstream& file1, int flag, std::vector<double> vec)
{
	file1.precision(8);
	if (flag == 1)
	{
		file1 << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
		file1 << "\n<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">";
		file1 << "\n<Document>";
		file1 << "\n    ";
		file1 << "<name>Шереметьево-Хитроу.kml</name>";
		file1 << "\n    ";
		file1 << "<Style id=\"s_ylw-pushpin\">";
		file1 << "\n        ";
		file1 << "<IconStyle>";
		file1 << "\n            ";
		file1 << "<scale>1.1</scale>";
		file1 << "\n            ";
		file1 << "<Icon>";
		file1 << "\n                ";
		file1 << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>";
		file1 << "\n            ";
		file1 << "</Icon>";
		file1 << "\n            ";
		file1 << "<hotSpot x=\"20\" y=\"2\" xunits=\"pixels\" yunits=\"pixels\"/>";
		file1 << "\n        ";
		file1 << "</IconStyle>";
		file1 << "\n        ";
		file1 << "<LineStyle>";
		file1 << "\n            ";
		file1 << "<color>ffffad41</color>";
		file1 << "\n        ";
		file1 << "</LineStyle>";
		file1 << "\n    ";
		file1 << "</Style>";
		file1 << "\n    ";
		file1 << "<Style id=\"s_ylw-pushpin_hl\">";
		file1 << "\n        ";
		file1 << "<IconStyle>";
		file1 << "\n            ";
		file1 << "<scale>1.3</scale>";
		file1 << "\n            ";
		file1 << "<Icon>";
		file1 << "\n                ";
		file1 << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>";
		file1 << "\n            ";
		file1 << "</Icon>";
		file1 << "\n            ";
		file1 << "<hotSpot x=\"20\" y=\"2\" xunits=\"pixels\" yunits=\"pixels\"/>";
		file1 << "\n        ";
		file1 << "</IconStyle>";
		file1 << "\n        ";
		file1 << "<LineStyle>";
		file1 << "\n            ";
		file1 << "<color>ffffad41</color>";
		file1 << "\n        ";
		file1 << "</LineStyle>";
		file1 << "\n    ";
		file1 << "</Style>";
		file1 << "\n    ";
		file1 << "<StyleMap id=\"m_ylw-pushpin\">";
		file1 << "\n        ";
		file1 << "<Pair>";
		file1 << "\n            ";
		file1 << "<key>normal</key>";
		file1 << "\n            ";
		file1 << "<styleUrl>#s_ylw-pushpin</styleUrl>";
		file1 << "\n        ";
		file1 << "</Pair>";
		file1 << "\n        ";
		file1 << "<Pair>";
		file1 << "\n            ";
		file1 << "<key>highlight</key>";
		file1 << "\n            ";
		file1 << "<styleUrl>#s_ylw-pushpin_hl</styleUrl>";
		file1 << "\n        ";
		file1 << "</Pair>";
		file1 << "\n    ";
		file1 << "</StyleMap>";
		file1 << "\n    ";
		file1 << "<Placemark>";
		file1 << "\n        ";
		file1 << "<name>Шереметьево-Хитроу</name>";
		file1 << "\n        ";
		file1 << "<styleUrl>#m_ylw-pushpin</styleUrl>";
		file1 << "\n        ";
		file1 << "<LineString>";
		file1 << "\n            ";
		file1 << "<tessellate>1</tessellate>";
		file1 << "\n            ";
		file1 << "<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode>";
		file1 << "\n            ";
		file1 << "<coordinates>";
		file1 << "\n                ";
	}
	if (flag == 2)
	{
		file1 << vec[0]; file1 << ",";
		file1 << vec[1]; file1 << ",";
		file1 << vec[2]; file1 << " ";
	}
	if (flag == 3)
	{
		file1 << "\n            ";
		file1 << "</coordinates>";
		file1 << "\n        ";
		file1 << "</LineString>";
		file1 << "\n    ";
		file1 << "</Placemark>";
		file1 << "\n</Document>";
		file1 << "\n</kml>";
	}
}

void Transition::fillFile(std::ofstream& file1, const char* filenamef, int flag, std::vector<double> vec, long countOperation, int index, double PPMs_size)
{
	if (flag == 1)
	{
		file1.open(filenamef, std::ios::out);
		WriteFile(file1, flag, vec);
		file1.close();
	}
	if (flag == 2)
	{
		if (countOperation % 10 == 0)
		{
			file1.open(filenamef, std::ios::app);
			WriteFile(file1, flag, vec);
			file1.close();
		}

		if (index == PPMs_size)
		{
			file1.open(filenamef, std::ios::app);
			WriteFile(file1, 3, vec);
			file1.close();
		}
	}
}

void Transition::WriteBomb(std::ofstream& file2, std::string name, std::vector<std::vector<double>> cG)
{
	file2.precision(8);
	file2.clear();
	file2.open(name, std::ios::out);

	file2 << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
	file2 << "\n<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">";
	file2 << "\n<Document>";
	file2 << "\n    ";
	file2 << "<name>Шереметьево-Хитроу.kml</name>";
	file2 << "\n    ";
	file2 << "<Style id=\"s_ylw-pushpin\">";
	file2 << "\n        ";
	file2 << "<IconStyle>";
	file2 << "\n            ";
	file2 << "<scale>1.1</scale>";
	file2 << "\n            ";
	file2 << "<Icon>";
	file2 << "\n                ";
	file2 << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>";
	file2 << "\n            ";
	file2 << "</Icon>";
	file2 << "\n            ";
	file2 << "<hotSpot x=\"20\" y=\"2\" xunits=\"pixels\" yunits=\"pixels\"/>";
	file2 << "\n        ";
	file2 << "</IconStyle>";
	file2 << "\n        ";
	file2 << "<LineStyle>";
	file2 << "\n            ";
	file2 << "<color>ffffad41</color>";
	file2 << "\n        ";
	file2 << "</LineStyle>";
	file2 << "\n    ";
	file2 << "</Style>";
	file2 << "\n    ";
	file2 << "<Style id=\"s_ylw-pushpin_hl\">";
	file2 << "\n        ";
	file2 << "<IconStyle>";
	file2 << "\n            ";
	file2 << "<scale>1.3</scale>";
	file2 << "\n            ";
	file2 << "<Icon>";
	file2 << "\n                ";
	file2 << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>";
	file2 << "\n            ";
	file2 << "</Icon>";
	file2 << "\n            ";
	file2 << "<hotSpot x=\"20\" y=\"2\" xunits=\"pixels\" yunits=\"pixels\"/>";
	file2 << "\n        ";
	file2 << "</IconStyle>";
	file2 << "\n        ";
	file2 << "<LineStyle>";
	file2 << "\n            ";
	file2 << "<color>ffffad41</color>";
	file2 << "\n        ";
	file2 << "</LineStyle>";
	file2 << "\n    ";
	file2 << "</Style>";
	file2 << "\n    ";
	file2 << "<StyleMap id=\"m_ylw-pushpin\">";
	file2 << "\n        ";
	file2 << "<Pair>";
	file2 << "\n            ";
	file2 << "<key>normal</key>";
	file2 << "\n            ";
	file2 << "<styleUrl>#s_ylw-pushpin</styleUrl>";
	file2 << "\n        ";
	file2 << "</Pair>";
	file2 << "\n        ";
	file2 << "<Pair>";
	file2 << "\n            ";
	file2 << "<key>highlight</key>";
	file2 << "\n            ";
	file2 << "<styleUrl>#s_ylw-pushpin_hl</styleUrl>";
	file2 << "\n        ";
	file2 << "</Pair>";
	file2 << "\n    ";
	file2 << "</StyleMap>";
	file2 << "\n    ";
	file2 << "<Placemark>";
	file2 << "\n        ";
	file2 << "<name>Шереметьево-Хитроу</name>";
	file2 << "\n        ";
	file2 << "<styleUrl>#m_ylw-pushpin</styleUrl>";
	file2 << "\n        ";
	file2 << "<LineString>";
	file2 << "\n            ";
	file2 << "<tessellate>1</tessellate>";
	file2 << "\n            ";
	file2 << "<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode>";
	file2 << "\n            ";
	file2 << "<coordinates>";
	file2 << "\n                ";

	int i = 0;
	int n = cG.size();
	int k = int(round(n / 10));
	while (i < n)
	{

		file2 << cG[i][0]; file2 << ",";
		file2 << cG[i][1]; file2 << ",";
		file2 << cG[i][2]; file2 << " ";

		if (i == n - 1)
			break;

		i += k;
		if (i > n)
			i = n - 1;
	}


	file2 << "\n            ";
	file2 << "</coordinates>";
	file2 << "\n        ";
	file2 << "</LineString>";
	file2 << "\n    ";
	file2 << "</Placemark>";
	file2 << "\n</Document>";
	file2 << "\n</kml>";

	file2.close();
}

void Transition::writeAngles(std::ofstream& fileA, std::string name, std::vector <double> thetaList, std::vector<double> AList)
{
	int i = 0;
	fileA.open("Angles.txt", std::ios::out);
	fileA << "Номер операции" << "      " << "Угол элевации" << "       " << "Угол курса" << std::endl;
	while (i < thetaList.size()) {
		fileA << i << "                   " << thetaList[i] << "               " << AList[i] << std::endl;
		i++;
	}
	fileA.close();
}

void Transition::WritePPM(std::ofstream& file3, std::string name, std::vector<double> cG)
{
	file3.precision(8);
	file3.clear();
	file3.open(name, std::ios::out);

	file3 << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
	file3 << "\n<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">";
	file3 << "\n<Document>";
	file3 << "\n    ";
	file3 << "<name>PPM.kml</name>";
	file3 << "\n    ";
	file3 << "<StyleMap id=\"m_ylw-pushpin\">";
	file3 << "\n        ";
	file3 << "<Pair>";
	file3 << "\n            ";
	file3 << "<key>normal</key>";
	file3 << "\n            ";
	file3 << "<styleUrl>#s_ylw-pushpin</styleUrl>";
	file3 << "\n        ";
	file3 << "</Pair>";
	file3 << "\n        ";
	file3 << "<Pair>";
	file3 << "\n            ";
	file3 << "<key>highlight</key>";
	file3 << "\n            ";
	file3 << "<styleUrl>#s_ylw-pushpin_hl</styleUrl>";
	file3 << "\n        ";
	file3 << "</Pair>";
	file3 << "\n    ";
	file3 << "</StyleMap>";
	file3 << "\n    ";
	file3 << "<Style id=\"s_ylw-pushpin\">";
	file3 << "\n        ";
	file3 << "<IconStyle>";
	file3 << "\n            ";
	file3 << "<scale>1.1</scale>";
	file3 << "\n            ";
	file3 << "<Icon>";
	file3 << "\n                ";
	file3 << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>";
	file3 << "\n            ";
	file3 << "</Icon>";
	file3 << "\n            ";
	file3 << "<hotSpot x=\"20\" y=\"2\" xunits=\"pixels\" yunits=\"pixels\"/>";
	file3 << "\n        ";
	file3 << "</IconStyle>";
	file3 << "\n    ";
	file3 << "</Style>";
	file3 << "\n    ";
	file3 << "<Style id=\"s_ylw-pushpin_hl\">";
	file3 << "\n        ";
	file3 << "<IconStyle>";
	file3 << "\n            ";
	file3 << "<scale>1.3</scale>";
	file3 << "\n            ";
	file3 << "<Icon>";
	file3 << "\n                ";
	file3 << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>";
	file3 << "\n            ";
	file3 << "</Icon>";
	file3 << "\n            ";
	file3 << "<hotSpot x=\"20\" y=\"2\" xunits=\"pixels\" yunits=\"pixels\"/>";
	file3 << "\n        ";
	file3 << "</IconStyle>";
	file3 << "\n    ";
	file3 << "</Style>";
	file3 << "\n    ";
	file3 << "<Placemark>";
	file3 << "\n        ";
	file3 << "<name>PPM</name>";
	file3 << "\n        ";
	file3 << "<styleUrl>#m_ylw-pushpin</styleUrl>";
	file3 << "\n        ";
	file3 << "<Polygon>";
	file3 << "\n            ";
	file3 << "<tessellate>1</tessellate>";
	file3 << "\n            ";
	file3 << "<outerBoundaryIs>";
	file3 << "\n                ";
	file3 << "<LinearRing>";
	file3 << "\n                    ";
	file3 << "<coordinates>";
	file3 << "\n                        ";

	file3 << cG[0]; file3 << ",";
	file3 << cG[1]; file3 << ",";
	file3 << cG[2]; file3 << " ";
	file3 << cG[0]; file3 << ",";
	file3 << cG[1]; file3 << ",";
	file3 << cG[2]; file3 << " ";

	file3 << "\n                    ";
	file3 << "</coordinates>";
	file3 << "\n                ";
	file3 << "</LinearRing>";
	file3 << "\n            ";
	file3 << "</outerBoundaryIs>";
	file3 << "\n        ";
	file3 << "</Polygon>";
	file3 << "\n    ";
	file3 << "</Placemark>";
	file3 << "\n</Document>";
	file3 << "\n</kml>";

	file3.close();
}

double Transition::fromGrad2Rad(double angle)
{
	return (angle * PI) / 180;
}

double Transition::fromRad2Grad(double rads)
{
	return (rads * 180) / PI;
}