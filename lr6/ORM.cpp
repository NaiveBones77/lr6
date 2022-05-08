#include "ORM.h"
#include <cmath>
#include <list>

ORM::ORM(std::vector<Aircraft*> Aircrafts)
{
	this->Aircrafts = Aircrafts;
}

void ORM::checkAircrafts()
{
	bool newlanding = false;
	int sum = 0;
	int n = Aircrafts.size();
	for (int i = 0; i < n; i++)
		if (Aircrafts[i]->islanding == false)
			sum++;
	if (sum == Aircrafts.size())
	{
		int i = 0;
		while ((newlanding == false) && (i < n))
		{
			std::vector <double> a = Aircrafts[i]->startSK;
			std::vector <double> g = gliss[0];
			double D = tr.getDistance(std::vector<double>{g[0], g[2]}, std::vector<double>{a[0], a[2]});
			double angleD = abs(tr.getAngleFromScalars(std::vector<double>{g[0], g[2]}, std::vector<double>{a[0]-g[0], a[2]-g[2]}));
			double AC = sqrt(pow(a[0], 2) + pow(a[2], 2));
			double angleAC = tr.fromRad2Grad(asin(AC * sin(angleD) / D));
			angleD = 180 - tr.fromRad2Grad(angleD);
			bool angletrue = false;
			if ((angleD < 130) && (angleD > 90))
				angletrue = true;

			if ((Aircrafts[i]->islanded == false) && (a[2] > g[2]) && (a[0] < g[0]) && (angletrue == true))
			{
				newlanding = true;
				Aircrafts[i]->calculateEllisp(Aircrafts[i]->startSK, gliss);
				Aircrafts[i]->islanding = true;
			}
			else
				i++;
		}
	}
}
