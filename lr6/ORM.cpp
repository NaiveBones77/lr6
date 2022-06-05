#include "ORM.h"
#include <cmath>
#include <list>

ORM::ORM(std::vector<Aircraft*> Aircrafts)
{
	this->Aircrafts = Aircrafts;
	int len = Aircrafts.size();
	tableAirs = new int* [len];
	for (int i = 0; i < len; i++)
	{
		tableAirs[i] = new int [len];
	}
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

void ORM::checkZones()
{
	mutex.lock();
	int n = Aircrafts.size();
	for (int i = 0; i < n - 1; i++)
	{
		for (int j = i + 1; j < n; j++)
		{
			double dist = tr.getDistance(Aircrafts[i]->startSK, Aircrafts[j]->startSK);
			if (dist < 2000 && tableAirs[i][j] != 1 && !(Aircrafts[i]->islanded) && !(Aircrafts[j]->islanded))
			{
				double shift2 = 0;
				double shift1 = 0;
				if (tableAirs[i][i] == 1)
				{
					shift1 = 1000;
					shift2 = -1000;
					tableAirs[j][j] = -1;
				}
				else if (tableAirs[i][i] == -1)
				{
					shift1 = -1000;
					tableAirs[j][j] = 1;
					shift2 = 1000;
				}
				else
				{
					double y1 = Aircrafts[i]->startSK[1];
					double y2 = Aircrafts[j]->startSK[1];

					if (y1 > y2)
					{
						tableAirs[i][i] = 1;
						tableAirs[j][j] = -1;
						shift1 = 1000;
						shift2 = -1000;
					}
					else
					{
						tableAirs[i][i] = -1;
						tableAirs[j][j] = 1;
						shift1 = -1000;
						shift2 = 1000;
					}
					
				}
				
				if (Aircrafts[i]->islanding)
				{
					double y1 = Aircrafts[i]->startSK[1];
					double y2 = Aircrafts[j]->startSK[1];
					shift1 = 0;
					shift2 = 0;
					if (tableAirs[j][j] == 1 && (y2 > y1))
					{
						shift2 = 1000;
					}
					else if (tableAirs[j][j] == -1 && (y2 < y1))
					{
						shift2 = -1000;
					}
					else
					{
						if (y2 > y1)
						{
							tableAirs[j][j] = 1;
							shift2 = 1000;
						}
						else
						{
							tableAirs[j][j] = -1;
							shift2 = -1000;
						}
					}

				}

				if (Aircrafts[j]->islanding)
				{
					double y1 = Aircrafts[i]->startSK[1];
					double y2 = Aircrafts[j]->startSK[1];
					shift1 = 0;
					shift2 = 0;
					if (tableAirs[j][j] == 1 && (y1 > y2))
					{
						shift1 = 1000;
					}
					else if (tableAirs[j][j] == -1 && (y1 < y2))
					{
						shift1 = -1000;
					}
					else
					{
						if (y1 > y2)
						{
							tableAirs[i][i] = 1;
							shift1 = 1000;
						}
						else
						{
							tableAirs[i][i] = -1;
							shift1 = -1000;
						}
					}

				}
				if (!(Aircrafts[i]->islanding))
				{
					Aircrafts[i]->initRound(0, shift1);
				}
				if (!(Aircrafts[j]->islanding))
				{
					Aircrafts[j]->initRound(0, shift2);
				}
				tableAirs[i][j] = 1;
				
			}
			else if (dist >= 2000)
			{
				tableAirs[i][i] = 0;
				tableAirs[j][j] = 0;
				tableAirs[i][j] = 0;
			}
		}
	}
	mutex.unlock();
}

void ORM::checkZones2()
{
	double dist_max = 2000;
	double changedH = 800;
	int n = Aircrafts.size();
	double shift1 = 0;
	double shift2 = 0;

	for (int i = 0; i < n - 1; i++)
	{
		for (int j = i + 1; j < n; j++)
		{
			double dist = tr.getDistance(Aircrafts[i]->startSK, Aircrafts[j]->startSK);
			double y1 = Aircrafts[i]->startSK[1];
			double y2 = Aircrafts[j]->startSK[1];

			if (dist < dist_max && tableAirs[i][j] != 1 && !(Aircrafts[i]->islanded) && !(Aircrafts[j]->islanded))
			{
				double a = 0;
				if (tableAirs[i][i] == 1 && tableAirs[j][j] != -1)
				{
					shift1 = changedH;
					shift2 = 0.9*changedH;
					tableAirs[i][j] = 1;
					tableAirs[i][i] = 1;
					tableAirs[j][j] = 1;
				}
				else if (tableAirs[i][i] == -1  && tableAirs[j][j] != 1)
				{
					shift1 = -changedH;
					shift2 = -0.9*changedH;
					tableAirs[i][j] = 1;
					tableAirs[i][i] = -1;
					tableAirs[j][j] = -1;
				}
				else if (y1 >= y2 && (tableAirs[i][i] != -1 && tableAirs[i][i] != 1) && (tableAirs[j][j] != -1 && tableAirs[j][j] != 1))
				{
					shift1 = changedH;
					shift2 = -changedH;
					tableAirs[i][j] = 1;
					tableAirs[i][i] = 1;
					tableAirs[j][j] = -1;

				}
				else if (y1 <= y2 && (tableAirs[i][i] != -1 && tableAirs[i][i] != 1) && (tableAirs[j][j] != -1 && tableAirs[j][j] != 1))
				{
					shift1 = -changedH;
					shift2 = changedH;
					tableAirs[i][j] = 1;
					tableAirs[i][i] = -1;
					tableAirs[j][j] = 1;

				}
				else if ( (tableAirs[i][i] != -1 && tableAirs[i][i] != 1) && tableAirs[j][j] == 1)
				{
					shift1 = changedH;
					shift2 = 0;
					tableAirs[i][j] = 1;
					tableAirs[i][i] = 1;
					tableAirs[j][j] = 1;
				}
				else if ( (tableAirs[i][i] != -1 && tableAirs[i][i] != 1) && tableAirs[j][j] == -1)
				{
					shift1 = -changedH;
					shift2 = 0;
					tableAirs[i][j] = 1;
					tableAirs[i][i] = -1;
					tableAirs[j][j] = -1;
				}
				else if (y1 < y2 && (tableAirs[i][i] == 1 && tableAirs[j][j] == -1))
				{
					shift1 = 0;
					shift2 = 0;
					tableAirs[i][j] = 1;
					tableAirs[i][i] = 0;
					tableAirs[j][j] = 0;
				}
				else if (y1 > y2 && (tableAirs[i][i] == -1 && tableAirs[j][j] == 1))
				{
					shift1 = 0;
					shift2 = 0;
					tableAirs[i][j] = 1;
					tableAirs[i][i] = 0;
					tableAirs[j][j] = 0;
				}

				if (Aircrafts[i]->islanding)
				{
					double y1 = Aircrafts[i]->startSK[1];
					double y2 = Aircrafts[j]->startSK[1];
					shift1 = 0;
					shift2 = 0;
					if (tableAirs[j][j] == 1 && (y2 > y1))
					{
						shift2 = changedH;
					}
					else if (tableAirs[j][j] == -1 && (y2 < y1))
					{
						shift2 = -changedH;
					}
					else
					{
						if (y2 > y1)
						{
							tableAirs[j][j] = 1;
							shift2 = changedH;
						}
						else
						{
							tableAirs[j][j] = -1;
							shift2 = -changedH;
						}
					}

				}

				if (Aircrafts[j]->islanding)
				{
					double y1 = Aircrafts[i]->startSK[1];
					double y2 = Aircrafts[j]->startSK[1];
					shift1 = 0;
					shift2 = 0;
					if (tableAirs[j][j] == 1 && (y1 > y2))
					{
						shift1 = changedH;
					}
					else if (tableAirs[j][j] == -1 && (y1 < y2))
					{
						shift1 = -changedH;
					}
					else
					{
						if (y1 > y2)
						{
							tableAirs[i][i] = 1;
							shift1 = changedH;
						}
						else
						{
							tableAirs[i][i] = -1;
							shift1 = -changedH;
						}
					}

				}
				if (!(Aircrafts[i]->islanding))
				{
					Aircrafts[i]->initRound(0, shift1);
				}
				if (!(Aircrafts[j]->islanding))
				{
					Aircrafts[j]->initRound(0, shift2);
				}
			}
			else if (dist >= dist_max)
			{
			tableAirs[i][i] = 0;
			tableAirs[j][j] = 0;
			tableAirs[i][j] = 0;
			}
		}
	}

}
