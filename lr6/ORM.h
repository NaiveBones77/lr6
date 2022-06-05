#pragma once
#include <vector>
#include "Transition.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <list>
#include "integrator.h"
#include "Aircraft.h"
#include <string>

class ORM
{
private:
	Transition tr;
	std::vector<Aircraft*> Aircrafts;
	std::vector <std::vector<double>> gliss = tr.getGlissade(3000, 2);
	int** tableAirs;  // матрица, которая хранит состояния текущего полета ЛА и взаимные режимы полета для двух ЛА

public:
	ORM(std::vector<Aircraft*> Aircrafts);
	void checkAircrafts();
	void checkZones();
	void isLanding();

};