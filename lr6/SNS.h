
#pragma once
#include <mutex>
#include <WinSock2.h>
#include <iostream>
#include "Conversion.h"
#include <random>
#include "protocol.h"
class SNS
{
public:
		Val H, HDOP, VDOP, trackAngle, curLatitude, curLatitude_pr, curLongitude, curLongitude_pr, delay,
			UTC_high, UTC_low, V_h;
		int Year = 21, Mounth = 12, Day = 15;
		std::mutex mutex;
		char* buffer;
		bool isStart = false;
		SOCKET _s;
		sockaddr_in _destAddr;
		int buff_count, count_nav;
		Conversion conv1;
		std::mt19937 generator;



		// ����������� ������
		SNS(Val H, Val HDOP, Val VDOP, Val trackAngle, Val curLatitude, Val curLatitude_pr,
			Val curLongitude, Val curLongitude_pr, Val delay, Val UTC_high, Val UTC_low,
			Val V_h);
		SNS();


		//����� ���� �������� ����� ���������
		void test();

		//����� ����������
		void preparation();

		//����� ���������
		void navigation();

		//�����, ����������� ������ ���
		void start();

		//����� �������� ������
		void sendPack();

		//����� ����������� ������� �������� ������
		int bindPort(SOCKET s, sockaddr_in destAddr);

		template <class T>
		void fill_buff(T pack);

};

