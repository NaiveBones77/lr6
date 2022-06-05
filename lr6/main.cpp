#pragma once
#include "Conversion.h"
#include "INS.h"
#include "protocol.h"
#include "SNS.h"
#include "Timer.cpp"

#include <WinSock2.h>
#include "Aircraft.h"
#include "ORM.h"
#include "integrator.h"
#include "Transition.h"
#pragma comment(lib,"Ws2_32.lib")

#define PORT 12346
#define SERVERADDR "127.0.0.1"
#pragma warning(disable: 4996)

SOCKET _s;
sockaddr_in _destAddr;

void test_run(Aircraft& a)
{
    a.run3();
}

void test_check(ORM& A)
{
    A.checkAircrafts();
}

void test_zones(ORM& A)
{
    A.checkZones();
}


void test_ops(Aircraft& a)
{
    a.OPS2();
}

void test_bomb(Aircraft& a)
{
    a.startBomb();
}


int main()
{
    setlocale(LC_ALL, "rus");



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

    char buff[10 * 1014];

    if (WSAStartup(0x202, (WSADATA*)&buff[0]))
    {
        printf("WSAStartup error: %d\n",
            WSAGetLastError());
        return -1;
    }

    _s = socket(AF_INET, SOCK_DGRAM, 0);
    if (_s == INVALID_SOCKET)
    {
        printf("socket() error: %d\n", WSAGetLastError());
        WSACleanup();
        return -1;
    }

    HOSTENT* hst;
    _destAddr;
    _destAddr.sin_family = AF_INET;
    _destAddr.sin_port = htons(PORT);

    if (inet_addr(SERVERADDR))
        _destAddr.sin_addr.s_addr = inet_addr(SERVERADDR);

    else
    {
        if (hst = gethostbyname(SERVERADDR))
            _destAddr.sin_addr.s_addr = ((unsigned long**)hst->h_addr_list)[0][0];

        else
        {
            printf("Unknown host: %d\n", WSAGetLastError());
            closesocket(_s);
            WSACleanup();
            return -1;
        }
    }



    Timer timer;

    Transition tr;

    std::vector<double> x01 = { -10000, 5000, 7000 };
    std::vector<double> x02 = { -12204, 5000, -206 };
    std::vector<double> x03 = { -10000, 6000, 15000 };
    std::vector<double> x04 = { -8000, 3000, 9000 };
    std::vector<double> x05 = { 6000, 4000, 14000 };
    std::vector<double> x06 = { -10000, 5000, 10000 };
    std::vector<double> x07 = { 13000, 6000, 11000 };
    std::vector<double> x08 = { 12000, 7000, 10000 };
    std::vector<std::vector<double>> glissade = tr.getGlissade(3000, 2);

    Aircraft* a1 = new Aircraft(x01, 200, 0, 1);
    Aircraft* a2 = new Aircraft(x02, 400, 0, 2);
    Aircraft* a3 = new Aircraft(x03, 240, 0, 3);
    Aircraft* a4 = new Aircraft(x04, 250, 0, 4);
    Aircraft* a5 = new Aircraft(x05, 240, 0, 5);
    Aircraft* a6 = new Aircraft(x06, 230, 0, 6);
    Aircraft* a7 = new Aircraft(x07, 220, 0, 7);
    Aircraft* a8 = new Aircraft(x08, 200, 0, 8);
    std::vector<Aircraft*> A;
    A.push_back(a1);
    A.push_back(a2);
    A.push_back(a3);
    A.push_back(a4);
    A.push_back(a5);
    A.push_back(a6);
    A.push_back(a7);
    A.push_back(a8);
    ORM Orm(A);

    a1->bindPort(_s, _destAddr);
    a2->bindPort(_s, _destAddr);
    a3->bindPort(_s, _destAddr);
    a4->bindPort(_s, _destAddr);
    a5->bindPort(_s, _destAddr);
    a6->bindPort(_s, _destAddr);
    a7->bindPort(_s, _destAddr);
    a8->bindPort(_s, _destAddr);


    timer.add(std::chrono::microseconds(100), [&]() {test_run(*a1); });
    timer.add(std::chrono::microseconds(100), [&]() {test_ops(*a1); });

    timer.add(std::chrono::microseconds(100), [&]() {test_run(*a2); });
    timer.add(std::chrono::microseconds(100), [&]() {test_ops(*a2); });

    timer.add(std::chrono::microseconds(100), [&]() {test_run(*a3); });
    timer.add(std::chrono::microseconds(100), [&]() {test_ops(*a3); });

    timer.add(std::chrono::microseconds(100), [&]() {test_run(*a4); });
    timer.add(std::chrono::microseconds(100), [&]() {test_ops(*a4); });

    timer.add(std::chrono::microseconds(100), [&]() {test_run(*a5); });
    timer.add(std::chrono::microseconds(100), [&]() {test_ops(*a5); });

    timer.add(std::chrono::microseconds(100), [&]() {test_run(*a6); });
    timer.add(std::chrono::microseconds(100), [&]() {test_ops(*a6); });

    timer.add(std::chrono::microseconds(100), [&]() {test_run(*a7); });
    timer.add(std::chrono::microseconds(100), [&]() {test_ops(*a7); });

    timer.add(std::chrono::microseconds(100), [&]() {test_run(*a8); });
    timer.add(std::chrono::microseconds(100), [&]() {test_ops(*a8); });

    timer.add(std::chrono::microseconds(10), [&]() {test_check(Orm); });
    timer.add(std::chrono::microseconds(10), [&]() {test_zones(Orm); });

    // timer.add(std::chrono::microseconds(1500), [&]() {test_bomb(a1); });

    int i = 0;
    while (true) { std::this_thread::sleep_for(std::chrono::seconds(30));/* i++;*/ };

}