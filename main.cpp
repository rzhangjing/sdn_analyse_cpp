#include <QCoreApplication>
#include <iostream>
#include <string>
#include <windows.h>
#include <QDebug>
#include "network_deployment.h"
#include "network_bandwidth_allocation.h"
#include "network_resource_allocation.h"

int main(int argc, char *argv[])
{
    // 设置控制台 UTF-8 编码
    SetConsoleOutputCP(CP_UTF8);
    QCoreApplication app(argc, argv);

    std::cout << "\n正在执行网络部署延迟分析..." << std::endl;
    network_deployment::networkDeployment();
    std::cout << "\n网络部署延迟分析完成。" << std::endl;
        
    std::cout << "\n正在执行网络带宽分配能力测试..." << std::endl;
    //network_bandwidth_allocation::networkBandwidthAllocationCapabilityWork();
    std::cout << "\n网络带宽分配能力测试完成。" << std::endl;

    std::cout << "\n正在执行网络资源分配 EECN 构建..." << std::endl;
    //network_resource_allocation::networkAllocation();
    std::cout << "\n网络资源分配 EECN 构建完成。" << std::endl;

    return 0;
}
