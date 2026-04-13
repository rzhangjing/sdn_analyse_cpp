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
    
    std::cout << "========== SDN 网络分析工具 ==========" << std::endl;
    std::cout << "1. 网络部署延迟分析" << std::endl;
    std::cout << "2. 网络带宽分配能力测试" << std::endl;
    std::cout << "3. 网络资源分配 EECN 构建" << std::endl;
    std::cout << "请选择功能 (1/2/3): " << std::flush;
    
    std::string choice;
    std::getline(std::cin, choice);
    
    if (choice == "1") {
        std::cout << "\n正在执行网络部署延迟分析..." << std::endl;
        network_deployment::networkDeployment();
        std::cout << "\n网络部署延迟分析完成。" << std::endl;
    }
    else if (choice == "2") {
        std::cout << "\n正在执行网络带宽分配能力测试..." << std::endl;
        network_bandwidth_allocation::networkBandwidthAllocationCapabilityWork();
        std::cout << "\n网络带宽分配能力测试完成。" << std::endl;
    }
    else if (choice == "3") {
        std::cout << "\n正在执行网络资源分配 EECN 构建..." << std::endl;
        network_resource_allocation::networkAllocation();
        std::cout << "\n网络资源分配 EECN 构建完成。" << std::endl;
    }
    else {
        std::cout << "无效选择" << std::endl;
        return 1;
    }
    
    return 0;
}
