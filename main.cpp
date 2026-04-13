#include <QCoreApplication>
#include <QDebug>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    qDebug() << "Test";
    // 在这里添加调试断点
    return QCoreApplication::exec();
}
