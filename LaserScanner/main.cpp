#include "LaserScanner.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    LaserScanner w;
    w.show();
    return a.exec();
}
