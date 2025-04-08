#include "Mycode.h"
#include <QtWidgets/QApplication>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    
    Mycode w;
    w.setWindowTitle("3D V1.0");
    w.show();
    return a.exec();
}
