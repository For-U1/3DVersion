#include <QMainWindow>
#include "ui_Mycode.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Mycode; }
QT_END_NAMESPACE

class Mycode : public QMainWindow
{
    Q_OBJECT

public:
    Mycode(QWidget *parent = nullptr);
    ~Mycode();

private:
    Ui::MycodeClass ui;
};
