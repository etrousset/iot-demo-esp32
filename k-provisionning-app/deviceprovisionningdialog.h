#ifndef DEVICEPROVISIONNINGDIALOG_H
#define DEVICEPROVISIONNINGDIALOG_H

#include <QDialog>

namespace Ui {
class DeviceProvisionningDialog;
}

class DeviceProvisionningDialog : public QDialog
{
    Q_OBJECT

public:
    explicit DeviceProvisionningDialog(QWidget *parent = 0);
    ~DeviceProvisionningDialog();

private:
    Ui::DeviceProvisionningDialog *ui;
};

#endif // DEVICEPROVISIONNINGDIALOG_H
