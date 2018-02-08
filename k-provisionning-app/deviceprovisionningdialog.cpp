#include "deviceprovisionningdialog.h"
#include "ui_deviceprovisionningdialog.h"

DeviceProvisionningDialog::DeviceProvisionningDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DeviceProvisionningDialog)
{
    ui->setupUi(this);
}

DeviceProvisionningDialog::~DeviceProvisionningDialog()
{
    delete ui;
}
