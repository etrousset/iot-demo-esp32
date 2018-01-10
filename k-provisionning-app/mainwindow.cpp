#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QBluetoothUuid>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_bleDeviceDiscoveryAgent = new QBluetoothDeviceDiscoveryAgent();
    m_bleDeviceDiscoveryAgent->setLowEnergyDiscoveryTimeout(10000);

    connect(m_bleDeviceDiscoveryAgent, &QBluetoothDeviceDiscoveryAgent::deviceDiscovered, this, &MainWindow::bleDeviceFound);
    connect(m_bleDeviceDiscoveryAgent,
            static_cast<void (QBluetoothDeviceDiscoveryAgent::*)(QBluetoothDeviceDiscoveryAgent::Error)>(
                &QBluetoothDeviceDiscoveryAgent::error),
            this,
            &MainWindow::bleScanError);
    connect(m_bleDeviceDiscoveryAgent, &QBluetoothDeviceDiscoveryAgent::finished, this, &MainWindow::bleScanFinished);
    connect(m_bleDeviceDiscoveryAgent, &QBluetoothDeviceDiscoveryAgent::canceled, this, &MainWindow::bleScanFinished);
    m_bleDeviceDiscoveryAgent->start(QBluetoothDeviceDiscoveryAgent::LowEnergyMethod);

    connect(ui->bleDeviceList, &QListWidget::doubleClicked, this, &MainWindow::onBleDeviceDblClicked);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::bleDeviceFound(const QBluetoothDeviceInfo& device)
{
    // If device is LowEnergy-device, add it to the list
    if (device.coreConfigurations() & QBluetoothDeviceInfo::LowEnergyCoreConfiguration) {
        qDebug() << "Found device:" << device.name() << ", addr: " << device.address().toString();
        QListWidgetItem* deviceItem = new QListWidgetItem(device.name(), ui->bleDeviceList);
        deviceItem->setData(Qt::UserRole, device.address().toString());

        QList<QBluetoothUuid> l = device.serviceUuids();
        for (int i = 0; i <l.length(); i++) {
            QBluetoothUuid serviceUuid = device.serviceUuids().at(i);
            qDebug() << "Service: "<<serviceUuid.toString();
        }
        if(device.serviceUuidsCompleteness() == QBluetoothDeviceInfo::DataComplete){
            qDebug()<< "Service list is COMPLETE";
        }
        else {
            qDebug()<< "Service list is INCOMPLETE";
        }
    }
}

void MainWindow::bleScanFinished()
{
    qDebug() << "BLE scan finished...";
}

void MainWindow::bleScanError(QBluetoothDeviceDiscoveryAgent::Error error)
{
    qDebug() << "BLE scan error: " << m_bleDeviceDiscoveryAgent->errorString();
}

void MainWindow::onBleDeviceDblClicked(const QModelIndex& index)
{
    qDebug() << "Device Addr: " <<  index.data(Qt::UserRole).toString();


}
