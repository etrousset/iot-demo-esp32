#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtBluetooth/QBluetoothDeviceDiscoveryAgent>

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public slots:
    void bleDeviceFound(const QBluetoothDeviceInfo& device);
    void bleScanFinished();
    void bleScanError(QBluetoothDeviceDiscoveryAgent::Error error);

    void onBleDeviceDblClicked(const QModelIndex &index);

  public:
    explicit MainWindow(QWidget* parent = 0);
    ~MainWindow();

  private:
    Ui::MainWindow* ui;

    QBluetoothDeviceDiscoveryAgent* m_bleDiscoveryAgent;
};

#endif // MAINWINDOW_H
