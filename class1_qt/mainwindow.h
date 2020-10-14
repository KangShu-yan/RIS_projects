#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QProgressBar>


#include <QGraphicsView>    //graph
#include <QtSerialPort/QSerialPort> //serial
#include <QtSerialPort/QSerialPortInfo>

#include <QTextStream>  //read and write
#include <QTimer>
#include <QByteArray>
#include <QObject>

#include <QStandardItemModel>
#include <QTableView>
#include <QThread>
#include <QMetaType>
#include <QQueue>
#include <QMessageBox>
#include <QTimer>


#include "form.h"       //
#include "usb_communication.h"
#include "msg.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void serial_init();

public slots:
//    void slot_btn_click(bool);
    void slot_btn2_click(bool);
    void slot_checkbox_state_changed(int);
    void slot_sliderbar_value_changed(int);
    void slot_close_and_open();
    void slot_table_update();
    void slot_tableview_update(const Msg);
    void slot_serial_read();
    void slot_time_update();

private slots:
    void on_open_new_btn_clicked();
    void on_checkBox_2_stateChanged(int arg1);
    void on_open_dev_button_clicked();
    void on_send_button_clicked();
    void on_send_CAN_btn_clicked();

signals:
    void signal_analyse();
private:
    Ui::MainWindow *ui;
    QPushButton* btn2;
    QStandardItemModel* table_view ;
    QSerialPort serial_port;
    UsbCommunication* usb_com;  //thread
    QQueue<Msg> queue_msg;
    Msg* msg;
    QQueue<QByteArray> mrec_buffer;
    QTimer* m_timer;
    QByteArray rec_buffer;

};

//Q_DECLARE_METATYPE(struct Msg)
#endif // MAINWINDOW_H
