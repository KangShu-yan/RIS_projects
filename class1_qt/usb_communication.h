#ifndef USB_COMMUNICATION_H
#define USB_COMMUNICATION_H

//#include <QWidget>

#include <QThread>
#include <QString>
#include <QtSerialPort/QSerialPort>
#include <QTextStream>  //read and write
#include <QMetaType>
#include <QDebug>

#include "msg.h"

//namespace Ui {
//class Form;
//}

class CustomDataType
{
public:
    CustomDataType(){}
    ~CustomDataType(){}
    void show()
    {

    }

};

struct Msg
{
    int read_count;
    int page_address;
    QString meaning;
    int value ;
};
struct RawMsg
{
    int analyse_count;
    QString rec_buffer[100];
};

class UsbCommunication : public QThread
{
    Q_OBJECT
public:
    explicit UsbCommunication(QObject *parent = nullptr);
    ~UsbCommunication();

protected:
    void run();

public slots:
    void slot_analyse_data();
signals:
    void signal_get_msg(Msg msg);
//   void signal_serial_read();
public:
    Msg msg;
//    CustomDataType msg;

};
Q_DECLARE_METATYPE(Msg);
#endif // USB_COMMUNICATION_H
