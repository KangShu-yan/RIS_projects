#include "usb_communication.h"
//#include "ui_form.h"


char buffer[20] = {0};

UsbCommunication::UsbCommunication(QObject *parent) :
    QThread(parent)
{
    msg.read_count=0;
    msg.page_address=1;
    msg.meaning="meaning";
    msg.value=1000;
}

UsbCommunication::~UsbCommunication()
{
//    delete ui;
}
void UsbCommunication::slot_analyse_data()
{
    qDebug()<<"slot_analyse_data";
}
void UsbCommunication::run()
{
    int while_count=0;
    while(1)
    {
        if(while_count>1000)
        {
            while_count=0;
            ++msg.read_count;
            emit signal_get_msg(msg);
        }
//        emit signal_serial_read();
//        sleep(1);
        ++while_count;
        usleep(5000);   //1ms
    }
//    this->hide();
}


