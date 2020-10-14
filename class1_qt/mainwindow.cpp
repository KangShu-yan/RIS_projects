#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);  //
//    qRegisterMetaType<Msg>("Msg");
    qRegisterMetaType<Msg>("Msg");  //meta declaration

//    serial_port=new QSerialPort;
    //
    ui->device_combo->clear();
    foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
    {
        ui->device_combo->addItem(info.portName());
    }
    serial_init();

//    ui->send_CAN_btn->setText("hello word!");     //
    // sender signals receiver method
//    connect(ui->send_CAN_btn,SIGNAL(clicked(bool)),this,SLOT(slot_btn_click(bool)));

    btn2=new QPushButton;   //class QPushButton inherited class QAbstractButton
    btn2->setText("btn2");
    ui->verticalLayout_2->addWidget(btn2);

    connect(btn2,SIGNAL(clicked(bool)),this,SLOT(slot_btn2_click(bool)));
    //checkbox signal
    connect(ui->checkBox,SIGNAL(stateChanged(int)),this,SLOT(slot_checkbox_state_changed(int)));
    ui->horizontalSlider->setMaximum(20);
    ui->horizontalSlider->setMinimum(0);

    //progressbar
    connect(ui->horizontalSlider,SIGNAL(valueChanged(int)),this,SLOT(slot_sliderbar_value_changed(int)));
    ui->progressBar->setRange(0,100);
    ui->progressBar->setValue(0);

    QComboBox* box1=new QComboBox;
    box1->setMaximumWidth(160);
    box1->addItem("map");
    box1->addItem("map1");
    box1->addItem("map2");
    box1->setEditable(true);
    //
    table_view = new QStandardItemModel();
    table_view->setColumnCount(4);
    table_view->setHeaderData(0,Qt::Horizontal,QString::fromLocal8Bit("序号"));
    table_view->setHeaderData(1,Qt::Horizontal,QString::fromLocal8Bit("页地址"));
    table_view->setHeaderData(2,Qt::Horizontal,QString::fromLocal8Bit("物理含义"));
    table_view->setHeaderData(3,Qt::Horizontal,QString::fromLocal8Bit("当前值"));
    ui->tableView->setModel(table_view);
    msg = new Msg;
    msg->read_count=0;
    msg->page_address =0x55;
    msg->meaning="sk";
    msg->value=199;
//    queue_msg = new QQueue<Msg>;
    queue_msg.enqueue(*msg);
    for(int i=0;i<4;i++)
    {
        table_view->setItem(i,0,new QStandardItem(QString::number(msg->read_count,10)));    //QString::number(1,10),10就指十进制
        table_view->setItem(i,1,new QStandardItem(QString::number(msg->page_address,10)));
        table_view->setItem(i,2,new QStandardItem(msg->meaning));
        table_view->setItem(i,3,new QStandardItem(QString::number(msg->value,10)));
    }
//    table_view->setItem(4,0,new QStandardItem("0x55"));
//    table_view->setItem(4,1,new QStandardItem("0x1111111"));
//    table_view->setItem(4,2,new QStandardItem("0x12121"));
//    table_view->setItem(4,3,new QStandardItem("0x7777"));

    ui->tableView->setModel(table_view);
    ui->tableView->resizeColumnsToContents();
    connect(ui->open_dev_button,SIGNAL(clicked(bool)),this,SLOT(slot_table_update()));
    m_timer = new QTimer();
    connect(m_timer,SIGNAL(timeout()),this,SLOT(slot_time_update()));
    connect(&serial_port,SIGNAL(readyRead()),this,SLOT(slot_serial_read()));

    usb_com = new UsbCommunication();       //thread
    connect(usb_com,SIGNAL(signal_get_msg(Msg)),this,SLOT(slot_tableview_update(const Msg)));
//    connect(usb_com,SIGNAL(signal_serial_read()),this,SLOT(slot_serial_Read()));
    connect(this,SIGNAL(signal_analyse()),usb_com,SLOT(slot_analyse_data()));
    usb_com->start();   //start thread
}

//deconstructor

void MainWindow::slot_sliderbar_value_changed(int value)
{
    qDebug()<<"sliderbar value:"<<value;
    ui->label_slider_value->setText(QString::number(value));
    ui->progressBar->setValue(value);
}
void MainWindow::slot_checkbox_state_changed(int state)
{
    qDebug()<<"checkbox state:"<<state;
}

//void MainWindow::slot_btn_click(bool)
//{
//    ui->pushButton->setText("clicked"); //change from hello world to clicked
////    ui->checkBox->setChecked(true);
//    ui->checkBox->toggle();     //change checkBox
//    ui->checkBox_2->toggle();
//    bool is_check= ui->checkBox->isChecked();
//    qDebug()<<"is_check : "<<is_check;
//}

void MainWindow::slot_btn2_click(bool)
{
    btn2->setText("clicked");
    qDebug()<<"slot_btn2_clicked ";
}
MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::slot_close_and_open()
{
    this->show();
}
void MainWindow::on_open_new_btn_clicked()
{
    Form* f=new Form;
    f->show();
    connect(f,SIGNAL(close_and_open()),this,SLOT(slot_close_and_open()));
    this->hide();
}
void MainWindow::on_checkBox_2_stateChanged(int state)
{
    qDebug()<<"on_checkBox_2_stateChanged"<<state;
}

void MainWindow::slot_table_update()
{
    int readcount = table_view->data(table_view->index(0,0)).toInt();
    table_view->setData(table_view->index(0,0),QString::number(++readcount));
}
 void MainWindow::slot_tableview_update(const Msg new_msg)
 {
    Msg cur_msg;
    queue_msg.enqueue(new_msg);
    qDebug()<<"msg.read_count : "<<new_msg.read_count<<"msg.page_address : "<<new_msg.page_address<<"msg.meaning : "<<new_msg.meaning<<"msg.value : "<<new_msg.value;
    static int idx=0;
    if(idx>60000){idx=0;}
    ++idx;
    cur_msg=queue_msg.dequeue();
    table_view->setData(table_view->index(idx%4,0),QString::number(cur_msg.read_count));
    table_view->setData(table_view->index(idx%4,1),QString::number(cur_msg.page_address));
    table_view->setData(table_view->index(idx%4,2),cur_msg.meaning);
    table_view->setData(table_view->index(idx%4,3),QString::number(idx%cur_msg.value));
    if(cur_msg.read_count%10)
    {
        emit signal_analyse();
    }
//    serial_Read();
 }
void MainWindow::serial_init()
{
    //读取串口信息
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
            // 自动读取串口号添加到端口portBox中
//        QSerialPort serial;
        serial_port.setPort(info);
        if(serial_port.open(QIODevice::ReadWrite))
        {
                ui->device_combo->addItem(info.portName());
                serial_port.close();
        }
    }
        QStringList baudList;   //波特率
        QStringList parityList; //校验位
        QStringList dataBitsList;   //数据位
        QStringList stopBitsList;   //停止位
        // 波特率    //波特率默认选择下拉第三项：9600
        baudList<<"1200"<<"2400"<<"4800"<<"9600"
               <<"14400"<<"19200"<<"38400"<<"56000"
              <<"57600"<<"115200";
        ui->baudrate_combo->addItems(baudList);
        ui->baudrate_combo->setCurrentIndex(9);
        // 校验      //校验默认选择无
        parityList<<"无"<<"奇"<<"偶";
        ui->parity_combo->addItems(parityList);
        ui->parity_combo->setCurrentIndex(0);
        // 数据位      //数据位默认选择8位
        dataBitsList<<"5"<<"6"<<"7"<<"8";
        ui->data_bits_combo->addItems(dataBitsList);
        ui->data_bits_combo->setCurrentIndex(3);
        // 停止位      //停止位默认选择1位
        stopBitsList<<"1"<<"2";
        ui->stop_bits_combo->addItems(stopBitsList);
        ui->stop_bits_combo->setCurrentIndex(0);
}
void MainWindow::slot_serial_read()
{
    static int read_count=0;
//    m_timer->start(1);     //ms
//    if(read_count>50000)
//    {
//        read_count=0;
//    }
     ++read_count;
//    qDebug()<<"read_count : "<<read_count;

    //从缓冲区中读取数据
//    serial_port.
//    if(read_count%30==0)
//    {
//        mrec_buffer.enqueue(serial_port.readAll());
//        qDebug()<<"serial_port.readAll()"<<mrec_buffer.head();
//    }
    //  rec_buffer.append();

        QString buffer_1;
        int a=0;
    //  qDebug()<<"slot_time_update";
    //  m_timer->stop();
        if(read_count%90==0)
        {
//            rec_buffer=mrec_buffer.dequeue();
            rec_buffer=serial_port.readAll();
            if(!rec_buffer.isEmpty())//如果非空说明有数据接收
            {
                qDebug()<<"rec_buffer is not empty";

                //转换成16进制大写
                qDebug()<<"rec_buffer : "<<rec_buffer;
                QString str=rec_buffer.toHex().data();
                qDebug()<<"str : "<<str;
                str=str.toUpper();
                //一个16进制占4位，8位为一字节，所以每两位16进制空一格
                for(int i=0;i<str.length();i+=2)
                {
                    QString str_1 = str.mid (i,2);
                    buffer_1 += str_1;
                    buffer_1 += " ";
                }
                 QString receive;
                //读取之前显示数据
                if(read_count<5000)
                {
                    receive = ui->received_data->toPlainText();
                     //清空显示
                    ui->received_data->clear();
                }
                else
                {
                    read_count=0;
                    //清空显示
                    ui->received_data->clear();
                }
                //重新显示
                if(a==0)
                {
                    receive += QString(rec_buffer);
                    ui->received_data->append(receive);
                }//直接显示
                else
                {
                    receive += QString(buffer_1);
                    ui->received_data->append(receive);
                }//16进制显示
                rec_buffer.clear();
            }
        }

}

void  MainWindow::slot_time_update()
{
    QString buffer_1;
    int a=0;
    qDebug()<<"slot_time_update";
//   m_timer->stop();
    if(!rec_buffer.isEmpty())//如果非空说明有数据接收
    {
        //转换成16进制大写
        QString str=rec_buffer.toHex().data();
        str=str.toUpper();
        //一个16进制占4位，8位为一字节，所以每两位16进制空一格
        for(int i=0;i<str.length();i+=2)
        {
            QString str_1 = str.mid (i,2);
            buffer_1 += str_1;
            buffer_1 += " ";
        }
        //读取之前显示数据
        QString receive = ui->received_data->toPlainText();
        //清空显示
        ui->received_data->clear();
        //重新显示
        if(a==0)
        {
            receive += QString(rec_buffer);
            ui->received_data->append(receive);
        }//直接显示
        else
        {
            receive += QString(buffer_1);
            ui->received_data->append(receive);
        }//16进制显示
    }
    rec_buffer.clear();


}
//void serial::on_sendBox_clicked()
//{   QByteArray Data_1;
//    //获取输入窗口sendData的数据
//    QString Data = ui->sendData->toPlainText();
//    if(c)       {Data+='\r';Data+='\n';}//插入换行
//    if(b==0)    Data_1 = Data.toUtf8();//转换成utf8格式的字节流发送
//    else        Data_1 = QByteArray::fromHex (Data.toLatin1().data());//按十六进制编码发送
//    // 写入发送缓存区
//    SerialPort.write(Data_1);
//}
void MainWindow::on_open_dev_button_clicked()
{
    // 打开串口
    if(ui->open_dev_button->text() == "Open device")
    {
        // 设置串口号
        serial_port.setPortName(ui->device_combo->currentText());
        // 打开串口
        if(serial_port.open(QIODevice::ReadWrite))
        {
            // 设置波特率
            serial_port.setBaudRate(ui->baudrate_combo->currentText().toInt());
            //设置数据位数
            switch(ui->data_bits_combo->currentIndex())
            {
                case 5: serial_port.setDataBits(QSerialPort::Data5); break;
                case 6: serial_port.setDataBits(QSerialPort::Data6); break;
                case 7: serial_port.setDataBits(QSerialPort::Data7); break;
                case 8: serial_port.setDataBits(QSerialPort::Data8); break;
                default: break;
            }
            // 设置校验位
            //SerialPort->setParity(QSerialPort::NoParity);
            //设置奇偶校验
            switch(ui->parity_combo->currentIndex())
            {
                case 0: serial_port.setParity(QSerialPort::NoParity); break;
                case 1: serial_port.setParity(QSerialPort::OddParity); break;
                case 2: serial_port.setParity(QSerialPort::EvenParity); break;
                default: break;
            }
             // 设置流控制
            serial_port.setFlowControl(QSerialPort::NoFlowControl);
                //设置停止位
            switch(ui->stop_bits_combo->currentIndex())
            {

                case 1: serial_port.setStopBits(QSerialPort::OneStop); break;
                case 2: serial_port.setStopBits(QSerialPort::TwoStop); break;
                default: break;

            }
            //打开串口
        }
        else
        {
            QMessageBox::about(NULL, "提示", "串口无法打开\r\n不存在或已被占用");
            return;
        }
        ui->open_dev_button->setText("Close device ");
        //下拉菜单控件使能
        ui->device_combo->setEnabled(false);
        ui->baudrate_combo->setEnabled(false);
        ui->data_bits_combo->setEnabled(false);
        ui->parity_combo->setEnabled(false);
        ui->stop_bits_combo->setEnabled(false);
            //搜索串口按键使能
          //  ui->searchBtn->setEnabled(false);
            //发送按键使能
        ui->send_button->setEnabled(true);

    }
     // 关闭串口
    else
    {
        serial_port.close();
        ui->open_dev_button->setText("Open device");
            //下拉按键使能
        ui->device_combo->setEnabled(true);
        ui->baudrate_combo->setEnabled(true);
        ui->data_bits_combo->setEnabled(true);
        ui->parity_combo->setEnabled(true);
        ui->stop_bits_combo->setEnabled(true);
            //搜索串口按键使能
//            ui->searchBtn->setEnabled(true);
            //发送按键使能
        ui->send_button->setEnabled(false);
    }
}

void MainWindow::on_send_button_clicked()
{

}

void MainWindow::on_send_CAN_btn_clicked()
{
    qDebug()<<"send_CAN_btn";
}
