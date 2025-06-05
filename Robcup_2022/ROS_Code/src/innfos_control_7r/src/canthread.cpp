#include <iostream>
#include "canthread.h"
// #include "actuatorInterface.h"
#include "ftsensorInterface.h"
#include <string>
#include <chrono>
using namespace std;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;

double GetTimeValue(high_resolution_clock::time_point t1)
{
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    double ret = time_span.count();
    return ret;
}

CANThread::CANThread()
{
    stopped = false;
    // qRegisterMetaType<VCI_CAN_OBJ>("VCI_CAN_OBJ");
    // qRegisterMetaType<unsigned int>("DWORD");
}

void CANThread::stop()
{
    unsigned int dwRel;
    VCI_BOARD_INFO pInfo[2];
    dwRel = VCI_FindUsbDevice2(pInfo);
    stopped = true;
}

// 1.打开设备
bool CANThread::openDevice(UINT debicIndex)
{
    // deviceType = deviceType;/* USBCAN-2A或USBCAN-2C或CANalyst-II */
    //  debicIndex = debicIndex;/* 第1个设备 */
    // baundRate = baundRate;
    unsigned int dwRel;
    cout << "open device" << endl;
    dwRel = VCI_OpenDevice(deviceType, debicIndex, 0);
    if (dwRel != 1)
    {
        cout << "open failure!" << endl;
        return false;
    }
    else
    {
        cout << "open success!" << endl;
        return true;
    }
}

// 2.初始化CAN
bool CANThread::initCAN(UINT debicIndex, UINT baundRate)
{
    cout << "initialize CAN" << endl;
    unsigned int dwRel = VCI_ClearBuffer(deviceType, debicIndex, 0);
    dwRel = VCI_ClearBuffer(deviceType, debicIndex, 1);
    cout << "ClearBuffer"
         << "number of buffer to clear: " << dwRel << endl;

    VCI_INIT_CONFIG vic;
    vic.AccCode = 0x80000008; // 默认值见手册，勿修改
    vic.AccMask = 0xFFFFFFFF; // 默认值见手册，勿修改
    vic.Filter = 1;           // 接受所有帧
    vic.Mode = 0;             // 正常模式
    // ti5robot电机得设1000kbps
    switch (baundRate)
    {
    case 10:
        vic.Timing0 = 0x31;
        vic.Timing1 = 0x1c;
        break;
    case 20:
        vic.Timing0 = 0x18;
        vic.Timing1 = 0x1c;
        break;
    case 40:
        vic.Timing0 = 0x87;
        vic.Timing1 = 0xff;
        break;
    case 50:
        vic.Timing0 = 0x09;
        vic.Timing1 = 0x1c;
        break;
    case 80:
        vic.Timing0 = 0x83;
        vic.Timing1 = 0xff;
        break;
    case 100:
        vic.Timing0 = 0x04;
        vic.Timing1 = 0x1c;
        break;
    case 125:
        vic.Timing0 = 0x03;
        vic.Timing1 = 0x1c;
        break;
    case 200:
        vic.Timing0 = 0x81;
        vic.Timing1 = 0xfa;
        break;
    case 250:
        vic.Timing0 = 0x01;
        vic.Timing1 = 0x1c;
        break;
    case 400:
        vic.Timing0 = 0x80;
        vic.Timing1 = 0xfa;
        break;
    case 500:
        vic.Timing0 = 0x00;
        vic.Timing1 = 0x1c;
        break;
    case 666:
        vic.Timing0 = 0x80;
        vic.Timing1 = 0xb6;
        break;
    case 800:
        vic.Timing0 = 0x00;
        vic.Timing1 = 0x16;
        break;
    case 1000:
        vic.Timing0 = 0x00;
        vic.Timing1 = 0x14;
        break;
    case 33:
        vic.Timing0 = 0x09;
        vic.Timing1 = 0x6f;
        break;
    case 66:
        vic.Timing0 = 0x04;
        vic.Timing1 = 0x6f;
        break;
    case 83:
        vic.Timing0 = 0x03;
        vic.Timing1 = 0x6f;
        break;
    default:
        break;
    }
    dwRel = VCI_InitCAN(deviceType, debicIndex, 0, &vic);
    if (dwRel == 1)
    {
        cout << "initialize CAN channel 1 successfully！" << endl;
    }
    else
    {
        cout << "iinitialize CAN channel 1 failedly！" << endl;
        return false;
    }
    dwRel = VCI_InitCAN(deviceType, debicIndex, 1, &vic);
    if (dwRel == 1)
    {
        cout << "initialize CAN channel 2 successfully！" << endl;
    }
    else
    {
        cout << "iinitialize CAN channel 2 failedly！" << endl;
        return false;
    }

    VCI_BOARD_INFO vbi;
    dwRel = VCI_ReadBoardInfo(deviceType, debicIndex, &vbi);

    if (dwRel == 1)
    {
        printf(">>Get VCI_ReadBoardInfo success!\n");

        // printf(">>Serial_Num:%c", vbi.str_Serial_Num[0]);
        // printf("%c", vbi.str_Serial_Num[1]);
        // printf("%c", vbi.str_Serial_Num[2]);
        // printf("%c", vbi.str_Serial_Num[3]);
        // printf("%c", vbi.str_Serial_Num[4]);
        // printf("%c", vbi.str_Serial_Num[5]);
        // printf("%c", vbi.str_Serial_Num[6]);
        // printf("%c", vbi.str_Serial_Num[7]);
        // printf("%c", vbi.str_Serial_Num[8]);
        // printf("%c", vbi.str_Serial_Num[9]);
        // printf("%c", vbi.str_Serial_Num[10]);
        // printf("%c", vbi.str_Serial_Num[11]);
        // printf("%c", vbi.str_Serial_Num[12]);
        // printf("%c", vbi.str_Serial_Num[13]);
        // printf("%c", vbi.str_Serial_Num[14]);
        // printf("%c", vbi.str_Serial_Num[15]);
        // printf("%c", vbi.str_Serial_Num[16]);
        // printf("%c", vbi.str_Serial_Num[17]);
        // printf("%c", vbi.str_Serial_Num[18]);
        // printf("%c", vbi.str_Serial_Num[19]);printf("\n");

        // printf(">>hw_Type:%c", vbi.str_hw_Type[0]);
        // printf("%c", vbi.str_hw_Type[1]);
        // printf("%c", vbi.str_hw_Type[2]);
        // printf("%c", vbi.str_hw_Type[3]);
        // printf("%c", vbi.str_hw_Type[4]);
        // printf("%c", vbi.str_hw_Type[5]);
        // printf("%c", vbi.str_hw_Type[6]);
        // printf("%c", vbi.str_hw_Type[7]);
        // printf("%c", vbi.str_hw_Type[8]);
        // printf("%c", vbi.str_hw_Type[9]);printf("\n");

        // printf(">>Firmware Version:V");
        // printf("%x", (vbi.fw_Version&0xF00)>>8);
        // printf(".");
        // printf("%x", (vbi.fw_Version&0xF0)>>4);
        // printf("%x", vbi.fw_Version&0xF);
        // printf("\n");
    }

    else
        printf(">>Get VCI_ReadBoardInfo failure!\n");
    // else
    //     boardInfo(vbi);
    return true;
}

// 3.启动CAN
bool CANThread::startCAN(UINT debicIndex)
{
    if (VCI_StartCAN(deviceType, debicIndex, 0) != 1)
    {
        printf("start 0 fail.");
        VCI_CloseDevice(deviceType, debicIndex);
        return false;
    }
    else
        printf("start 0 success.");
    if (VCI_StartCAN(deviceType, debicIndex, 1) != 1)
    {
        printf("start 1 fail.");
        VCI_CloseDevice(deviceType, debicIndex);
        return false;
    }
    else
        printf("start 1 success.");
    return true;
}

// 4.发送数据
bool CANThread::sendData(UINT debicIndex, UINT channel, UINT ID, BYTE remoteFlag, BYTE externFlag, const unsigned char *data, BYTE len)
{
    unsigned int dwRel;
    VCI_CAN_OBJ vco;
    vco.ID = ID;
    vco.RemoteFlag = remoteFlag;
    vco.ExternFlag = externFlag;
    vco.DataLen = len;
    for (UINT j = 0; j < len; j++)
        vco.Data[j] = data[j];
    dwRel = VCI_Transmit(deviceType, debicIndex, channel, &vco, 1);
    if (dwRel > 0)
        return true;
    else
        return false;
}
bool CANThread::sendData_batch(UINT debicIndex, UINT channel, UINT *ID, BYTE remoteFlag, BYTE externFlag, unsigned char **data, BYTE len, INT batch)
{
    unsigned int dwRel;
    VCI_CAN_OBJ vco[batch];
    for (int i = 0; i < batch; i++)
    {
        vco[i].ID = ID[i];
        vco[i].RemoteFlag = remoteFlag;
        vco[i].ExternFlag = externFlag;
        vco[i].DataLen = len;
        for (UINT j = 0; j < len; j++)
            vco[i].Data[j] = data[i][j];
    }
    dwRel = VCI_Transmit(deviceType, debicIndex, channel, vco, batch);
    if (dwRel > 0)
        return true;
    else
        return false;
}

unsigned char *CANThread::receieveData_single(UINT debicIndex, UINT channel, UINT ID) // 单电机 电流/速度/位置单个数据
{
    unsigned int dwRel;
    bool loop_flag = 1;
    VCI_CAN_OBJ vco[1];
    while (loop_flag)
    {
        dwRel = VCI_GetReceiveNum(deviceType, debicIndex, channel);
        if (dwRel == 1)
        {
            loop_flag = false;
            dwRel = VCI_Receive(deviceType, debicIndex, channel, vco, 1, 0);
        }

        if (dwRel == -1)
        {
            cout << "Failed to receive data" << endl;
            loop_flag = false;
            cin.get();
        }
    }

    static struct CanIdData canIdData;
    for (int i = 0; i < 5; i++)
    {
        canIdData.IdData[i] = vco[0].Data[i];
    }

    canIdData.Id = vco[0].ID;

    return canIdData.IdData; // 定义基本结构体，都采用指针传递
}
bool CANThread::receieveData_s8(UINT debicIndex, UINT channel, UINT ID, UCHAR *&canData) // 电流 速度 位置打包数据
{

    unsigned int dwRel;
    VCI_CAN_OBJ vco[1];
    bool loop_flag = 1;
    //    unsigned char str;
    //    unsigned char messageList;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    while (loop_flag)
    {
        if (GetTimeValue(t1) < 0.01) // 当10ms都没接收到数据时可视为超时失败
        {
            dwRel = VCI_GetReceiveNum(deviceType, debicIndex, channel);
            if (dwRel == 1)
            {
                loop_flag = 0;
                dwRel = VCI_Receive(deviceType, debicIndex, channel, vco, 1, 0);
            }
            // cout << "number of VCI_Receive: " << dwRel << endl;
            if (dwRel == -1)
            {
                cout << "Failed to receive data" << endl;
                loop_flag = 0;
                return false;
            }
        }
        else
        {
            cout << "time delay!" << endl;
            loop_flag = 0;
            return false;
        }
    }

    CanIdData canIdData;
    if (ID == vco[0].ID)
    { // cout <<  "ID: " << ID <<  "  vco[0].ID: " << vco[0].ID << endl;
        for (int i = 0; i < 8; i++)
        {
            canIdData.IdData[i] = vco[0].Data[i];
            // cout << hex <<int( vco[0].Data[i]) << "|";
        }
        // cout << endl;
        // 应对偶尔会出现的错误的位置为0的多余帧,出现这种情况重新收一帧
        if ((vco[0].Data[5] == 0x00) && (vco[0].Data[6] == 0x00) && (vco[0].Data[7] == 0x00))
        {
            cout << "数据帧错误" << endl;
            bool sub_loop_flag = 1;
            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            while (sub_loop_flag)
            {
                if (GetTimeValue(t2) < 0.01) // 当10ms都没接收到数据时可视为超时失败
                {
                    dwRel = VCI_GetReceiveNum(deviceType, debicIndex, channel);
                    if (dwRel == 1)
                    {
                        sub_loop_flag = 0;
                        dwRel = VCI_Receive(deviceType, debicIndex, channel, vco, 1, 0);
                        cout << "数据帧错误后重新获取后成功!" << endl;

                        if ((ID == vco[0].ID) && (!((vco[0].Data[5] == 0x00) && (vco[0].Data[6] == 0x00) && (vco[0].Data[7] == 0x00))))
                        {
                            // cout <<  "ID: " << ID <<  "  vco[0].ID: " << vco[0].ID << endl;
                            for (int i = 0; i < 8; i++)
                            {
                                canIdData.IdData[i] = vco[0].Data[i];
                                //    cout << hex <<int( vco[0].Data[i]) << "|";
                            }
                            // cout << endl;
                        }
                        else
                        {
                            cout << "数据帧错误后重新获取后成功!但又存在数据错误或者ID匹配错误" << endl;
                            return false;
                        }
                    }
                    else
                    {
                        cout << "数据帧错误后重新获取，但超时!" << endl;
                        sub_loop_flag = 0;
                        return false;
                    }
                }
            }
        }

        unsigned char *canIdDataIdData = new unsigned char[8];
        canIdData.Id = vco[0].ID;
        for (int i = 0; i < 8; i++)
        {
            canIdDataIdData[i] = canIdData.IdData[i];
        }
        canData = canIdDataIdData;
        return true; // 定义基本结构体，都采用指针传递
    }
    else
    {
        cout << "ID:" << ID << "id数据对应错误!"
             << "错误ID: " << vco[0].ID << endl;
        bool sub_loop_flag = 1;
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        while (sub_loop_flag)
        {
            if (GetTimeValue(t2) < 0.01) // 当10ms都没接收到数据时可视为超时失败
            {
                dwRel = VCI_GetReceiveNum(deviceType, debicIndex, channel);
                if (dwRel == 1)
                {
                    sub_loop_flag = 0;
                    dwRel = VCI_Receive(deviceType, debicIndex, channel, vco, 1, 0);
                    cout << "id数据对应后重新获取成功!!" << endl;
                    if ((ID == vco[0].ID) && (!((vco[0].Data[5] == 0x00) && (vco[0].Data[6] == 0x00) && (vco[0].Data[7] == 0x00))))
                    {
                        // cout <<  "ID: " << ID <<  "  vco[0].ID: " << vco[0].ID << endl;
                        for (int i = 0; i < 8; i++)
                        {
                            canIdData.IdData[i] = vco[0].Data[i];
                            // cout << hex <<int( vco[0].Data[i]) << "|";
                        }
                        // cout << endl;
                    }
                    else
                    {
                        cout << "id数据对应错误后重新获取后成功!但又存在数据错误或者ID匹配错误" << endl;
                        return false;
                    }
                }
            }
            else
            {
                cout << "id数据对应后重新获取，但超时!" << endl;
                sub_loop_flag = 0;
                return false;
            }
        }
        unsigned char *canIdDataIdData = new unsigned char[8];
        canIdData.Id = vco[0].ID;
        for (int i = 0; i < 8; i++)
        {
            canIdDataIdData[i] = canIdData.IdData[i];
        }
        canData = canIdDataIdData;
        return true; // 定义基本结构体，都采用指针传递
        // return false;
    }
}

bool CANThread::receieveData_s8_thread(UINT debicIndex, UINT channel, UINT ID, UCHAR (&canData)[8]) // 电流 速度 位置打包数据
{

    unsigned int dwRel;
    VCI_CAN_OBJ vco[1];
    bool loop_flag = 1;
    //    unsigned char str;
    //    unsigned char messageList;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    while (loop_flag)
    {
        if (GetTimeValue(t1) < 0.01) // 当10ms都没接收到数据时可视为超时失败
        {
            dwRel = VCI_GetReceiveNum(deviceType, debicIndex, channel);
            if (dwRel == 1)
            {
                loop_flag = 0;
                dwRel = VCI_Receive(deviceType, debicIndex, channel, vco, 1, 0);
            }
            // cout << "number of VCI_Receive: " << dwRel << endl;
            if (dwRel == -1)
            {
                cout << "Failed to receive data" << endl;
                loop_flag = 0;
                return false;
            }
        }
        else
        {
            cout << "time delay!" << endl;
            loop_flag = 0;
            return false;
        }
    }

    CanIdData canIdData;
    if (ID == vco[0].ID)
    { // cout <<  "ID: " << ID <<  "  vco[0].ID: " << vco[0].ID << endl;
        for (int i = 0; i < 8; i++)
        {
            canIdData.IdData[i] = vco[0].Data[i];
            // cout << hex <<int( vco[0].Data[i]) << "|";
        }
        // cout << endl;
        // 应对偶尔会出现的错误的位置为0的多余帧,出现这种情况重新收一帧
        if ((vco[0].Data[5] == 0x00) && (vco[0].Data[6] == 0x00) && (vco[0].Data[7] == 0x00))
        {
            cout << "数据帧错误" << endl;
            bool sub_loop_flag = 1;
            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            while (sub_loop_flag)
            {
                if (GetTimeValue(t2) < 0.01) // 当10ms都没接收到数据时可视为超时失败
                {
                    dwRel = VCI_GetReceiveNum(deviceType, debicIndex, channel);
                    if (dwRel == 1)
                    {
                        sub_loop_flag = 0;
                        dwRel = VCI_Receive(deviceType, debicIndex, channel, vco, 1, 0);
                        cout << "数据帧错误后重新获取后成功!" << endl;

                        if ((ID == vco[0].ID) && (!((vco[0].Data[5] == 0x00) && (vco[0].Data[6] == 0x00) && (vco[0].Data[7] == 0x00))))
                        {
                            // cout <<  "ID: " << ID <<  "  vco[0].ID: " << vco[0].ID << endl;
                            for (int i = 0; i < 8; i++)
                            {
                                canIdData.IdData[i] = vco[0].Data[i];
                                // cout << hex <<int( vco[0].Data[i]) << "|";
                            }
                            // cout << endl;
                        }
                        else
                        {
                            cout << "数据帧错误后重新获取后成功!但又存在数据错误或者ID匹配错误" << endl;
                            return false;
                        }
                    }
                    else
                    {
                        cout << "数据帧错误后重新获取，但超时!" << endl;
                        sub_loop_flag = 0;
                        return false;
                    }
                }
            }
        }
        canIdData.Id = vco[0].ID;
        for (int i = 0; i < 8; i++)
        {
            canData[i] = canIdData.IdData[i];
        }
        return true; // 定义基本结构体，都采用指针传递
    }
    else
    {
        cout << "ID:" << ID << "id数据对应错误!"
             << "错误ID: " << vco[0].ID << endl;
        bool sub_loop_flag = 1;
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        while (sub_loop_flag)
        {
            if (GetTimeValue(t2) < 0.01) // 当10ms都没接收到数据时可视为超时失败
            {
                dwRel = VCI_GetReceiveNum(deviceType, debicIndex, channel);
                if (dwRel == 1)
                {
                    sub_loop_flag = 0;
                    dwRel = VCI_Receive(deviceType, debicIndex, channel, vco, 1, 0);
                    cout << "id数据对应后重新获取成功!!" << endl;
                    if ((ID == vco[0].ID) && (!((vco[0].Data[5] == 0x00) && (vco[0].Data[6] == 0x00) && (vco[0].Data[7] == 0x00))))
                    {
                        // cout <<  "ID: " << ID <<  "  vco[0].ID: " << vco[0].ID << endl;
                        for (int i = 0; i < 8; i++)
                        {
                            canIdData.IdData[i] = vco[0].Data[i];
                            // cout << hex <<int( vco[0].Data[i]) << "|";
                        }
                        // cout << endl;
                    }
                    else
                    {
                        cout << "id数据对应错误后重新获取后成功!但又存在数据错误或者ID匹配错误" << endl;
                        return false;
                    }
                }
            }
            else
            {
                cout << "id数据对应后重新获取，但超时!" << endl;
                sub_loop_flag = 0;
                return false;
            }
        }
        canIdData.Id = vco[0].ID;
        for (int i = 0; i < 8; i++)
        {
            canData[i] = canIdData.IdData[i];
        }
        return true; // 定义基本结构体，都采用指针传递
        // return false;
    }
}

unsigned char (*CANThread::receieveData_s8batch(UINT debicIndex, UINT channel, INT batch))[8] // 批量获取 电流 速度 位置打包数据
{
    unsigned int dwRel;
    VCI_CAN_OBJ vco[batch];
    //    unsigned char str;
    //    unsigned char messageList;

    dwRel = VCI_Receive(deviceType, debicIndex, channel, vco, batch, 0);

    // cout << "number of VCI_Receive: " << dwRel << endl;
    if (dwRel == -1)
    {
        cout << "Failed to receive data" << endl;
    }

    CanIdData canIdData[9];
    static BYTE IdData[9][8];

    for (int i = 0; i < batch; i++)
    {
        canIdData[i].Id = vco[i].ID;
        int idon = canIdData[i].Id % 10 - 1;
        cout << "i: " << i << "id: " << vco[i].ID << "idon" << idon;

        for (int j = 0; j < 8; j++)
        {
            canIdData[i].IdData[j] = vco[i].Data[j];
            IdData[idon][j] = canIdData[i].IdData[j];
        }
    }
    return IdData; // 采用指针传递
}

unsigned char **CANThread::receieveData(UINT debicIndex, UINT channel, UINT *ID) // 批量获取数据
{

    unsigned int dwRel;
    unsigned int num_rec = 0;
    bool loop_flag = 1;
    VCI_CAN_OBJ vco[2500];
    //    unsigned char str;
    //    unsigned char messageList;
    while (loop_flag)
    {
        dwRel = VCI_GetReceiveNum(deviceType, debicIndex, channel);
        if (dwRel > 0)
        {
            num_rec += dwRel;
            if (num_rec == sizeof(ID) / sizeof(ID[0])) // 当获取的帧且未被读取的数目==ID数组长度时，读取接收的所有帧
            {
                dwRel = VCI_Receive(deviceType, debicIndex, channel, vco, 2500, 0);
                loop_flag = 0;
            }
        }
        if (dwRel == -1)
        {
            cout << "Failed to receive data" << endl;
            loop_flag = 0;
        }
    }

    VCI_CAN_OBJ *vci = vco;
    struct CanIdData_batch *canIdData_batch;
    for (unsigned int j = 0; j < (sizeof(ID) / sizeof(ID[0])); j++)
    {
        unsigned int i = 0;
        while (i < num_rec)
        {
            if (ID[j] == vci[i].ID) // 需要重新排序一下
            {
                // cout << vci[i].Data << endl;
                canIdData_batch->Id[j] = vci[i].ID;
                canIdData_batch->IdData[j] = vci[i].Data;
                canIdData_batch->IdData[j] = vci[i].Data;
                break;
            }
            i++;
        }
    }
    return canIdData_batch->IdData; // 定义基本结构体，都采用指针传递
}

bool CANThread::receieveData_double(UINT debicIndex, UINT channel, UINT ID, CanIdData_double &candata_double)
{
    unsigned int dwRel;
    bool loop_flag = 1;
    VCI_CAN_OBJ vco[2];
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    while (loop_flag)
    {
        if (GetTimeValue(t1) < 0.01) // 当10ms都没接收到数据时可视为超时失败
        {
            dwRel = VCI_GetReceiveNum(deviceType, debicIndex, channel);
            if (dwRel == 2)
            {
                loop_flag = 0;
                dwRel = VCI_Receive(deviceType, debicIndex, channel, vco, 2, 0);
            }
            if (dwRel == -1)
            {
                cout << "Failed to receive data" << endl;
                loop_flag = 0;
                return false;
            }
        }
        else
        {
            cout << "time delay!" << endl;
            loop_flag = 0;
            return false;
        }
    }
    if ((ID == vco[0].ID) && (ID + 1 == vco[1].ID))
    {
        for (int i = 0; i < 7; i++)
        {
            candata_double.IdData[0][i] = vco[0].Data[i];
            candata_double.IdData[1][i] = vco[1].Data[i];
        }
        // cout << "vci[0].ID:" << vco[0].ID << " | vci[1].ID:"<< vco[1].ID << endl;
        candata_double.Id[0] = vco[0].ID;
        candata_double.Id[1] = vco[1].ID;
        return true; // 定义基本结构体，都采用指针传递
    }
    else
    {
        cout << "ID:" << ID << "id数据对应错误!"
             << "错误ID: " << vco[0].ID << " | " << vco[1].ID << endl;
        return false;
    }
}
bool CANThread::receieveData_doublebatch(UINT debicIndex, UINT channel, UINT *ID, CanIdData_double (&candata_double)[3])
{
    unsigned int dwRel;
    // unsigned int dw;
    bool loop_flag = 1;
    VCI_CAN_OBJ vco[6];
    // VCI_CAN_OBJ vci[6];
    //    unsigned char str;
    //    unsigned char messageList;

    while (loop_flag)
    {
        dwRel = VCI_GetReceiveNum(deviceType, debicIndex, channel);
        //    dw=VCI_Receive(deviceType, debicIndex,channel,vci,6,0);
        if (dwRel == 6)
        {
            loop_flag = false;
            dwRel = VCI_Receive(deviceType, debicIndex, channel, vco, 6, 0);
            // if(num_rec==2*sizeof(ID) / sizeof(ID[0]))//当获取的帧且未被读取的数目==ID数组长度时，读取接收的所有帧
            // {
            //     dwRel=VCI_Receive(deviceType, debicIndex,channel,vco,2500,0);
            //     loop_flag=0;
            // }
        }
        if (dwRel == -1)
        {
            cout << "Failed to receive data" << endl;
            loop_flag = 0;
            return false;
        }
    }
    for (int i = 0; i < 6; i++)
    {
        int idon = vco[i / 2].ID % 20 - 6 * (1 - channel); // 20 22 24/26 28 30
        for (int j = 0; j < 7; j++)
        {
            candata_double[idon].IdData[i % 2][j] = vco[i].Data[j];
        }
        // cout <<"i: " << i <<"id: " << vco[i].ID << "idon" << idon;
    }
    return true;
}

bool CANThread::ifreceieveDatanum(UINT debicIndex, UINT channel, int num)
{
    bool ifreceieved = false;
    unsigned int dwRel = 0;
    for (int i = 0; i < 20; i++)
    {
        dwRel = VCI_GetReceiveNum(deviceType, debicIndex, channel);
        if (dwRel == num)
        {
            ifreceieved = true;
            break;
        }

        if (i > 10)
        {
            this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    return ifreceieved;
}

bool CANThread::ifreceieveData(UINT debicIndex, UINT channel)
{
    unsigned int dwRel;
    bool ifreceieved;
    unsigned int num_rec = 0;
    VCI_CAN_OBJ vco[25];
    //    unsigned char str;
    //    unsigned char messageList;
    // cout << "读数" << endl;
    for (int i = 0; i < 10; i++)
    {
        dwRel = VCI_Receive(deviceType, debicIndex, channel, vco, 25, 0);
        num_rec += dwRel;
        cout << dwRel << endl;
        if (dwRel == 0 & num_rec > 1)
        {
            cout << "Successfully received data" << endl;
            cout << "number of VCI_Receive" << dwRel << endl;
            ifreceieved = true;
            break;
        }
        else if (dwRel == -1)
        {
            cout << "Failed to receive data" << endl;
            ifreceieved = false;
            break;
        }
        else
            ifreceieved = false;
    }
    return ifreceieved;
}
bool CANThread::clearBuffer(UINT debicIndex, UINT channel)
{
    UINT dwRel;
    dwRel = VCI_ClearBuffer(deviceType, debicIndex, channel);
    cout << "清除一次该通道接收Buffer" << endl;
    if (dwRel == 1)
        return true;
    else
        return false;
}

// 5.关闭设备
void CANThread::closeDevice(UINT debicIndex)
{
    VCI_CloseDevice(deviceType, debicIndex);
}

// 0.复位设备，  复位后回到3
bool CANThread::reSetCAN(UINT debicIndex)
{
    if (VCI_ResetCAN(deviceType, debicIndex, 0) != 1)
    {
        printf("reset 0 fail.");
        return false;
    }
    else
        printf("reset 0 success.");
    if (VCI_ResetCAN(deviceType, debicIndex, 1) != 1)
    {
        printf("reset 1 fail.");
        return false;
    }
    else
        printf("reset 1 success.");
    return true;
}

// void CANThread::run()
// {
//     while(!stopped)
//     {
//         unsigned int dwRel;
//         VCI_CAN_OBJ vco[2500];
//         dwRel = VCI_Receive(deviceType, debicIndex, 0, vco,2500,0);
//         if(dwRel > 0)
// //            getProtocolData(vco,dwRel,0);//需要替换成其他函数
//         dwRel = VCI_Receive(deviceType, debicIndex, 1, vco,2500,0);
//         if(dwRel > 0)
// //            getProtocolData(vco,dwRel,1);//需要替换出需要替换成其他函数
//         sleep(30);
//     }
//     stopped = false;
// }
