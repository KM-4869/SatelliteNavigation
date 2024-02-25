
#include <iostream>
#include<vector>
#include"SPP.h"
#include"RTK.h"
int lambda(int n, int m, const double* a, const double* Q, double* F, double* s);
int main()
{

    OBSERVATION_DATA Rover_ObsData;//流动站观测数据结构体
    OBSERVATION_DATA Base_ObsData;//基站观测数据结构体
    
    vector<EPHEMERIS> GPSEphem;//GPS卫星星历
    vector<EPHEMERIS> BDSEphem;//BDS卫星星历

    vector<SATPOSRES> Rover_SatPosRes;//流动站卫星位置结果
    vector<SATPOSRES> Base_SatPosRes;//基站卫星位置结果

    DECODEPOS Rover_Pos;//解码所得流动站位置
    DECODEPOS Base_Pos;//解码所得基站位置
    
    STAPOSRES Rover_StaPosRes;//流动站定位结果
    STAPOSRES Base_StaPosRes;//基站定位结果

    NonAteloem7DataLoader RoverDataLoader;//流动站数据加载器
    NonAteloem7DataLoader BaseDataLoader;//基站数据加载器

    unsigned char Roverbuff[MaxBuffLen];//定义流动站解码缓冲区
    unsigned char Basebuff[MaxBuffLen];//定义基站解码缓冲区
    int RoverRemLen = 0;//定义流动站解码后剩余长度
    int BaseRemLen = 0;//定义基站解码后剩余长度

    SD_OBSERVATION_DATA Sd_ObsData;//单差观测数据结构体
    DD_DATA Dd_Data;//双差有关数据结构体


    int RunMode;//运行模式 （1=文件，2=实时）
    cout << "请选择运行模式（1=文件，2=实时）\n";
    cin >> RunMode;

    RoverDataLoader.setRunMode(RunMode);
    BaseDataLoader.setRunMode(RunMode);
    
    RoverDataLoader.SetFileAddress("D:\\卫星导航算法与程序设计\\short-baseline\\oem719-202203170900-2.bin");/*\\Zero-baseline\\oem719-202203031500-2.bin*/ /*\\short-baseline\\oem719-202203170900-2.bin*/
    RoverDataLoader.SetSocketAddress("8.140.46.126", 3002);
    BaseDataLoader.SetFileAddress("D:\\卫星导航算法与程序设计\\short-baseline\\oem719-202203170900-1.bin");/*\\Zero-baseline\\oem719-202203031500-1.bin*/ /*\\short-baseline\\oem719-202203170900-1.bin*/
    BaseDataLoader.SetSocketAddress("47.114.134.129", 7190);

    fstream fout;
    //fout.open("D:\\卫星导航算法与程序设计\\RTK20230422.txt", ios::out);


    //打开文件或网络
    if (!RoverDataLoader.open())
    {
        cout << "RunMode=" << RunMode << "Rover open fail\n";
        return -RunMode;
    }

    if (!BaseDataLoader.open())
    {
        cout << "RunMode=" << RunMode << "Base open fail\n";
        return -RunMode;
    }

    //获取流动站buff数据，文件结束或网络断网返回false
    while (RoverDataLoader.ReadBuff(Roverbuff, RoverRemLen) == true)
    {

        unsigned short Rover_MesID = NonAteloem7DataDecode(Roverbuff, &RoverRemLen, &Rover_ObsData, &GPSEphem, &BDSEphem, &Rover_Pos);
        
        //流动站单点定位
        if (Rover_MesID == 43)
        {
            //粗差探测
            DetectOutlier(&Rover_ObsData);
            //计算信号发射时刻卫星位置
            SignalSendTimeSatPos(&Rover_ObsData, &GPSEphem, &BDSEphem, &Rover_SatPosRes);
            //SPP&SPV
            if (StandardPointPositioning(&Rover_ObsData, &Rover_SatPosRes, &Rover_StaPosRes))
                StandardPointVelocity(&Rover_ObsData, &Rover_SatPosRes, &Rover_StaPosRes);

            //输出
            //cout << "Rover  ";
            //StaPosRes_Show(&Rover_ObsData.GpsTime, &Rover_StaPosRes, Rover_SatPosRes.size());

            //基站单点定位（文件结束或断网则跳过，直接使用流动站结果）
            while (BaseDataLoader.ReadBuff(Basebuff, BaseRemLen) == true)
            {
                unsigned short Base_MesID = NonAteloem7DataDecode(Basebuff, &BaseRemLen, &Base_ObsData, &Base_Pos);

                //基站数据的时间在后，则直接使用流动站单点定位
                if (Base_ObsData.GpsTime - Rover_ObsData.GpsTime > 0.1)
                    break;
                //流动站数据的时间在后或者，两站时间相同但基站没解到观测值，则继续解码
                if (Rover_ObsData.GpsTime - Base_ObsData.GpsTime > 0.1 || Base_MesID != 43)
                    continue;

                //粗差探测
                DetectOutlier(&Base_ObsData);
                //计算信号发射时刻卫星位置
                SignalSendTimeSatPos(&Base_ObsData, &GPSEphem, &BDSEphem, &Base_SatPosRes);
                //SPP&SPV
                if (StandardPointPositioning(&Base_ObsData, &Base_SatPosRes, &Base_StaPosRes))
                StandardPointVelocity(&Base_ObsData, &Base_SatPosRes, &Base_StaPosRes);

                //单差计算
                SingleDifference(&Rover_ObsData, &Base_ObsData, &Sd_ObsData);

                //单差粗差探测
                SDDetectOutlier(&Sd_ObsData);

                //选取参考星
                SelectReferenceSatellite(&Sd_ObsData, &Dd_Data);

                //获取基站精确坐标
                //BLH_to_XYZ(&Base_Pos.Blh, &Base_StaPosRes.StationPos, WGS84_a, WGS84_e2);
                Base_StaPosRes.StationPos.Xyz.X = -2267804.5263;
                Base_StaPosRes.StationPos.Xyz.Y = 5009342.3723;
                Base_StaPosRes.StationPos.Xyz.Z = 3220991.8632;
                //计算浮点解
                if (RTKFloat(&Sd_ObsData, Rover_SatPosRes, Base_SatPosRes, &Rover_StaPosRes, &Base_StaPosRes, &Dd_Data))
                {
                    //lambda固定模糊度
                    lambda(Dd_Data.n_of_FloatAmbiguity, 2, Dd_Data.FloatAmbiguity, Dd_Data.Q_of_FloatAmbiguity, Dd_Data.FixedAmbiguity, Dd_Data.ResidualAmbiguity);
                    Dd_Data.Ratio = Dd_Data.ResidualAmbiguity[1] / Dd_Data.ResidualAmbiguity[0];

                    //计算固定解
                    if (Dd_Data.Ratio > 3.0)
                    {
                        Dd_Data.isFixed = true;
                        RTKFixed(&Sd_ObsData, Rover_SatPosRes, Base_SatPosRes, &Rover_StaPosRes, &Base_StaPosRes, &Dd_Data);
                    }

                    delete[] Dd_Data.FloatAmbiguity;
                    delete[] Dd_Data.Q_of_FloatAmbiguity;
                }

                //输出
                //cout << "Base  ";
                //StaPosRes_Show(&Base_ObsData.GpsTime, &Base_StaPosRes, Base_SatPosRes.size());
                //cout << "Rover  ";
                //StaPosRes_Show(&Rover_ObsData.GpsTime, &Rover_StaPosRes, Rover_SatPosRes.size());

                RTK_Show(&Sd_ObsData.GpsTime, &Rover_StaPosRes, &Dd_Data, Sd_ObsData.Sat_Number);
                //RTK_OutputToFile(fout, &Sd_ObsData.GpsTime, &Rover_StaPosRes, &Dd_Data, Sd_ObsData.Sat_Number);

                //数据清理
                Base_SatPosRes.clear();
                Base_ObsData.SatObs.clear();
                Sd_ObsData.SdSatObs.clear();
                Base_ObsData.Sat_Number = 0;
                Sd_ObsData.Sat_Number = 0;
                Dd_Data.isFixed = false;


                break;

            }

            //数据清理
            Rover_SatPosRes.clear();
            Rover_ObsData.SatObs.clear();
            Rover_ObsData.Sat_Number = 0;

        }

     }

    BaseDataLoader.close();
    RoverDataLoader.close();
    return RunMode;
}

