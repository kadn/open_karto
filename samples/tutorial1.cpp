/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <open_karto/Mapper.h>

//this data is used to assign distance that precalculated by wph. 这是第二帧与第一帧重合的地方在第二帧的scan位置看到的距离。总共有90度范围，另外数据是逆时针的，这与数学相同。
/*
you will see the predefined map by wph.
%MATLAB code.  This code show how I got const double data[91], actually data[0] is added by hand, not code.
% used for calculate the distance between frame 2 and frame 1, used in
% open_karto turtorial1 for my example. because I think the origin code
% builds a confusing map. So I correct it.

x1=[1,0];
t=(90:-0.1:-90)/180*pi;
x = 3*cos(t)+1;
y = 3*sin(t);

point = [x',y'];

distance = [];
length = 705;

pp = point(length:-1:1,:);

for i=1:length   %
    p = [pp(i,1)-1,pp(i,2)-1];
    theta = atan(p(2)/p(1))/pi*180;
    if(theta-floor(theta)<0.2) && floor(theta)>0
        distance(floor(theta))=((p(1))^2+(p(2))^2)^0.5;
    end
end


% uncomment following code to make the data has comma
% fid = fopen('data.txt','wt')
% fprintf(fid,'%f,',distance);
% fclose(fid)
*/
const double data[91]={2.8279,2.808727,2.791282,2.773847,2.756430,2.740774,2.723405,2.706070,2.690502,2.673248,2.657763,2.642325,2.625231,2.609906,2.594642,2.579444,2.564316,2.549265,2.534295,2.519411,2.504618,2.489923,2.476945,2.462447,2.448061,2.435373,2.421215,2.408738,2.396367,2.382581,2.370449,2.358435,2.346543,2.334779,2.321702,2.310222,2.298882,2.289078,2.278014,2.267102,2.256347,2.245754,2.236621,2.226342,2.216238,2.207543,2.197777,2.189385,2.179973,2.171897,2.162853,2.155107,2.147518,2.139043,2.131803,2.124730,2.117826,2.110148,2.103617,2.097264,2.091092,2.085103,2.079300,2.073685,2.068260,2.063028,2.057991,2.053151,2.048510,2.044693,2.040428,2.036368,2.032514,2.028870,2.025435,2.022660,2.019620,2.016795,2.014185,2.012122,2.009916,2.007930,2.006403,2.004826,2.003471,2.002486,2.001544,2.000825,2.000386,2.000082,2.000000};
/**
 * Sample code to demonstrate karto map creation
 * Create a laser range finder device and three "dummy" range scans. 
 * Add the device and range scans to a karto Mapper.
 */
karto::Dataset* CreateMap(karto::Mapper* pMapper)
{
  karto::Dataset* pDataset = new karto::Dataset();

  /////////////////////////////////////
  // Create a laser range finder device - use SmartPointer to let karto subsystem manage memory.
  karto::Name name("laser0");
  //LaserRangeFinder 之中包含了 对于不同型号的laser的参数的填写，还可以自定义laser，这里面有默认的关于laser的配置数据
  karto::LaserRangeFinder* pLaserRangeFinder = karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name);
  pLaserRangeFinder->SetOffsetPose(karto::Pose2(1.0, 0.0, karto::math::DegreesToRadians(0.0)));   //这里说了scan laser0在车坐标系中的位置。
  pLaserRangeFinder->SetAngularResolution(karto::math::DegreesToRadians(1));  //180度的laser，分辨率是0.5，就会有361个光束
  pLaserRangeFinder->SetRangeThreshold(12.0);  //这个是说 12.0以内的数据有效，大于12.0可以认为不准确

  //将 pLaserRangeFinder加入管理，其中用到了 singleton的概念来创建管理器，并将这个pLaserRangeFinder管理起来
  pDataset->Add(pLaserRangeFinder);

  /////////////////////////////////////
  // Create three localized range scans, all using the same range readings, but with different poses. 
  karto::LocalizedRangeScan* pLocalizedRangeScan = NULL;

  // 1. localized range scan

  // Create a vector of range readings. Simple example where all the measurements are the same value.
  std::vector<kt_double> readings;
  for (int i=0; i<181; i++)
  {
    readings.push_back(3.0);
  }
  
  // create localized range scan
  // LocalizedRangeScan 则存储了这一帧的数据，数据来源，以及相关的robot,sensor之间的相对关系，以及 他们的里程计信息
  pLocalizedRangeScan = new karto::LocalizedRangeScan(name, readings);
  pLocalizedRangeScan->SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
  pLocalizedRangeScan->SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));

  // Add the localized range scan to the mapper
  pMapper->Process(pLocalizedRangeScan);
  std::cout << "Pose: " << pLocalizedRangeScan->GetOdometricPose() << " Corrected Pose: " << pLocalizedRangeScan->GetCorrectedPose() << std::endl;

  // Add the localized range scan to the dataset
  pDataset->Add(pLocalizedRangeScan);

  // 2. localized range scan


  // create new range
  readings.clear();
  for (int i=0; i<91; i++)
  {
    readings.push_back(data[i]);
  }
  for (int i=0;i<90; i++)
  {
    readings.push_back(3.0);
  }
  // create localized range scan
  pLocalizedRangeScan = new karto::LocalizedRangeScan(name, readings);
  pLocalizedRangeScan->SetOdometricPose(karto::Pose2(1.0, 0.0, 1.57));
  pLocalizedRangeScan->SetCorrectedPose(karto::Pose2(1.0, 0.0, 1.57));

  // Add the localized range scan to the mapper
  pMapper->Process(pLocalizedRangeScan);
  std::cout << "Pose: " << pLocalizedRangeScan->GetOdometricPose() << " Corrected Pose: " << pLocalizedRangeScan->GetCorrectedPose() << std::endl;

  // Add the localized range scan to the dataset
  pDataset->Add(pLocalizedRangeScan);

  // 3. localized range scan

  // create localized range scan
  // pLocalizedRangeScan = new karto::LocalizedRangeScan(name, readings);
  // pLocalizedRangeScan->SetOdometricPose(karto::Pose2(1.0, -1.0, 2.35619449));
  // pLocalizedRangeScan->SetCorrectedPose(karto::Pose2(1.0, -1.0, 2.35619449));

  // // Add the localized range scan to the mapper
  // pMapper->Process(pLocalizedRangeScan);
  // std::cout << "Pose: " << pLocalizedRangeScan->GetOdometricPose() << " Corrected Pose: " << pLocalizedRangeScan->GetCorrectedPose() << std::endl;

  // // Add the localized range scan to the dataset
  // pDataset->Add(pLocalizedRangeScan);

  return pDataset;
}

/**
 * Sample code to demonstrate basic occupancy grid creation and print occupancy grid.
 */
karto::OccupancyGrid* CreateOccupancyGrid(karto::Mapper* pMapper, kt_double resolution)
{
  std::cout << "Generating map..." << std::endl;

  // Create a map (occupancy grid) - time it
  karto::OccupancyGrid* pOccupancyGrid = karto::OccupancyGrid::CreateFromScans(pMapper->GetAllProcessedScans(), resolution);

  return pOccupancyGrid;
}

/**
 * Sample code to print a basic occupancy grid
 */
void PrintOccupancyGrid(karto::OccupancyGrid* pOccupancyGrid)
{
  if (pOccupancyGrid != NULL)
  {
    // Output ASCII representation of map
    kt_int32s width = pOccupancyGrid->GetWidth();
    kt_int32s height = pOccupancyGrid->GetHeight();
    karto::Vector2<kt_double> offset = pOccupancyGrid->GetCoordinateConverter()->GetOffset();

    std::cout << "width = " << width << ", height = " << height << ", scale = " << pOccupancyGrid->GetCoordinateConverter()->GetScale() << ", offset: " << offset.GetX() << ", " << offset.GetY() << std::endl;
    for (kt_int32s y=height-1; y>=0; y--)
    {
      for (kt_int32s x=0; x<width; x++) 
      {
        // Getting the value at position x,y
        kt_int8u value = pOccupancyGrid->GetValue(karto::Vector2<kt_int32s>(x, y));

        switch (value)
        {
        case karto::GridStates_Unknown:
          std::cout << "*";
          break;
        case karto::GridStates_Occupied:
          std::cout << "X";
          break;
        case karto::GridStates_Free:
          std::cout << " ";
          break;
        default:
          std::cout << "?";
        }
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}

int main(int /*argc*/, char /***argv*/)
{
  // use try/catch to catch karto exceptions that might be thrown by the karto subsystem. 
  /////////////////////////////////////
  // Get karto default mapper
  karto::Mapper* pMapper = new karto::Mapper();
  if (pMapper != NULL)
  {
    karto::OccupancyGrid* pOccupancyGrid = NULL;

    /////////////////////////////////////
    // sample code that creates a map from sample device and sample localized range scans

    std::cout << "Tutorial 1 ----------------" << std::endl << std::endl;

    // clear mapper
    pMapper->Reset();

    // create map from created dataset
    karto::Dataset* pDataset = CreateMap(pMapper);

    // create occupancy grid at 0.1 resolution and print grid
    pOccupancyGrid = CreateOccupancyGrid(pMapper, 0.1);
    PrintOccupancyGrid(pOccupancyGrid);
    delete pOccupancyGrid;

    // delete mapper
    delete pMapper;

    delete pDataset;
  }

  return 0;
}
