#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <math.h>
#include <pcl/common/centroid.h>


int
main (int argc, char** argv)
{

    //set the centroid and Bounding Box

    float cloudCentroid[3]={1,0,0};
    float BBsize[3]={0.4,0.4,0.4};
    octomap::point3d sensorOrigin(0,0,0);

    /*  READ bunny.plc  */

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("bunny.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file bunny.pcd \n");
    return (-1);
  }

    /*  Create OcTree  */
    
  octomap::Pointcloud x;
  octomap::OcTree tree (0.01),cloudAndUnknown (0.01);            //  Two octrees are needed, "tree" will have all the information
                                                                //  needed for processing, "cloudAndUnknown" is a simplified OcTree
                                                                //  containing just the known occupied voxels and unknown voxels.

  for (size_t i = 0; i < cloud->points.size (); ++i){
//      std::cout << "    " << cloud->points[i].x
//                << " "    << cloud->points[i].y
//                << " "    << cloud->points[i].z << std::endl;                     //  Print cloud points

      x.push_back(cloud->points[i].x+cloudCentroid[0]
                  ,cloud->points[i].y+cloudCentroid[1]
                  ,cloud->points[i].z+cloudCentroid[2]);
      tree.insertPointCloud(x,sensorOrigin);

      octomap::OcTreeNode * cloudNode=cloudAndUnknown.updateNode(cloud->points[i].x+cloudCentroid[0]
              ,cloud->points[i].y+cloudCentroid[1]
              ,cloud->points[i].z+cloudCentroid[2],true);
      cloudNode->setValue(13);          //  the number 13(random) is saved on each cloud voxel, so that when
                                        //  rayCast finds the node I can check if ==13 I found an object voxel
  }

    std::cout<< cloudCentroid[0]<< " < X "<<cloudCentroid[1]<< " < Y "<<cloudCentroid[2]<< " < Z ";


    //      Create point wall assuming camera center is in (0,0,0) and the camera direction is towards (1,0,0)
    //      kinect has a Field of View of (58.5 x 46.6 degrees) and a Resolution of (320 x 240 px).
    //      Offline computation results in a pointwall from (1,-0.56,-0.431) to (1,0.56,0.431)
    //      with 320 points along the Y axes (0.003489 distance between points)
    //      with 240 points along the Z axes (0.003574 distance between points)


    octomap::Pointcloud Pointwall,PW,cPW,vPW;       // Pointwall will represent the sensor FoV in origin coordinates
                                                    // PW will be translated to be in front of the sensor for visualization
                                                    // cPW (constant PW) will be a generic PointWall representing the FoV of a kinect
                                                    // vPW (variable PW) will be cPW rolled so the Next Best Views candidates can
                                                    //      raytrace towards the centroid of the point cloud.


    octomap::point3d Point3dwall(1,0,0);

    for(int ii=1;ii<321;ii++){
        for(int iii=0;iii<241;iii++){
            Point3dwall.y()= (-0.560027)+(ii*0.003489);
            Point3dwall.z()= (-0.430668)+(iii*0.003574);
            Pointwall.push_back(Point3dwall);

        }

    }
    cPW=vPW=Pointwall;


    //      Rotate the pointwall to the camera origin and octogonal to the vector pointing from
    //      the camera origin to the centroid, translation is not needed since the function castRay
    //      works with origin and direction(not end).


    octomath::Vector3 Translation(0,0,0),T2(sensorOrigin);       //translation
    float roll=atan2(-sensorOrigin.y()+cloudCentroid[1],-sensorOrigin.x()+cloudCentroid[0]);
    //      std::cout <<std::endl<<" roll= "<<roll;

    octomath::Quaternion Rotation(0,0,roll),R2(0,0,0);       //(Yaw,Pitch,Roll)
    octomap::pose6d RotandTrans(Translation,Rotation),RT2(T2,R2);


    Pointwall.transform(RotandTrans);
        PW=Pointwall;
    PW.transform(RT2);

    octomap::point3d iterator;      //  Helper needed for castRay function

    //      Raytracing
    int unknownHits=0;
    int knownHits=0;
    float alpha;
    float beta;
    float xp, yp, zp, xpp, distancep, distance=3;

//      A background wall is built leaving empty the shadow of the object,
//      this is necesary so that the octree can recognize what area is empty known and unknown,
//      otherwise it will assume all surroundings of the cloud as unknown.

    for(int i=0;i<Pointwall.size();i++){

        if(!tree.castRay(sensorOrigin,Pointwall.getPoint(i),iterator)){
//            std::cout << "unknown!" << std::endl;
            unknownHits++;
            xp=-sensorOrigin.x()+PW.getPoint(i).x();
            yp=-sensorOrigin.y()+PW.getPoint(i).y();
            zp=-sensorOrigin.z()+PW.getPoint(i).z();
            alpha=atan2(yp,xp);
            xpp=sqrt((xp*xp)+(yp*yp));
            beta=atan2(zp,xpp);
            iterator.z()=sensorOrigin.z()+distance*sin(beta);
            distancep=sqrt((distance*distance)-(zp*zp));
            iterator.y()=sensorOrigin.y()+distancep*sin(alpha);
            iterator.x()=sensorOrigin.x()+distancep*cos(alpha);
            x.push_back(iterator);

//            std::cout <<iterator.x()<<" <x "<<iterator.y()<<" <y "<<iterator.z()<<" <z "<<std::endl<<"unknows > "<<unknownHits<<std::endl;
        }
    }


    tree.insertPointCloud(x,sensorOrigin);

    tree.insertPointCloud(PW,sensorOrigin);             //  PW inserted for visualizing the FoV from the sensorOrigin

    tree.writeBinary("check.bt");


/*  Search for unknown voxels  */

octomap::point3d unknown;

float resolution = 0.005;

for (float ix = (cloudCentroid[0]-(BBsize[0]/2)); ix < (cloudCentroid[0]+(BBsize[0]/2)); ix += resolution){
   for (float iy = (cloudCentroid[1]-(BBsize[1]/2)); iy < (cloudCentroid[1]+(BBsize[1]/2)); iy += resolution){
         for (float iz = (cloudCentroid[2]-(BBsize[2]/2)); iz < (cloudCentroid[2]+(BBsize[2]/2)); iz += resolution)
         {
              if (tree.search(ix,iy,iz)==NULL)
              {

                  unknown.x()=ix;
                  unknown.y()=iy;
                  unknown.z()=iz;
                  tree.updateNode(unknown,true);
                  octomap::OcTreeNode * unknownCloud=cloudAndUnknown.updateNode(unknown,false);  //  Compose tree needed for NBV computation
                  unknownCloud->setValue(24);               //  the number 24(random) is saved on each cloud voxel, so that when
                                                            //  rayCast finds the node I can check if ==24 I found an unkown voxel
              }
         }
   }
}




/*  Compute Next Best View candidates       */

//  The candidates will be computed at a constant distance from the object, all the views will be pointing
//      towards the centroid of the cloud. The candidates are computed in Z=0 by circular tessellation.
//      futher updates will include sphere tessellation for 3D NBV.

int NBVcandidateNumber=16;                          //number of candidates
float AngleBetweenCandidate=6.2832/NBVcandidateNumber;
float Candidates [NBVcandidateNumber][5];           //x,y,z,roll,Occlussion Aware VI
float P_Occupation=0.5;                             //Probability of a random voxel being occupied
int iNBV=0;
octomap::point3d NBVcandidate;
octomap::KeyRay rayBeam;

for(int i=0;i<NBVcandidateNumber;i++){

NBVcandidate.x()=Candidates[i][0]=cloudCentroid[0]+(0.8*sin(AngleBetweenCandidate*i));       // [x]
NBVcandidate.y()=Candidates[i][1]=cloudCentroid[1]+(0.8*cos(AngleBetweenCandidate*i));       // [y]
NBVcandidate.z()=Candidates[i][2]=cloudCentroid[2];                                          // [z]
Candidates[i][3]=atan2(-Candidates[i][1]+cloudCentroid[1],-Candidates[i][0]+cloudCentroid[0]);      //  [roll]
//std::cout<<Candidates[i][3];
Candidates[i][4]=0;
octomath::Vector3 Translation2(NBVcandidate.x(),NBVcandidate.y(),NBVcandidate.z());
octomath::Quaternion Rotation2(0,0,Candidates[i][3]);
octomath::Pose6D RotandTrans2(Translation2,Rotation2);
vPW=cPW;
vPW.transform(RotandTrans2);                                    // The Pointcloud representing the FoV of the kinect
                                                                //  is rotated and translated so that computeRayKeys can raytrace
                                                                //  from the NBVcandidate position until each of this Pointcloud points.

//cloudAndUnknown.insertPointCloud(vPW,NBVcandidate);                                       //Visualize Candidate FoV
//cloudAndUnknown.updateNode(Candidates[i][0],Candidates[i][1],Candidates[i][2],true);     //Visualize Candidates

//Occlussion Aware VI is used as method to compute "how good" is a candidate

int unknownVoxelsInRay=0;
Candidates[i][4]=0;
for(int ii=0;ii<vPW.size();ii++){
    bool Continue=true;
    cloudAndUnknown.computeRayKeys(NBVcandidate,vPW.getPoint(ii),rayBeam);
    for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
        octomap::OcTreeNode * node=cloudAndUnknown.search(*it);
        if(node!=NULL){
            if (node->getValue()==13){
                Continue=false;
            }else if(node->getValue()==24){
                unknownVoxelsInRay++;
                Candidates[i][4]+=pow(0.5,unknownVoxelsInRay);
            }

        }
    }

    unknownVoxelsInRay=0;

}
std::cout<< std::endl<<"from (" << Candidates[i][0] << ") ("<< Candidates[i][1] << ") ("<< Candidates[i][2]<<") VI: "<<Candidates[i][4];
if(Candidates[i][4]>Candidates[iNBV][4]){
        iNBV=i;
}


}
std::cout<< std::endl<<"NEXT BEST VIEW IS (" << Candidates[iNBV][0] << ") ("<< Candidates[iNBV][1] << ") ("<< Candidates[iNBV][2]<<")";

  return (0);
}
