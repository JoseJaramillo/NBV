/****************************************************
 *             Next Best View Computation           *
 *               Using Occlussion Aware             *
 *               Volumetric Information             *
 *                                                  *
 * The algorithm reads a PLC file and prints the    *
 * Next Best View based on Occlussion Aware method. *
 *                                                  *
 * Author: José Jaramillo                           *
 ****************************************************/

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <math.h>

int
main (int argc, char** argv){

    //  Set the centroid and Bounding Box
    float cloudCentroid[3]={1,0,0};
    float BBsize[3]={0.4,0.4,0.4};
    octomap::point3d sensorOrigin(0,0,0);

    //>>>>>>>>>> Read .pcd file <<<<<<<<<<

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("bunny.pcd", *cloud) == -1){       //  Load the file bunny.pcd

        PCL_ERROR ("Couldn't read file bunny.pcd \n");
        return (-1);

    }

    //>>>>>>>>>> Create octree <<<<<<<<<<

    /*  Two octrees are needed, "tree" will have all the information
        needed for processing, "cloudAndUnknown" is a simplified OcTree
        containing just the known occupied voxels and unknown voxels.   */

    octomap::Pointcloud scan;       //  Pointcloud will hold the .pcd information
    octomap::OcTree tree (0.01),cloudAndUnknown (0.01);

    for (size_t i = 0; i < cloud->points.size (); ++i){     //  Iterate over the loaded .pcd file

        //  Insert the .pcd cloud points into scan
        scan.push_back(cloud->points[i].x+cloudCentroid[0]
                ,cloud->points[i].y+cloudCentroid[1]
                ,cloud->points[i].z+cloudCentroid[2]);

        //  Insert the .pcd cloud points into the octree cloudAndUnknown
        octomap::OcTreeNode * cloudNode=cloudAndUnknown.updateNode(cloud->points[i].x+cloudCentroid[0]
                ,cloud->points[i].y+cloudCentroid[1]
                ,cloud->points[i].z+cloudCentroid[2],true);

        /*  The number 13(random) is saved on each cloud voxel, so that when rayCast
                finds the node I can check if ==13 I found an object voxel  */

        cloudNode->setValue(13);

    }

    /*  InsertPointCloud automatically sets as unknown voxels, the ones behind scan when seeing from sensorOrigin
            the algorithm will look for the unknown volxels which are necesary to compute the NBV. */

    tree.insertPointCloud(scan,sensorOrigin);

    //>>>>>>>>>> Create pointwall as Field of View of the camera <<<<<<<<<<

    /*  Create point wall representing the Field of View of the camera so that raytracing is made
            from the camera center towards each of its points.
            Assuming camera center is in (0,0,0) and the camera direction is towards (1,0,0).
            Kinect has a Field of View of (58.5 x 46.6 degrees) and a Resolution of (320 x 240 px).
            Offline computation results in a pointwall from (1,-0.56,-0.431) to (1,0.56,0.431),
            with 320 points along the Y axes (0.003489 distance between points),
            with 240 points along the Z axes (0.003574 distance between points).    */

    octomap::Pointcloud pointwall;      //  pointwall will represent the sensor FoV in origin coordinates.
    octomap::Pointcloud pointwallOrigin;        //  pointwallOrigin will represent the sensor FoV in origin coordinates,
                                                //      rolled to point towards the cloud centroid
    octomap::Pointcloud pointwallSensor;        //  pointwallSensor will be translated to be in front of the sensor for visualization (check.bt)
    octomap::point3d Point3dwall(1,0,0);    //  each point3d to be inserted into Pointwall

    //  Iterate on each pixel (320x240)
    for(int ii=1;ii<321;ii++){

        for(int iii=1;iii<241;iii++){

            Point3dwall.y()= (-0.560027)+(ii*0.003489);
            Point3dwall.z()= (-0.430668)+(iii*0.003574);
            pointwallOrigin.push_back(Point3dwall);

        }

    }

    pointwall=pointwallOrigin;

    /*  Rotate the pointwall to the camera origin and ortogonal to the vector pointing from
            the camera origin to the centroid, translation is not needed since the function
            castRay works with origin and direction(not end).   */

    octomath::Vector3 Translation(0,0,0),T2(sensorOrigin);
    float roll=atan2(-sensorOrigin.y()+cloudCentroid[1],-sensorOrigin.x()+cloudCentroid[0]);

    octomath::Quaternion Rotation(0,0,roll),R2(0,0,0);       // (Yaw,Pitch,Roll)
    octomap::pose6d RotandTrans(Translation,Rotation),RT2(T2,R2);

    pointwallOrigin.transform(RotandTrans);
    pointwallSensor=pointwallOrigin;
    pointwallSensor.transform(RT2);              //  For visualization PW is translated in front of the sensorOrigin
    octomap::point3d iterator;      //  Helper needed for castRay function

    //>>>>>>>>>> Create background wall to identify known empty volxels <<<<<<<<<<

    /*	A background wall is built leaving empty the shadow of the object, this is
            necesary so that the octree can recognize what area is empty known and
            unknown, otherwise it will assume all tree.writeBinary("check.bt");surroundings of the cloud as unknown.  */

    float alpha;	//	Angle in xy plane from sensorOrigin to each point in Pointwall
    float beta;		//	Elevation angle from sensorOrigin to each point in Pointwall
    float xp, yp, zp;		//	x,y,z coordinates of each point in Pointwall expressed in sensorOrigin coordinates
    float legAdjacentPointWall;		//	Leg adjacent length of a right triangle formed from sensorOrigin to each point in Pointwall
    float legAdjacentBackgroundPoint;		//	Leg adjacent length of a right triangle formed from sensorOrigin to the new background point
    float DISTANCE=3;		//  Distance from the sensorOrigin and the new background point
    octomap::Pointcloud backgroundWall;     //  Pointcloud holding the background wall

    for(int i=0;i<pointwallOrigin.size();i++){

        if(!tree.castRay(sensorOrigin,pointwallOrigin.getPoint(i),iterator)){

            //	Transform pointwall point to sensorOrigin coordinates subtracting sensorOrigin
            xp=-sensorOrigin.x()+pointwallSensor.getPoint(i).x();
            yp=-sensorOrigin.y()+pointwallSensor.getPoint(i).y();
            zp=-sensorOrigin.z()+pointwallSensor.getPoint(i).z();

            //	Get alpha and beta angles
            alpha=atan2(yp,xp);
            legAdjacentPointWall=sqrt((xp*xp)+(yp*yp));	//TODO check if this is correct (is minus)
            beta=atan2(zp,legAdjacentPointWall);

            //	Get the new background points and return to global coordinates by adding sensorOrigin
            iterator.z()=sensorOrigin.z()+DISTANCE*sin(beta);
            legAdjacentBackgroundPoint=sqrt((DISTANCE*DISTANCE)-(zp*zp));
            iterator.y()=sensorOrigin.y()+legAdjacentBackgroundPoint*sin(alpha);
            iterator.x()=sensorOrigin.x()+legAdjacentBackgroundPoint*cos(alpha);

            backgroundWall.push_back(iterator);		//	add points to point cloud

        }

    }

    tree.insertPointCloud(backgroundWall,sensorOrigin);     //  Check if i can use other than scan, since it contains the cloud
    //    tree.insertPointCloud(pointwallSensor,sensorOrigin);     //  PW inserted for visualizing the FoV from the sensorOrigin
    tree.writeBinary("check.bt");       //  Visualize "check.bt" with octovis

    //>>>>>>>>>> Search for unknown voxels <<<<<<<<<<

    float RESOLUTION = 0.005;		//	Search resolution

    //	Iterate over all the bounding box and search for unknown voxels
    for (float ix = (cloudCentroid[0]-(BBsize[0]/2)); ix < (cloudCentroid[0]+(BBsize[0]/2)); ix += RESOLUTION){

        for (float iy = (cloudCentroid[1]-(BBsize[1]/2)); iy < (cloudCentroid[1]+(BBsize[1]/2)); iy += RESOLUTION){

            for (float iz = (cloudCentroid[2]-(BBsize[2]/2)); iz < (cloudCentroid[2]+(BBsize[2]/2)); iz += RESOLUTION){

                if (tree.search(ix,iy,iz)==NULL){		//	If ==NULL it did not find any known (occupied or empty) voxel

                    //	Add a voxel in the empty position in the cloudAndUnknown OcTree
                    iterator.x()=ix;
                    iterator.y()=iy;
                    iterator.z()=iz;

                    octomap::OcTreeNode * unknownCloud=cloudAndUnknown.updateNode(iterator,false);  //  Compose tree needed for NBV computation

                    /*  The number 24(random) is saved on each cloud voxel, so that when rayCast
                            finds the node I can check if ==24 I found an unknown voxel */

                    unknownCloud->setValue(24);

                }

            }

        }

    }

    //>>>>>>>>>> Compute Next Best View candidates <<<<<<<<<<

    /*  The candidates will be computed at a constant distance from the object, all the views
            will be pointing towards the centroid of the cloud. The candidates are computed in Z=0
            by circular tessellation. Further updates will include sphere tessellation for 3D NBV.    */

    int NBV_CANDIDATENUMBER=16;		//  Number of candidates
    float AngleBetweenCandidate=6.2832/NBV_CANDIDATENUMBER;		//	AngleBetweenCandidate= 360 degrees (2*pi)/ Number of candidates
    float Candidates [NBV_CANDIDATENUMBER][5];		//	Array holding NBV candidates position [x,y,z],roll, and Occlussion Aware VI
    float P_OCCUPATION=0.5;		//	Probability of a random voxel being occupied
    int iNBV=0;     //  Index of the Next Best View in Candidates[i]
    float NBV_DISTANCE=0.8;      //  Distance from the cloud centroid and the NBV
    octomap::Pointcloud variablePointwall;      //  variablePointwall hold the FoV of the NBV candidates

    for(int i=0;i<NBV_CANDIDATENUMBER;i++){

        /* 	The Pointcloud representing the FoV of the kinect is rotated and translated
                so that computeRayKeys can raytrace from the NBVcandidate position until
                each of this Pointcloud points.    */

        iterator.x()=Candidates[i][0]=cloudCentroid[0]+(NBV_DISTANCE*sin(AngleBetweenCandidate*i));       // [x]
        iterator.y()=Candidates[i][1]=cloudCentroid[1]+(NBV_DISTANCE*cos(AngleBetweenCandidate*i));       // [y]
        iterator.z()=Candidates[i][2]=cloudCentroid[2];                                          // [z]
        Candidates[i][3]=atan2(-Candidates[i][1]+cloudCentroid[1],-Candidates[i][0]+cloudCentroid[0]);      //  [roll]
        Candidates[i][4]=0;		//	Set 0 the candidates Occlussion Aware VI
        octomath::Vector3 Translation2(iterator.x(),iterator.y(),iterator.z());		//	Vector with the NBVcandidate coordinates
        octomath::Quaternion Rotation2(0,0,Candidates[i][3]);		//	Quaternion containing the roll of NBVcandidate
        octomath::Pose6D RotandTrans2(Translation2,Rotation2);		//	Pose6D contains the pose (rotation and translation) of the NBVcandidate
        variablePointwall=pointwall;		//	Reset variablePointwall
        variablePointwall.transform(RotandTrans2);		//	Change the pose of variablePointwall

        //  Occlussion Aware VI is used as method to compute "how good" is a candidate
        octomap::KeyRay rayBeam;		//	Contains the position of each voxel in a ray
        int unknownVoxelsInRay=0;		//	Counter of unknown voxels found in a ray
        for(int ii=0;ii<variablePointwall.size();ii++){		//	iterate over all the pointwall
            bool Continue=true;		//	Boolean needed to stop searching when the ray hits a known occupied voxel

            //	Get the position of each voxel in a ray starting from NBVcandidate to each point in variablePointwall
            cloudAndUnknown.computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
            for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
                octomap::OcTreeNode * node=cloudAndUnknown.search(*it);		//	return the voxel found in such position
                if(node!=NULL){		//	when !=NULL, it found an unknown voxel or a known occupied voxel

                    //	When found a known occupied voxel stop
                    if (node->getValue()==13){

                        Continue=false;

                    }

                    /*	When found an unknown voxel it will add the probability of of being seen from the
                            NBVcandidate position which is given by the P_OCCUPATION to the number of
                            unknown voxels from NBVcandidate to that voxel. */

                    else if(node->getValue()==24){

                        unknownVoxelsInRay++;
                        Candidates[i][4]+=pow(P_OCCUPATION,unknownVoxelsInRay);

                    }

                }

            }

            unknownVoxelsInRay=0;		//	set to 0 after it stops raytracing

        }

        std::cout<< std::endl<<"from (" << Candidates[i][0] << ") ("<< Candidates[i][1] << ") ("<< Candidates[i][2]<<") VI: "<<Candidates[i][4];

        //	search the index of the biggest VI
        if(Candidates[i][4]>Candidates[iNBV][4]){

            iNBV=i;

        }

    }

    //	Print NBV
    std::cout<< std::endl<<"NEXT BEST VIEW IS (" << Candidates[iNBV][0] << ") ("<< Candidates[iNBV][1] << ") ("<< Candidates[iNBV][2]<<")";
    return (0);

}
