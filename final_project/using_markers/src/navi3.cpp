// ****************************************Yansong Wu*************************************************
//
// This cpp receives NAO position, target position and obstacles position. Based on the information,
// it generates a navigation trajectory to move to the target without collision. When there is no more
// obstales between NAO and target, that means NAO can walks directly towards the taget. Then the nav-
// gation part is finished. It can call the service in the detection.cpp. 
//
// ****************************************Yansong Wu*************************************************

#include <using_markers/navi.h>


// distance between point and segment
double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1)
{
    double A=pX-x0;
    double B=pY-y0;
    double C=x1-x0;
    double D=y1-y0;

    double dot=A*C+B*D;
    double len_sq=C*C+D*D;
    double param=dot/len_sq;
    double xx,yy;

    if(param<0)
    {
        xx=x0;
        yy=y0;
    }
    else if(param>1)
    {
        xx=x1;
        yy=y1;
    }
    else
    {
        xx=x0+param*C;
        yy=y0+param*D;
    }
    double distance; 
    distance=pow((pX-xx),2)+pow((pY-yy),2);
    distance=sqrt(distance);
    return distance;
}

// detect if there is obstacle on the way to target
// This function is used to detect, if NAO can run directly to the target
// If the distances fronm all the obstacle to the segment(from NAO to target) is bigger than the radial 
// That means NAO can walk directly towards the obstacle

bool ifobstcale(double px[3],double py[3],double destx,double desty,
        double localx,double localy,double localtheta)
{
    bool flag[3]={1,1,1};
    double r[3]={r0,r1,r2};
    for (int i=0;i<3;i++)
    {
        if(flag[i]==1)
        {
            double Ldist;
            Ldist=distanceToLine(px[i],py[i],localx,localy,destx,desty); //wrt world
            if(r[i]<Ldist)
            {
                flag[i]=0;
            }
        }
        
    }

    if(flag[0]+flag[1]+flag[2])
    {
        return 0;   // there's no obstacle on the way to destination
    }
    else
    {
        return 1;   // there's obstacle on the way
    }
}


// trajectory planning
// this function aims at mergeing all the obstacles together and generate the trajectory
// the trajectory is returned as waypoints
double *navi(double px[3],double py[3],double destx,double desty,
        double localx,double localy,double localtheta)
{
    static double waypoint[4];
    
    double xmin,xmax,ymin,ymax,rmax;
    double r[3]={r0,r1,r2};

    xmin=px[0]-r[0];
    xmax=px[0]+r[0];
    ymin=py[0]-r[0];
    ymax=py[0]+r[0];
    
    for(int i=1;i<3;i++)
    {
        if (px[i]-r[i]<xmin)
        {
            xmin=px[i]-r[i];
        }
        if (px[i]+r[i]>xmax)
        {
            xmax=px[i]+r[i];
        }
        if (py[i]-r[i]<ymin)
        {
            ymin=py[i]-r[i];
        }
        if (py[i]+r[i]>ymax)
        {
            ymax=py[i]+r[i];
        }
    }
    xmin=xmin;
    ymin=ymin-0.05;
    xmax=xmax;
    ymax=ymax+0.05;

    ROS_INFO_STREAM("x-range"<<xmin<<"  "<<xmax);
    ROS_INFO_STREAM("y-range"<<ymin<<"  "<<ymax);
    
    double d1,d2;
    d1=(ymin-localy)*(ymin-localy);
    d2=(ymax-localy)*(ymax-localy);

    if (d1<d2)
    {
        waypoint[0]=xmax;
        waypoint[1]=ymax;
        waypoint[2]=xmin;
        waypoint[3]=ymax;
    }
    else
    {
        waypoint[0]=xmax;
        waypoint[1]=ymin;
        waypoint[2]=xmin;
        waypoint[3]=ymin;
    }

    return waypoint;  

}

// this service is used to receive the position of obstacles and the NAO's position wrt world

bool my_serviceServer1(using_markers::obstacleDetect::Request &req,using_markers::obstacleDetect::Response &res)
{    
   
    ROS_INFO_STREAM(req);
    // obstacle position
    ox[0]=req.obx1;
    ox[1]=req.obx2;
    ox[2]=req.obx3;
    oy[0]=req.oby1;
    oy[1]=req.oby2;
    oy[2]=req.oby3;
    // nao pose
    naox=req.naox;
    naoy=req.naoy;
    naotheta=req.naotheta;
    res.ret=1;
    launchflag=1;
    return 1;
}

// this service is used to receive the position of target wrt NAO

bool my_serviceServer2(using_markers::destination::Request &req,using_markers::destination::Response &res)
{
    dist_dest=req.dist;
    theta_dest=req.theta;
    res.ret=1;
    ROS_INFO_STREAM("DESTINATION DISTANCE:"<<dist_dest<<"   THETA:"<<theta_dest);
    return 1;
}


int main( int argc, char** argv )
{
    
    ros::init(argc, argv, "navi");
    ros::NodeHandle n;
    
    ros::ServiceServer my_service1 = n.advertiseService("naviObstacle",my_serviceServer1);
    ros::ServiceServer my_service2 = n.advertiseService("naviDestination",my_serviceServer2);
        
    
    ros::ServiceClient client1 = n.serviceClient<speech::walk_polar>("WalkPolar"); //walk client
    ros::ServiceClient client2 = n.serviceClient<detection::mode1>("Mode_change"); //change detection mode for grasp part
    ros::ServiceClient client3 = n.serviceClient<using_markers::localization>("localization"); //get NAO's current positions
    ros::ServiceClient client4 = n.serviceClient<using_markers::dest>("drawDestt"); //draw destination and planned trajectory


    ros::Publisher marker_pub2 = n.advertise<visualization_msgs::Marker>("visualization_marker9", 10);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker8", 10);
    

    bool result; //if there is no more obstacle in front of destination, result=0
    
    launchflag=0;
    x_dest=NOTEXIST;
    dist_dest=NOTEXIST;
    bool flag_firsttime=1;
    double *p;
    int navi_state; // 1 -- on the way to 1st waypoint  2 -- on the way to the 2ndwaypoint   3 -- can go direktly to destination
  
    
    
    while(ros::ok())
    {
        
        if(dist_dest!=NOTEXIST)
        {
            if(launchflag)
            {
                
                        

                if(flag_firsttime==1)
                {
                    ROS_INFO_STREAM("NAVIGATION BEGIN...");

                    x_dest=naox+cos(theta_dest+naotheta)*dist_dest;
                    y_dest=naoy+sin(theta_dest+naotheta)*dist_dest;

                                       
                    // call

                    ROS_INFO_STREAM("destination"<<x_dest<<"   "<<y_dest);
                    
                    p=navi(ox,oy,x_dest,y_dest,naox,naoy,naotheta);
                    ROS_INFO_STREAM("WAYPOINTS GENREATING ...");
                    ROS_INFO_STREAM("1st: "<<*p<<"  "<<*(p+1));
                    ROS_INFO_STREAM("2nd: "<<*(p+2)<<"  "<<*(p+3));

                    // call
                    using_markers::dest srvd;
                    srvd.request.dx=x_dest;
                    srvd.request.dy=y_dest;
                    srvd.request.x1=double(*p);
                    srvd.request.y1=double(*(p+1));
                    srvd.request.x2=double(*(p+2));
                    srvd.request.y2=double(*(p+3));
                    client4.call(srvd);
                    
                    flag_firsttime=0;
                    navi_state=1;
                    ROS_INFO_STREAM("**************************");
                }
                else
                {
                    bool endflag=ifobstcale(ox,oy,x_dest,y_dest,naox,naoy,naotheta);
                    if(endflag==0) // NAO can walk directly to the target
                    {
                        navi_state=3;
                    }
                    

                    using_markers::localization srv3;
                    srv3.request.localrequest=0;
                    ROS_INFO_STREAM("**************************");
                    if(client3.call(srv3))
                    {
                        ROS_INFO_STREAM("%%%%%%%% Localization result %%%%%%%%");
                        ROS_INFO_STREAM(srv3.response);
                        naox=srv3.response.x;
                        naoy=srv3.response.y;
                        naotheta=srv3.response.theta;

                        // MOVE 
                        double move_distance, move_theta, waypoint_distance;
                        speech::walk_polar srv1;


                        if(navi_state==3) // NAO has finished navigation or NAO can walk directly to the obstacle without the effect of obstacles
                        {
                            
                            move_theta=atan2(y_dest-naoy,x_dest-naox)-naotheta;
                            if(move_theta>PI)
                            {
                                move_theta-=2*PI; 
                            }

                            if(move_theta<-PI)
                            {
                                move_theta+=2*PI; 
                            }

                            move_theta=move_theta;
                            move_distance=0;
                            srv1.request.dist=move_distance;
                            srv1.request.theta=move_theta;


                            ROS_INFO_STREAM("move_distance"<<move_distance<<"   move_theta"<<move_theta);

                            
                            client1.call(srv1);  

                            launchflag=0;
                            ROS_INFO_STREAM("NAVIGATION END");
                            ROS_INFO_STREAM("There is no more obstacle on the way to destination.");

                            // call the sevice to trigger grasp part
                            detection::mode1 tmp_grasp;
                            tmp_grasp.request.req = 3;
                            client2.call(tmp_grasp);
                            
                            

                        }

                        // on the way to 2nd waypoint
                        if(navi_state==2)
                        {
                            waypoint_distance = sqrt((naox-*(p+2))*(naox-*(p+2))+(naoy-*(p+3))*(naoy-*(p+3))); 
                            if(waypoint_distance>0.05)
                            {
                                if(waypoint_distance>=0.15)
                                {
                                    move_distance=0.15;
                                }
                                else
                                {
                                    move_distance=waypoint_distance;
                                }
                                move_theta=atan2(*(p+3)-naoy,*(p+2)-naox)-naotheta;

                                if(move_theta>PI)
                                {
                                   move_theta-=2*PI; 
                                }

                                if(move_theta<-PI)
                                {
                                   move_theta+=2*PI; 
                                }


                                srv1.request.dist=move_distance;
                                srv1.request.theta=move_theta;
                                ROS_INFO_STREAM("move_distance"<<move_distance<<"   move_theta"<<move_theta);
                                
                                client1.call(srv1);
                                srv1.request.dist=0;
                                if(move_theta>0)
                                {
                                    srv1.request.theta=-1*move_theta/1.03;
                                }
                                else
                                {
                                    srv1.request.theta=-1.03*move_theta;
                                }
                                
                                
                                client1.call(srv1);
                            } 
                            else
                            {
                                ROS_INFO_STREAM("reach 2nd waypoint");
                                navi_state=3;
                            }
                            
                        }

                        // on the way to 1st waypoint
                        if(navi_state==1)
                        {
                            waypoint_distance = sqrt((naox-*p)*(naox-*p)+(naoy-*(p+1))*(naoy-*(p+1))); 
                            if(waypoint_distance>0.10)
                            {
                                if(waypoint_distance>=0.05)
                                {
                                    move_distance=0.15;
                                }
                                else
                                {
                                    move_distance=waypoint_distance;
                                }
                                move_theta=atan2(*(p+1)-naoy,*p-naox)-naotheta;
                                if(move_theta>PI)
                                {
                                   move_theta-=2*PI; 
                                }

                                if(move_theta<-PI)
                                {
                                   move_theta+=2*PI; 
                                }

                                srv1.request.dist=move_distance;
                                srv1.request.theta=move_theta;
                                ROS_INFO_STREAM("move_distance"<<move_distance<<"   move_theta"<<move_theta);
                                
                                client1.call(srv1);
                                srv1.request.dist=0;
                                if(move_theta>0)
                                {
                                    srv1.request.theta=-1*move_theta/1.03;
                                }
                                else
                                {
                                    srv1.request.theta=-1.03*move_theta;
                                }
                                
                                client1.call(srv1);
                            } 
                            else 
                            {
                                ROS_INFO_STREAM("reach 1st waypoint");
                                navi_state=2;
                            }
                        }

                        
                        

                        

                    }
                }
                
                               
                

               
            }

        }
        
        
       
        ros::spinOnce();
    }

    
    ros::spin();
    return 0;
}
