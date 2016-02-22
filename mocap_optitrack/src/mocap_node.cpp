/// \author <a href="mailto:graeve@ais.uni-bonn.de">Kathrin Gräve</a>
///
/// ROS node that translates motion capture data from an OptiTrack rig to tf transforms.
/// The node receives the binary packages that are streamed by the Arena software,
/// decodes them and broadcasts the poses of rigid bodies as tf transforms.
///
/// Currently, this node supports the NatNet streaming protocol v1.4.

// Local includes
#include "mocap_optitrack/socket.h"
#include "mocap_optitrack/mocap_datapackets.h"
#include "mocap_optitrack/mocap_config.h"
#include "mocap_optitrack/skeletons.h"

// ROS includes
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include "std_msgs/Float32MultiArray.h"

// System includes
#include <string>
#include <unistd.h>

////////////////////////////////////////////////////////////////////////
// Constants

// ip on multicast group - cannot be changed in Arena
const std::string MULTICAST_IP = "224.0.0.1";

const std::string MOCAP_MODEL_KEY = "mocap_model";
const std::string RIGID_BODIES_KEY = "rigid_bodies";
const char ** DEFAULT_MOCAP_MODEL = SKELETON_WITHOUT_TOES;

const int LOCAL_PORT = 1511;
std_msgs::Float32MultiArray array;
////////////////////////////////////////////////////////////////////////

void processMocapData( const char** mocap_model, RigidBodyMap& published_rigid_bodies)
{
  ros::NodeHandle nh;
  ros::Publisher markers_coordinates = nh.advertise<std_msgs::Float32MultiArray>("/markers_coo",1);
  
  UdpMulticastSocket multicast_client_socket( LOCAL_PORT, MULTICAST_IP );
  ushort payload;
  int numberOfPackets = 0;
  while(ros::ok())
  {
    bool packetread = false;
    int numBytes = 0;
    
    do
    {
      // Receive data from mocap device
      numBytes = multicast_client_socket.recv();

      // Parse mocap data
      if( numBytes > 0 )
      {
        const char* buffer = multicast_client_socket.getBuffer();
        unsigned short header = *((unsigned short*)(&buffer[0]));
		
        // Look for the beginning of a NatNet package
        if (header == 7)
        {
          payload = *((ushort*) &buffer[2]);
          MoCapDataFormat format(buffer, payload);


          format.parse();
          packetread = true;
          numberOfPackets++;
         
          
          array.data.clear();
			
          cout << "marker "<< format.model.numOtherMarkers <<endl;
			
		  for(int i=0; i < format.model.numOtherMarkers; i++){
		  	array.data.push_back(format.model.otherMarkers[i].positionX);
			array.data.push_back(format.model.otherMarkers[i].positionY);
			array.data.push_back(format.model.otherMarkers[i].positionZ);
		  }
          
	      for(int i=0; i < format.model.numRigidBodies; i++){
            for(int j=0; j < format.model.rigidBodies[i].NumberOfMarkers; j++){
               array.data.push_back(format.model.rigidBodies[i].marker[j].positionX);
               array.data.push_back(format.model.rigidBodies[i].marker[j].positionY);
               array.data.push_back(format.model.rigidBodies[i].marker[j].positionZ);
              
            }
          }
          
                
          markers_coordinates.publish(array); 
       
   
          if( format.model.numRigidBodies > 0 )
          {
            for( int i = 0; i < format.model.numRigidBodies; i++ )
            {
              int ID = format.model.rigidBodies[i].ID;
              std::cout << "format.model.rigidBodies[i].ID: " << format.model.rigidBodies[i].ID << std::endl;
              RigidBodyMap::iterator item = published_rigid_bodies.find(ID);
       
              ///Conversion to rpy
              //float x = format.model.rigidBodies[i].pose.position.x;
              //float y = format.model.rigidBodies[i].pose.position.y;
              //float z = format.model.rigidBodies[i].pose.position.z;
              //cout<< "x:"<< x << "y:" << y << "z:" << z << endl;

              if (item != published_rigid_bodies.end())
              {
            	  /// send to YouBot based on i

                  item->second.publish(format.model.rigidBodies[i]);
              }
            }
          }
        }
        // else skip packet
      }
    } while( numBytes > 0 );

    // Don't try again immediately
    if( !packetread )
    {
      usleep( 10 );
    }
  }
}



////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[] )
{ 
  
  // Initialize ROS node
  ros::init(argc, argv, "mocap_node");
  ros::NodeHandle n("~");

  
  // Get configuration from ROS parameter server  
  const char** mocap_model( DEFAULT_MOCAP_MODEL );
  if( n.hasParam( MOCAP_MODEL_KEY ) )
  {    std::string tmp;
    if( n.getParam( MOCAP_MODEL_KEY, tmp ) )
    {
      if( tmp == "SKELETON_WITH_TOES" )
        mocap_model = SKELETON_WITH_TOES;
      else if( tmp == "SKELETON_WITHOUT_TOES" )
        mocap_model = SKELETON_WITHOUT_TOES;
      else if( tmp == "OBJECT" )
        mocap_model = OBJECT;
    }
  }

  RigidBodyMap published_rigid_bodies;

  cout << "Has Param?" << std::endl;

  if (n.hasParam(RIGID_BODIES_KEY))
  {
      XmlRpc::XmlRpcValue body_list;
      n.getParam("rigid_bodies", body_list);

      cout << "Has Param!" << std::endl;

      if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0)
      {
          XmlRpc::XmlRpcValue::iterator i;
          for (i = body_list.begin(); i != body_list.end(); ++i) {
              if (i->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                  PublishedRigidBody body(i->second);
                  string id = (string&) (i->first);
                  RigidBodyItem item(atoi(id.c_str()), body);

                  cout << "Body id: " << id << std::endl;

                  std::pair<RigidBodyMap::iterator, bool> result = published_rigid_bodies.insert(item);
                  if (!result.second)
                  {
                      ROS_ERROR("Could not insert configuration for rigid body ID %s", id.c_str());
                  }
              }
          }
      }
  }
	
  // Process mocap data until SIGINT
  processMocapData(mocap_model, published_rigid_bodies);
  

  return 0;
}
