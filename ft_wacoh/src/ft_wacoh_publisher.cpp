#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Wrench.h>

#include <math.h>
#include <stdio.h> 
#include <stdlib.h>
#include <vector>
#include <numeric> // computes mean value of vector

#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>  //internet domain addresses, struct hostent

// Sensor's total number of steps for all axis
#define STEP_SZ 8193
// Sensitivy for the sensor's "step"
#define SENSITIVITY_Fx 26.98
#define SENSITIVITY_Fy 28.68
#define SENSITIVITY_Fz 31.70
#define SENSITIVITY_Mx 1530.40
#define SENSITIVITY_My 1500.45
#define SENSITIVITY_Mz 1155.94

// Flags
int initialBias=1;
int biasTime=10; 		// Amount of time (secs) to use to compute offset
int ctr=0; 
/************************************************* Global Functions *************************************************/
// Print Error
void error(const char *msg) { perror(msg);exit(0); }

// ROS callback function to check for user flag correspdonding to activating FT sensor bias.
void correctBias(const std_msgs::Int16& flag) { initialBias=flag.data; }

int main(int argc, char **argv) {
  // Transport parameters
  int sockfd, portno, nc; 					//sockfd,  variables store the values returned by the socket system call and the accept system call.
  struct sockaddr_in serv_addr;
  struct hostent *server;

  // wrenchBiasAdjustment
  double tempOffsetBuffer[6]={0};  // 6x total number elements to be saved during the averaging period
  geometry_msgs::Wrench offset; 	 // initialized to zero

  //struct  hostent
  // {
  //    char    *h_name;        /* official name of host */
  //   char    **h_aliases;    /* alias list */
  //  int     h_addrtype;     /* host address type */
  // int     h_length;       /* length of address */
  // char    **h_addr_list;  /* list of addresses from name server */
  //  #define h_addr  h_addr_list[0]  /* address, for backward compatiblity */
  //};
  // */

  // Fixed default ip address for connection and port number:
  char a[]="10.0.0.5";
  char b[]="5001";

  char buffer[256];
  /*  if (argc < 3) {
      fprintf(stderr,"usage %s hostname port\n", argv[0]);
      exit(0);
      }*/

  portno = atoi(b);
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) 
    ROS_ERROR("ERROR opening socket");
  server = gethostbyname(a);

  // Takes such a name as an argument and returns a pointer to a hostent containing information about that host
  if (server == NULL) {
    ROS_ERROR("ERROR, no such host. Exiting the program!\n");
    exit(0);
  }

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, 
        (char *)&serv_addr.sin_addr.s_addr,
        server->h_length);
  serv_addr.sin_port = htons(portno);

  // The connect function is called by the client to establish a connection to the server
  if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
    ROS_ERROR("ERROR connecting");

  // Can request user to enter a certain command.
  // printf("Please enter the message: ");

  bzero(buffer,256);
  //fgets(buffer,255,stdin);
  //uses fgets to read the message from stdin
    
  nc = write(sockfd,"OOO",3);
  //writes the message to the socket

  if (nc < 0) 
    ROS_ERROR("ERROR writing to socket");


  // ROS Wrapper  ======================================================================

  ros::init(argc,argv,"ft_wacoh"); 	// name of this node will be ""Forcesensor_publisher"
  ros::NodeHandle n; 															                                              
  ros::Publisher  wrench_pub = n.advertise<geometry_msgs::Wrench>("/wrench/unbiased",100);      // wrench with no offset
  ros::Publisher  offset_pub = n.advertise<geometry_msgs::Wrench>("/wrench/bias/",100);         // computed offset
  ros::Publisher  offsetWrench_pub = n.advertise<geometry_msgs::Wrench>("/wrench/biased",100);  // wrench with offset
  ros::Subscriber wrench_sub = n.subscribe("/wrench/bias",1,correctBias);

  // Wrench and String type.
  geometry_msgs::Wrench myWrench,myWrench_unbiased,myWrench_biased; 

  // ROS Rates
  int rate=200;
  double period=rate*biasTime;          // Total time for averaging period to calculate offset
  ros::Rate loopRate(rate); 	       		// Create a ros::Rate object. Set the time for a 1Hz sleeper timer.
  int i,num,temp;

  while(ros::ok()) //work loop		        
    {
      // Writes the message to the socket
      nc = write(sockfd,"R",1);

      if (nc < 0) 
        ROS_ERROR("ERROR writing to socket");  
    
      //clear buffer to zero
      bzero(buffer,256);

      //read data from socket,255 is the size of the message
      nc = read(sockfd,buffer,64);

      if (nc < 0) 
        ROS_ERROR("ERROR reading from socket");

      // Print buffer info in HEX
      std_msgs::String mystring;
      ROS_DEBUG("%s\n",buffer); //debug the buffer string
      mystring.data = buffer;  
      ROS_DEBUG("%s\n\n",buffer);

      for(i=1; i<25; i++)
        {

          if(buffer[i]>='0'&&buffer[i]<='9') 
            temp = buffer[i] - '0';//tranfer ascii to decimal number
          else if(buffer[i]>='A'&&buffer[i]<='F')
            temp = buffer[i] - 'A'+10;///tranfer ascii to decimal number
          else
            temp = 0;
      

          // Parameter computation
          if(i/4 == 0)//first data
            {
              if(i%4 == 1)
                myWrench.force.x = 0;
              myWrench.force.x=temp*(pow(16,3-(i-1)%4))+ myWrench.force.x;
              //raw data
              //tranformed data    
            }
          else if(i/4 == 1)//second data
            {
              if(i%4 == 1)
                myWrench.force.y = 0;
              myWrench.force.y=temp*(pow(16,3-(i-1)%4))+ myWrench.force.y;
            }
          else if(i/4 == 2)//third data
            {
              if(i%4 == 1)
                myWrench.force.z = 0;         
              myWrench.force.z=temp*(pow(16,3-(i-1)%4))+ myWrench.force.z;
            }
          else if(i/4 == 3)//fourth data
            {
              if(i%4 == 1)
                myWrench.torque.x = 0;
              myWrench.torque.x=temp*(pow(16,3-(i-1)%4))+ myWrench.torque.x;
            }
          else if(i/4 == 4)//fifth data
            {
              if(i%4 == 1)
                myWrench.torque.y = 0;
              myWrench.torque.y=temp*(pow(16,3-(i-1)%4))+ myWrench.torque.y;
            }
          else if(i/4 == 5)//sixth data
            {
              if(i%4 == 1)
                myWrench.torque.z = 0;
              myWrench.torque.z=temp*(pow(16,3-(i-1)%4))+ myWrench.torque.z;	  
            }
        }
      
      // Unbiased Wrench (no-offset)
      // 8193 is number of steps steps in sensor. SENSITIVITY_XX means the sensitivity for that step.
      myWrench_unbiased.force.x  = (myWrench.force.x-STEP_SZ)/SENSITIVITY_Fx;
      myWrench_unbiased.force.y  = (myWrench.force.y-STEP_SZ)/SENSITIVITY_Fy;
      myWrench_unbiased.force.z  = (myWrench.force.z-STEP_SZ)/SENSITIVITY_Fz;
      myWrench_unbiased.torque.x = (myWrench.torque.x-STEP_SZ)/SENSITIVITY_Mx;
      myWrench_unbiased.torque.y = (myWrench.torque.y-STEP_SZ)/SENSITIVITY_My;
      myWrench_unbiased.torque.z = (myWrench.torque.z-STEP_SZ)/SENSITIVITY_Mz;

      // Offset set only if flag initialBias is true and the averaging period has been completed
      myWrench_biased.force.x  = offset.force.x  + myWrench_unbiased.force.x;
      myWrench_biased.force.y  = offset.force.y  + myWrench_unbiased.force.y;
      myWrench_biased.force.z  = offset.force.z  + myWrench_unbiased.force.z;
      myWrench_biased.torque.x = offset.torque.x + myWrench_unbiased.torque.x;
      myWrench_biased.torque.y = offset.torque.y + myWrench_unbiased.torque.y;
      myWrench_biased.torque.z = offset.torque.z + myWrench_unbiased.torque.z;

      // Check for bias adjustment (this should only be done at the beginning of the task once)
      if(initialBias) 
        {
          ROS_INFO_STREAM("Computing offset. Please wait for " << period << " seconds before offset is activated.");
        
          if (ctr<period) {// keep 10 seconds of data
            tempOffsetBuffer[0]+=myWrench_unbiased.force.x;
            tempOffsetBuffer[1]+=myWrench_unbiased.force.y;
            tempOffsetBuffer[2]+=myWrench_unbiased.force.z;
            tempOffsetBuffer[3]+=myWrench_unbiased.torque.x;
            tempOffsetBuffer[4]+=myWrench_unbiased.torque.y;
            tempOffsetBuffer[5]+=myWrench_unbiased.torque.z;  
            
            // Increment counter
            ctr++;
          }              
          // Compute the average and set it to offset
          else if (ctr==period) {           
            offset.force.x  = tempOffsetBuffer[0]/ctr;
            offset.force.y  = tempOffsetBuffer[1]/ctr;
            offset.force.z  = tempOffsetBuffer[2]/ctr;
            offset.torque.x = tempOffsetBuffer[3]/ctr;
            offset.torque.y = tempOffsetBuffer[4]/ctr;
            offset.torque.z = tempOffsetBuffer[5]/ctr;

            ROS_INFO("Offset has been computed.");
            ROS_INFO_STREAM("The wrench offset has a value of: " << offset << std::endl);
            
            // Reset flags
            initialBias=0;
            ctr=0;
          }
        }

      // Publish all wrench related info: unbiased, offset, and biased.
      wrench_pub.publish(myWrench_unbiased);
      offset_pub.publish(offset);
      offsetWrench_pub.publish(myWrench_biased);

      loopRate.sleep();			  
      ros::spinOnce();
    }

  close(sockfd); //close the socket
  return 0;
}
