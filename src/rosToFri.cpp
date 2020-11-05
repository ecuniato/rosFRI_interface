#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/time.h>
#include <netdb.h>

using namespace std;

typedef struct ROBOT_STATE {
  double jstate[7];
}ROBOT_STATE;

typedef struct ROBOT_CMD {
  double jcmd[7];
}ROBOT_CMD;

//Creazione socket in LETTURA
inline bool listener_socket(int port_number, int *sock) {
      sockaddr_in si_me;

  if ( (*sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    std::cout << "Listener::Open: error during socket creation!" << std::endl;
    return false;
  }

  memset((char *) &si_me, 0, sizeof(si_me));

  /* allow connections to any address port */
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port_number);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  int bind_ok = bind(*sock, (struct sockaddr*)&si_me, sizeof(si_me));

  if ( bind_ok == -1 )
    return false;
  else
    return true;

}

//Creazione socket in SCRITTURA
inline int create_socket(char* dest, int port, int *sock) {
  struct sockaddr_in temp;
  struct hostent *h;
  int error;

  temp.sin_family=AF_INET;
  temp.sin_port=htons(port);
  h=gethostbyname(dest);

  if (h==0) {
    printf("Gethostbyname fallito\n");
    exit(1);
  }

  bcopy(h->h_addr,&temp.sin_addr,h->h_length);
  *sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  error=connect(*sock, (struct sockaddr*) &temp, sizeof(temp));
  return error;
}

class ROS2FRI {
	public:
		ROS2FRI();
		void run();
		void get_jstate();
		void jointPos_command(const std_msgs::Float64MultiArrayConstPtr& );
	private:
		ros::NodeHandle _nh;
		ros::Subscriber _joints_sub;
		ros::Publisher _joints_pub;
		int _jstate_socket;
		int _jcommand_socket;
};

ROS2FRI::ROS2FRI() {
	_joints_sub = _nh.subscribe("/iiwa/jointsCommand", 0, &ROS2FRI::jointPos_command, this);
	_joints_pub = _nh.advertise<sensor_msgs::JointState>("/iiwa/joint_states", 0);

	listener_socket(9030, &_jstate_socket);
	create_socket("172.31.1.145",9031,&_jcommand_socket);
}

void ROS2FRI::jointPos_command(const std_msgs::Float64MultiArrayConstPtr& commands) { //read commands from topic
	ROBOT_CMD rc;
	for (int i=0; i<7; i++)
		rc.jcmd[i] = commands->data[i];

	write( _jcommand_socket, &rc, sizeof(rc) ); //write commands over socket
}

void ROS2FRI::get_jstate() {
	int slen, rlen;
	sockaddr_in si_me, si_other;
	ROBOT_STATE rs;
	while( ros::ok()) {
		rlen = recvfrom( _jstate_socket, &rs, sizeof(rs),0,(struct sockaddr*)&si_other, (socklen_t*)&slen);
		/*cout << "Jstate: ";
		for(int i=0; i<7; i++ ) cout << rs.jstate[i] << " ";
		cout << endl; */
		if(rlen>0) {
			sensor_msgs::JointState jstates;
			jstates.header.stamp = ros::Time::now();
			for (int i=1; i<=7; i++) {
				jstates.name.push_back("iiwa_joint_"+to_string(i));
				jstates.position.push_back(rs.jstate[i-1]);
				jstates.velocity.push_back(0);
				jstates.effort.push_back(0);
			}
			_joints_pub.publish(jstates);
		}
	}
}

void ROS2FRI::run() {
	//boost::thread ctrl_loop_t ( &ROS2FRI::ctrl_loop, this);
	boost::thread get_jstate( &ROS2FRI::get_jstate, this);
	ros::spin();
}


int main(int argc, char** argv) {

	ros::init(argc, argv, "rosToFri");

	ROS2FRI kc;
	kc.run();

	return 0;
}
