#include "ros/ros.h"
#include <iostream>
#include <json/json.h>
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include "std_msgs/Int32MultiArray.h"
#include <csignal>
#include <time.h>
#include <algorithm>
#include <chrono>

using namespace std;

string DataToJson()
{
	Json::FastWriter write;
	Json::Value writevalue;
	writevalue["val"] = true;
	writevalue["read"] = true;
	string data = write.write(writevalue);
	return data; 
}

void translateJson(string JsonValue)
{
	Json::Reader *JsonTrans = new Json::Reader(Json::Features::strictMode());
	Json::Value readValue;
	JsonTrans->parse(JsonValue, readValue);
	if(readValue.isArray()) {
		int length = readValue.size();
		for(int i = 0; i < length; i++) {
			cout << readValue[i] << endl;
		}
	}
}

#define SERVPort  8001
class handle {
	public:
		handle() {}
		void pub(std_msgs::Int32MultiArray msg) {
			codingMsgPub.publish(msg);
		}
		ros::NodeHandle n;
		ros::Publisher codingMsgPub = n.advertise<std_msgs::Int32MultiArray>("encoder_msg", 500);
		
};

void signalHandle(int signum)
{
	cout << "程序停止" << endl;
	exit(signum);
}
int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "codingpublish");
	ros::NodeHandle nl;
	ros::Publisher codingMsgPub = nl.advertise<std_msgs::Int32MultiArray>("encoder_msg", 500);

	char buffer[256];
	int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(sock_fd < 0) {  
        cout << "socket create failed" << endl;  
        exit(1);  
    }  
    struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(SERVPort);
	//addr.sin_addr.s_addr = htonl(INADDR_ANY);
	// inet_pton(AF_INET, "10.168.1.100", &addr.sin_addr.s_addr);
	addr.sin_addr.s_addr = inet_addr("192.168.1.125");
	socklen_t len = sizeof(addr);
	int ret = bind(sock_fd, (struct sockaddr*)&addr, len);
	if(ret < 0) {
		cout << "bind error!!!!" << endl;
	}

	struct sockaddr_in cli;
	socklen_t length = sizeof(cli);


	Json::Reader *JsonTrans = new Json::Reader(Json::Features::strictMode());
	
	signal(SIGINT, signalHandle);
	while(1) {
		auto start = std::chrono::steady_clock::now();
		int recvf = recvfrom(sock_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cli, &length);
		
		if(recvf == -1) {
			printf("recieve data fail!\n");
			exit(-1);
		}
		Json::Value readValue;
		string coding = buffer;
		if(coding.size() != 0 && coding[1] == ']') {
			coding[1] = '{' ;
		}
		int ret = JsonTrans->parse(coding, readValue);
		
		if(readValue.size() != 0) {
			
			vector<int> codingMsg;
			for(int i = 0; i < 8; i++) {
				if(i == 1 || i == 2 || i == 5 || i == 6) {
					int pv = readValue[i][to_string(i)]["pv"].asInt();
					codingMsg.push_back(pv);	
				} else {
					int vd = readValue[i][to_string(i)]["vd"].asInt();
					codingMsg.push_back(vd);
				}
			}
			std_msgs::Int32MultiArray msg;
			msg.data = codingMsg;
			
			//cout << "msg have published!!!!!" << endl;
			codingMsgPub.publish(msg);
			auto end = std::chrono::steady_clock::now();
			cout << "-------------time = " << std::chrono::duration<double>(end - start).count() << endl;
		} else {
			cout << "11111111111111111111111" << coding << endl;
		}
		
	}
	close(sock_fd);
	return 0;
}

