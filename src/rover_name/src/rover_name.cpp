//program listens to nameListPre topic to get all available swarmie names
//program publishes on nameList topic "count swarmie1 swarmie2 ....."
/*////////////////////////////////////////////////

Code currently working. In future include update to detect if swarmie is no longer connected
Swarmies should automatically update to new namelist, if this publishes correct one

complementary code in ROSAdapter::nameListHandler();

////////////////////////////////*/
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <random_numbers/random_numbers.h>

//Publishers
ros::Publisher roverNamePublisher;

//Subscribers
ros::Subscriber nameSubscriber;

// Random number generator
random_numbers::RandomNumberGenerator* rng;

//Callback handlers
//void nameDecideHandler (std_msgs::String::ConstPtr& name);
void nameList (const std_msgs::String::ConstPtr& name);

//other Functions
void printList(void);

int count = 0; //how many swarmies are connected
std::string names[10];//hostnames of swarmies

int main(int argc, char **argv){
  ros::init(argc, argv, "MASTER_NAMELIST");//only the last robot initalized will run this node
  ros::NodeHandle n;

  ros::Rate rate(20);

  nameSubscriber = n.subscribe("/nameListPre", 100, nameList);
  roverNamePublisher = n.advertise<std_msgs::String>(("/nameList"), 1, true);

  while(ros::ok()){

    ros::spinOnce();
    printList();
    rate.sleep();
  }
  return EXIT_SUCCESS;
}

void nameList (const std_msgs::String::ConstPtr& name){//not finding dupes
  int pass = 1;

  for(int i = 0; i < count; i++){
    if(names[i]==name->data){
      pass = 0;
      break;//if name is already in list ignore it
    }
  }
  if(pass){//if name is not in list add it and inc count
    names[count] = name->data;
    count++;
  }
}

void printList(void){
  std_msgs::String list;
  std::stringstream ss;
  ss.str(std::string());
  ss << count << " ";//gives count followed by names
  for(int i = 0; i < count; i++){
    ss << names[i] << " ";
  }
  list.data = ss.str();
  roverNamePublisher.publish(list);
}
