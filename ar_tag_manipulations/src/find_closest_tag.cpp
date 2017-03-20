#include "ros/ros.h"
#include "ar_tag_manipulations/getClosestTag.h"
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>

#include <map>
#include <math.h>


ros::Subscriber ar_sub;
ros::Time startTime;
bool scanning = false;
uint32_t tagID;
float scanDuration = 0;

struct detectedTag {
    uint32_t nrOfAccesses = 0;
    geometry_msgs::Pose registeredPose;
    geometry_msgs::Pose tagPose;
};

class ARTagManipulator
{
public:
    ros::NodeHandle nh;
    ros::ServiceServer service;
    ros::Subscriber ar_sub;
    ros::Time startTime;
    int scanCounter = 0;
    bool scanning = false;
    uint32_t tagID;
    geometry_msgs::Pose poseOfClosestTag;
    float scanDuration = false;
    std::map<uint32_t, std::vector <detectedTag> > detectedTags; // A map for keeping detected markers for filtering purposes

    ARTagManipulator()
    {
        service = nh.advertiseService("get_closest_tag", &ARTagManipulator::getNearestTag, this);
        ROS_INFO("AR tag manipuator object created");
    }

    void unsubscribeFromAR()
    {
        ROS_INFO("Unsubcribing from /ar_pose_marker topic");
        ar_sub.shutdown();
    }

    void getTagsCallback(const ar_track_alvar_msgs::AlvarMarkers &msg)
    {
        scanCounter++;
        if (msg.markers.size() <= 0)
        {
            ROS_INFO("The message contained no tags");
        }
        else
        {
            //ROS_INFO("Fond %lu tags in the message", msg.markers.size());
            for (int i=0; i<msg.markers.size(); i++)
            {
                geometry_msgs::Pose msgPose = msg.markers[i].pose.pose;

                // Does the map contain this tag?
                uint32_t id = msg.markers[i].id;
                auto search = detectedTags.find(id);
                if(search != detectedTags.end())
                {
                    //ROS_INFO("*-- Its an existing tag ID, comparing with other tags with ID=%u...", id);
                    // Check, if it is the same tag or another one with same ID
                    bool existingEntry = false;
                    for (int j=0; j<detectedTags[id].size(); j++)
                    {
                        //ROS_INFO("*---- j=%d", j);
                        detectedTag* ptrToTag = &(detectedTags[id])[j];
                        if (comparePoints( ptrToTag->registeredPose.position, msgPose.position, 0.02 ))
                        {
                            //ROS_INFO("*---- Found an existing entry, updating...");
                            ptrToTag->tagPose.position.x += msgPose.position.x;
                            ptrToTag->tagPose.position.y += msgPose.position.y;
                            ptrToTag->tagPose.position.z += msgPose.position.z;
                            ptrToTag->nrOfAccesses++;

                            existingEntry = true;
                            break;
                        }
                    }

                    if (!existingEntry)
                    {
                        ROS_INFO("*---- Its a new tag, adding...");
                        detectedTag newTag;
                        newTag.nrOfAccesses = 1;
                        newTag.tagPose = msgPose;
                        newTag.registeredPose = msgPose;
                        detectedTags[id].push_back(newTag);
                    }
                }
                else
                {
                    ROS_INFO("*-- New tag with ID=%u detected, adding...", id);
                    detectedTag newTag;
                    newTag.nrOfAccesses = 1;
                    newTag.tagPose = msgPose;
                    newTag.registeredPose = msgPose;
                    detectedTags[id].push_back(newTag);
                }
            }
        }
        //std::cout << std::endl;

        if ((ros::Time::now().toSec() - startTime.toSec()) >= scanDuration)
        {
            unsubscribeFromAR();
            scanning = false;

            // If there were any tags detected, then find the closest one
            if (detectedTags.size() > 0)
            {
                // Calculate the average abd then find the closest tag
                float closestDist = 999999999999999;
                for (auto& ent1 : detectedTags)
                {
                    ROS_INFO("Detected %lu tags with ID=%u",ent1.second.size() , ent1.first);
                    for (int i=0; i<ent1.second.size(); i++)
                    {
                        detectedTag* ptrToTag = &ent1.second[i];

                        // Firstly, calculate the mean position
                        ptrToTag->tagPose.position.x /= ptrToTag->nrOfAccesses;
                        ptrToTag->tagPose.position.y /= ptrToTag->nrOfAccesses;
                        ptrToTag->tagPose.position.z /= ptrToTag->nrOfAccesses;

                        // Check validity. Valid tags are ones that have been detected at least 70% of the time
                        // with no errors caused by averaging.
                        bool tagReliableDetection = (ptrToTag->nrOfAccesses >= scanCounter*0.7);
                        bool tagReliableAverage = comparePoints( ptrToTag->registeredPose.position, ptrToTag->tagPose.position, 0.02 );
                        bool reliabilityResult = tagReliableDetection && tagReliableAverage;

                        if (reliabilityResult)
                        {
                            ROS_INFO("*-- tag with subID=%d is reliable", i);
                        }
                        else
                        {
                            if (!tagReliableDetection)
                                ROS_ERROR("*-- tag with subID=%d  unreliable. Reason: UNSTABLE DETECTION", i);

                            if (!tagReliableAverage)
                                ROS_ERROR("*-- tag with subID=%d  unreliable. Reason: CALCULATION ERROR", i);
                        }

                        // Secondly, calculate the distance from the camera
                        float distance = sqrt( pow(ptrToTag->tagPose.position.x, 2) +
                                               pow(ptrToTag->tagPose.position.y, 2) +
                                               pow(ptrToTag->tagPose.position.z, 2) );

                        // Thirdly, check, if it is the closest registered distance
                        if ( (distance < closestDist) && reliabilityResult)
                        {
                            tagID = ent1.first;
                            poseOfClosestTag = ptrToTag->tagPose;
                            closestDist = distance;
                        }
                    }
                }
            }
            detectedTags.clear();
        }
    }

    void subscribeToAR()
    {
        ROS_INFO("Subcribing to /ar_pose_marker topic");
        scanning = true;
        scanCounter = 0;
        startTime = ros::Time::now();
        ar_sub = nh.subscribe("/ar_pose_marker", 10, &ARTagManipulator::getTagsCallback, this);
    }

    // Function for comparing points in 3D space. Returns true, if the distance between points is less than "allowedDiff"
    bool comparePoints(const geometry_msgs::Point point1, const geometry_msgs::Point point2, float allowedDiff)
    {
        return ( sqrt( pow(point1.x - point2.x, 2) +
                       pow(point1.y - point2.y, 2) +
                       pow(point1.z - point2.z, 2)) < allowedDiff);
    }

    bool getNearestTag(ar_tag_manipulations::getClosestTag::Request &req,
                     ar_tag_manipulations::getClosestTag::Response &res)
    {
        scanDuration = req.scanDuration;
        tagID = 999;
        subscribeToAR();
        ROS_INFO("Listening to /ar_pose_marker topic for %f seconds", scanDuration);

        while(scanning && ros::ok())
            ros::spinOnce();

        res.tagID = tagID;
        res.tagPose = poseOfClosestTag;
        ROS_INFO("responded to a request with: %d", res.tagID);
        return true;
    }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "closest_ar_tag_finder");

    ARTagManipulator artm;
    ROS_INFO("Ready to serve.");
    while(ros::ok())
         ros::spinOnce();

    return 0;
}

/*
std::map<std::string, std::map<std::string, std::string>> mymap;

for(auto const& ent1 : mymap) {
  // ent1.first is the first key
  for(auto const& ent2 : ent1.second) {
    // ent2.first is the second key
    // ent2.second is the data
  }
}
*/
