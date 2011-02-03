#!/usr/bin/python

PKG="dmp_motion_generation"
import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag

if __name__ == "__main__":
    
    rospy.init_node('dmp_bagfile_parser')

    directory='/u/pastor/tmp/dmp_2/'
    dmp_id=1
    num_trials=1
    num_roll_outs=8
    transformation_system_ids=[0,1,7]

    for i in range(1,num_trials+1):
        for j in range(0,num_roll_outs):
            
            bag_file_name = directory + 'dmp_' + str(dmp_id) + '_trial_' + str(i) + '_rollout_' + str(j) + '.bag'
            bag = rosbag.Bag(bag_file_name)

            print bag_file_name

            filename = directory + 'thetas_' + str(dmp_id) + '_trial_' + str(i) + '_rollout_' + str(j) + '.txt'
            f=open(filename, "w");

            for (topic, msg, t) in bag.read_messages():

                for t_id in transformation_system_ids:
                    thetas = msg.transformation_systems[t_id].lwr_model.thetas
                    thetas_string = str(thetas).strip("(").strip(")") + '\n'
                    f.write(thetas_string);

                    print thetas_string

            f.close()


    
