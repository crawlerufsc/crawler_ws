# import pandas as pd
# import sqlite3

# # Read sqlite query results into a pandas DataFrame
# con = sqlite3.connect("/content/drive/My Drive/test02_0.db3")
# df = pd.read_sql_query("SELECT * from messages", con)

# # Verify that result of SQL query is stored in the dataframe
# print(df.head())

# con.close()


import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import pandas as pd

import matplotlib.pyplot as plt

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]



if __name__ == "__main__":

        # bag_file = '/workspaces/foxy_leg_ws/rosbag2_2021_04_28-04_42_07/rosbag2_2021_04_28-04_42_07_0.db3'
        dir = "../bag_files/1511/test03/"
        bag_file = dir+'test151103_0.db3'

        parser = BagFileParser(bag_file)
        print(parser.topics_data)
        odom  = parser.get_messages("/odom")
        motor = parser.get_messages("/motor_state")
        servo = parser.get_messages("/servo_state")
        imu   = parser.get_messages("/imu")
        
        print(len(imu))
        print(imu[0][1])
        # print(type(odom[0][1]))
        # print(odom[0][1].header.stamp)
        odom_df  = pd.DataFrame(index=range(len(odom)),  columns=["first ts", "sec_ts_sec", "sec_ts_nanosec", "id", "value"])
        motor_df = pd.DataFrame(index=range(len(motor)), columns=["first ts", "sec_ts_sec", "sec_ts_nanosec", "id", "value"])
        servo_df = pd.DataFrame(index=range(len(servo)), columns=["first ts", "sec_ts_sec", "sec_ts_nanosec", "id", "value"])
        
        imu_columns = ["first ts", "sec_ts_sec", "sec_ts_nanosec", "id"]
        ori = ['ori_x', 'ori_y' , 'ori_z', 'ori_w']
        ori_covs = ["ori_cov"+str(i) for i in range(9)]
        ang_vel = ['ang_vel_x', 'ang_vel_y' , 'ang_vel_z']
        ang_vel_covs = ["ang_vel_cov"+str(i) for i in range(9)]
        lin_acc = ['lin_acc_x', 'lin_acc_y' , 'lin_acc_z']
        lin_acc_covs = ["lin_acc_cov"+str(i) for i in range(1,10)]
        imu_columns = imu_columns + ori+ori_covs+ang_vel+ang_vel_covs+lin_acc+lin_acc_covs
        imu_df = pd.DataFrame(index=range(len(imu)), columns=imu_columns)
        
        for i in range(len(odom)):
            odom_df.at[i] = [odom[i][0], odom[i][1].header.stamp.sec, odom[i][1].header.stamp.nanosec, odom[i][1].header.frame_id, odom[i][1].value]
        
        for i in range(len(motor)):
            motor_df.at[i] = [motor[i][0], motor[i][1].header.stamp.sec, motor[i][1].header.stamp.nanosec, motor[i][1].header.frame_id, motor[i][1].value]
        
        for i in range(len(servo)):
            servo_df.at[i] = [servo[i][0], servo[i][1].header.stamp.sec, servo[i][1].header.stamp.nanosec, servo[i][1].header.frame_id, servo[i][1].value]
        
        for i in range(len(imu)):
            row = [imu[i][0], imu[i][1].header.stamp.sec, imu[i][1].header.stamp.nanosec, imu[i][1].header.frame_id]
            row+= [imu[i][1].orientation.x, imu[i][1].orientation.y, imu[i][1].orientation.z, imu[i][1].orientation.w]
            row+= imu[i][1].orientation_covariance.tolist()
            row+= [imu[i][1].angular_velocity.x, imu[i][1].angular_velocity.y, imu[i][1].angular_velocity.z]
            row+= imu[i][1].angular_velocity_covariance.tolist()
            row+= [imu[i][1].linear_acceleration.x, imu[i][1].linear_acceleration.y, imu[i][1].linear_acceleration.z]
            row+= imu[i][1].linear_acceleration_covariance.tolist()
            imu_df.at[i] = row
        
        print(odom_df)
        print(motor_df)
        print(servo_df)
        print(imu_df)
        
        odom_df.to_pickle(dir+"odom.pkl")
        motor_df.to_pickle(dir+"motor.pkl")
        servo_df.to_pickle(dir+"servo.pkl")
        imu_df.to_pickle(dir+"imu.pkl")
        
        output = pd.read_pickle(dir+"odom.pkl")
        print(output)

            
        # trajectory = parser.get_messages("/joint_trajectory")[0][1] 
        # p_des_1 = [trajectory.points[i].positions[0] for i in range(len(trajectory.points))]
        # t_des = [trajectory.points[i].time_from_start.sec + trajectory.points[i].time_from_start.nanosec*1e-9 for i in range(len(trajectory.points))]

        # actual = parser.get_messages("/joint_states")

        # plt.plot(t_des, p_des_1)

        # plt.show()