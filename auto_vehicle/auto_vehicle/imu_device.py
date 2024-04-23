import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from auto_vehicle.lib import IMU
import datetime
import math
import tf_transformations

class IMUDevice(Node):

    RAD_TO_DEG = 57.29578
    M_PI = 3.14159265358979323846
    G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
    AA =  0.40      # Complementary filter constant

    ################# Compass Calibration values ############
    # Use calibrateBerryIMU.py to get calibration values
    # Calibrating the compass isnt mandatory, however a calibrated
    # compass will result in a more accurate heading values.

    magXmin =  0
    magYmin =  0
    magZmin =  0
    magXmax =  0
    magYmax =  0
    magZmax =  0
    ############### END Calibration offsets #################

    gyroXangle = 0.0
    gyroYangle = 0.0
    gyroZangle = 0.0
    CFangleX = 0.0
    CFangleY = 0.0

    a = datetime.datetime.now()
    
    def __init__(self):
        super().__init__('imu_device')

        self.imu_publisher = self.create_publisher(Imu, 'auto_vehicle/imu', 10)

        self.timer = self.create_timer(1.0 / 60.0, self.timer_callback)

        IMU.detectIMU()

        if(IMU.BerryIMUversion == 99):
            self.get_logger().info(" No BerryIMU found... ")

        IMU.initIMU()

    def timer_callback(self):
        #Read the accelerometer,gyroscope and magnetometer values
        ACCx = IMU.readACCx()
        ACCy = IMU.readACCy()
        ACCz = IMU.readACCz()
        GYRx = IMU.readGYRx()
        GYRy = IMU.readGYRy()
        GYRz = IMU.readGYRz()
        MAGx = IMU.readMAGx()
        MAGy = IMU.readMAGy()
        MAGz = IMU.readMAGz()

        #Apply compass calibration
        MAGx -= (self.magXmin + self.magXmax) / 2
        MAGy -= (self.magYmin + self.magYmax) / 2
        MAGz -= (self.magZmin + self.magZmax) / 2

        ##Calculate loop Period(LP). How long between Gyro Reads
        self.b = datetime.datetime.now() - self.a
        self.a = datetime.datetime.now()
        LP = self.b.microseconds/(1000000*1.0)
        outputString = "Loop Time %5.2f " % ( LP )

        #Convert Gyro raw to degrees per second
        rate_gyr_x =  GYRx * self.G_GAIN
        rate_gyr_y =  GYRy * self.G_GAIN
        rate_gyr_z =  GYRz * self.G_GAIN

        #Calculate the angles from the gyro.
        self.gyroXangle+=rate_gyr_x*LP
        self.gyroYangle+=rate_gyr_y*LP
        self.gyroZangle+=rate_gyr_z*LP

        #Convert Accelerometer values to degrees
        AccXangle =  (math.atan2(ACCy,ACCz)*self.RAD_TO_DEG)
        AccYangle =  (math.atan2(ACCz,ACCx)+self.M_PI)*self.RAD_TO_DEG

        #convert the values to -180 and +180
        if AccYangle > 90:
            AccYangle -= 270.0
        else:
            AccYangle += 90.0

        #Complementary filter used to combine the accelerometer and gyro values.
        self.CFangleX=self.AA*(self.CFangleX+rate_gyr_x*LP) +(1 - self.AA) * AccXangle
        self.CFangleY=self.AA*(self.CFangleY+rate_gyr_y*LP) +(1 - self.AA) * AccYangle

        #Calculate heading
        heading = 180 * math.atan2(MAGy,MAGx)/self.M_PI

        #Only have our heading between 0 and 360
        if heading < 0:
            heading += 360

        ####################################################################
        ###################Tilt compensated heading#########################
        ####################################################################
        #Normalize accelerometer raw values.
        accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

        #Calculate pitch and roll
        pitch = math.asin(accXnorm)
        roll = -math.asin(accYnorm/math.cos(pitch))

        #Calculate the new tilt compensated values
        #The compass and accelerometer are orientated differently on the the BerryIMUv1, v2 and v3.
        #This needs to be taken into consideration when performing the calculations

        #X compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
        else:                                                                #LSM9DS1
            magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)

        #Y compensation
        if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
        else:                                                                #LSM9DS1
            magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)

        #Calculate tilt compensated heading
        tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/self.M_PI

        if tiltCompensatedHeading < 0:
            tiltCompensatedHeading += 360
        ##################### END Tilt Compensation ########################

        imu_msg = Imu()

        quat = tf_transformations.quaternion_from_euler(0, 0, math.radians(self.gyroZangle))
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        imu_msg.linear_acceleration.x = ((ACCx * 0.244) / 1000) * 9.81
        imu_msg.linear_acceleration.y = ((ACCy * 0.244) / 1000) * 9.81
        imu_msg.linear_acceleration.z = ((ACCz * 0.244) / 1000) * 9.81

        self.imu_publisher.publish(imu_msg)

def main():
    rclpy.init()

    imuDevice = IMUDevice()

    rclpy.spin(imuDevice)

    imuDevice.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()