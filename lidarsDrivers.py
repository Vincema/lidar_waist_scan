from rplidar import RPLidar

# Documentation
class driverLidars:
        def __init__(self):
                self.lidars = []
                for i in range(3):
                        self.lidars.append(RPLidar(constants,constants.serialPort[i],baudrate=115200))
                        if self.lidars[i].getHealth() <> 0:
                                print('Cannot connect to the lidar ' , i , '!')
                        
                



class Lidar 
{
	public: 
		int mLidarNb;
		RPlidarDriver* mDrv;
		const char* mPortName;
		char mOutputFile[100];
		double mDatasArray[NB_OF_DATAS_TO_RETRIEVE][2];
		int mDatasLeft;
		
		Lidar();
		Lidar(int lidarNb, const char* portName, const char* outputDir); 
		void connect();
		void print_output_file();
		int get_single_data_sample();
};

class LidarsSet
{
	public:
		Lidar mLidar[3]; 

		LidarsSet();
		LidarsSet(const char* portName[3], const char* outputDir);
		void toggle_motors();
		void start_motors();
		void stop_motors();
		void retrieve_datas();
		bool mAreSpinning;
};

#endif
