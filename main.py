import constants
import bb_serial
from bb_serial import cust_print, cust_read
import lidars
import utility
import contour
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import lidarsDrv as lid
import pylab
import warnings
import time

def disp_menu():
    cust_print("")
    cust_print("")
    cust_print('----MENU----') 
    if drv.areConnected == 0:
        cust_print('1: Connect to lidars')
        cust_print('2: Plot datas')
        cust_print('3: Compute circumference')
    else:
        cust_print('1: Disconnect to lidars')
        cust_print('2: Start motors')
        cust_print('3: Stop motors')
        cust_print('4: Launch scan')
        cust_print('5: Plot datas')
        cust_print('6: Compute circumference')
    cust_print('q: Quit')

    
def execute_actions():
    choice = cust_read('    Waiting for input: ')
    if choice == 'q' or choice == 'Q':
        if drv.areConnected == 1:
            drv.disconnect()
            bb_serial.bb_ser.close_bb_serial()
        return True
    
    if drv.areConnected == 0:
        if choice == '1':
            # Connect to lidar
            drv.connect()
        elif choice == '2':
            lidarsSet.read_datas_files()
            lidarsSet.plot_raw_datas()
            if constants.disp_charts:
                cust_print('\nClose all figures to continue...\n')
                plt.show(block=True)
        elif choice == '3':
            # Compute circumference
            compute_algo_circumference()
        else:
            cust_print('        Wrong input!')
    else:
        if choice == '1':
            # Disconnect to lidar
            drv.disconnect()
        elif choice == '2':
            if drv.check_link_state() == 0:
                drv.start_motors()        
        elif choice == '3':
            if drv.check_link_state() == 0:
                drv.stop_motors()       
        elif choice == '4':
            run_scan = False
            try:
                height = int(cust_read('    Please enter patient\'s height in cm: '))
                if height > 0:
                    run_scan = True
            except:
                cust_print('    Incorrect height!')
            # Data scanning
            if run_scan:
                if drv.check_link_state() == 0:
                    drv.scan_datas(height)
        elif choice == '5':
            lidarsSet.read_datas_files()
            lidarsSet.plot_raw_datas()
            if constants.disp_charts:
                cust_print('\nClose all figures to continue...\n')
                plt.show(block=True)
        elif choice == '6':
            compute_algo_circumference()
        else:
            cust_print('        Wrong input!')
    return False            
        
        
def compute_algo_circumference():
    # Time measurement
    start = time.clock() 

    # Read lidars datas
    lidarsSet.read_datas_files()

    # Plot raw datas
    lidarsSet.plot_raw_datas()
    
    # Compute raw datas into merged datas
    lidarsSet.compute_raw_datas()

    # Remove outliers
    if len(utility.mergedPointsXY) > 0:
        #utility.remove_outliers()
        
        # Compute contour
        contour.contour()

        cust_print("Circumference computed in: " + str(time.clock() - start) + ' sec.')

        # Show the plots
        if constants.disp_charts:
            cust_print('\nClose all figures to continue...\n')
            plt.show(block=True)

    else:
        cust_print('    No data point has been read in the expected area!')

# Ignore warnings
warnings.simplefilter("ignore")

# Open bb serial
if constants.use_bb_serial:
    bb_serial.bb_ser = bb_serial.bitbang_serial(constants.bb_serial_RX, constants.bb_serial_TX, constants.bb_baudrate)
   
# Init lidars infos
lidarsSet = lidars.setOfLidars()

# Init lidars
drv = lid.driverLidars()

end = False
while end == False:
    try:
        disp_menu()
        end = execute_actions()
    except:
        cust_print("An error occured")

cust_print("Program ended")



